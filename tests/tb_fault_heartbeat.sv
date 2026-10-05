`timescale 1ns/1ps
// Fault injection removes ADC completion pulses only. All watchdog, protection
// and UART logic runs normally, including the CRC-protected v1 wire format.
module tb_fault_heartbeat;
    reg clk=0;
    always #10 clk=~clk;
    reg rst_n=0;
    reg [3:0] key_sw=4'b1111;
    reg [9:0] adc=624;
    wire uart_tx, adc_clk, adc_oe_n, an1, an2, pwma, bn1, bn2, pwmb;
    top #(.TELEMETRY_EXTENDED(0),.KEY_CYCLES(4),.ADC_SAMPLE_CYCLES(10),.SAMPLE_WATCHDOG_CYCLES(1000)) dut (
        .clk_50m(clk),.rst_n(rst_n),.key_sw(key_sw),.uart_rx(1'b1),.uart_tx(uart_tx),
        .adc_data_in(adc),.adc_otr(1'b0),.adc_clk(adc_clk),.adc_oe_n(adc_oe_n),
        .enc1_a(1'b0),.enc1_b(1'b0),.AN1(an1),.AN2(an2),.PWMA(pwma),
        .BN1(bn1),.BN2(bn2),.PWMB(pwmb));
    defparam dut.u_telemetry.BAUD=1000000;
    wire [7:0] rx_data;
    wire rx_valid, rx_error;
    uart_rx_byte #(.BAUD(1000000)) monitor (
        .clk(clk),.rst_n(rst_n),.rx(uart_tx),.data(rx_data),.valid(rx_valid),.framing_error(rx_error));
    reg [7:0] packet[0:23];
    reg [15:0] crc_work;
    integer packet_index=0, packet_count=0, fault_frames=0, k, timeout;
    function [15:0] crc_byte(input [15:0] crc,input [7:0] value);
        reg [15:0] work;
        integer n;
        begin
            work=crc^{value,8'd0};
            for(n=0;n<8;n=n+1) work=work[15] ? (work<<1)^16'h1021 : work<<1;
            crc_byte=work;
        end
    endfunction
    always @(posedge clk) begin
        if(rst_n && rx_error) $fatal(1,"heartbeat framing error");
        if(rst_n && rx_valid) begin
            packet[packet_index]=rx_data;
            if(packet_index==23) begin
                if({packet[0],packet[1],packet[2],packet[3]}!==32'haa550118)
                    $fatal(1,"heartbeat header changed");
                crc_work=16'hffff;
                for(k=0;k<22;k=k+1) crc_work=crc_byte(crc_work,packet[k]);
                if({packet[23],packet[22]}!==crc_work) $fatal(1,"heartbeat CRC");
                if(packet[18]==3 && packet[20]==8'h04) begin
                    if({packet[7],packet[6]}!==16'd624 ||
                       $signed({packet[9],packet[8]})<3200 ||
                       $signed({packet[9],packet[8]})>3217 || packet[21]!==8'h90)
                        $fatal(1,"watchdog heartbeat lost last state or start admission");
                    fault_frames=fault_frames+1;
                end
                packet_index=0; packet_count=packet_count+1;
            end else packet_index=packet_index+1;
        end
    end
    task ticks(input integer count);
        begin repeat(count) begin @(posedge clk); #2; end end
    endtask
    // Keep manual movement within each ADC window and wait for trustworthy
    // velocity history; instantaneous jumps are reserved for fault injection.
    task move_adc(input integer target);
        integer remaining;
        begin
            while (adc != target) begin
                @(negedge clk);
                remaining=target-$signed({1'b0,adc});
                if (remaining>16) adc=adc+16;
                else if (remaining < -16) adc=adc-16;
                else adc=target;
                ticks(160);
            end
            ticks(5120);
        end
    endtask
    task key(input integer index);
        begin
            @(negedge clk); key_sw[index]=0; ticks(10);
            @(negedge clk); key_sw[index]=1; ticks(10);
        end
    endtask
    initial begin #3000000; $fatal(1,"tb_fault_heartbeat timeout"); end
    initial begin
        ticks(5); @(negedge clk); rst_n=1; ticks(1000); key(0);
        move_adc(800); key(3); ticks(3000);
        if(!dut.calibrated || dut.sensor_fault) $fatal(1,"heartbeat setup calibration");
        move_adc(624); key(1); ticks(3000);
        if(dut.state!==1 || !dut.motor_enable) $fatal(1,"heartbeat setup start");
        while(packet_count<1) ticks(10);
        @(negedge clk); force dut.adc_valid=1'b0; ticks(1020);
        if(dut.state!==3 || dut.fault!==8'h04 ||
           {an1,an2,pwma,bn1,bn2,pwmb}!==6'b001001)
            $fatal(1,"acquisition outage did not stop both motor outputs");
        timeout=0;
        while(fault_frames<3 && timeout<100000) begin ticks(10); timeout=timeout+10; end
        if(fault_frames<3) $fatal(1,"watchdog stopped motor but its telemetry stopped too");
        key(2);
        if(dut.state!==0 || dut.motor_enable) $fatal(1,"SW3 failed during acquisition outage");
        release dut.adc_valid;
        $display("PASS tb_fault_heartbeat: acquisition fault injection/watchdog stop/three live CRC fault frames");
        $finish;
    end
endmodule
