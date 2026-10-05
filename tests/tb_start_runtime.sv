`timescale 1ns/1ps
// Use production clock, debounce, ADC, PWM and UART parameters. The reported
// field calibration (240 down / 770 up) must drive the same XH1 connector as
// encoder 1. Only the final acquisition-outage case injects an internal fault;
// no start, calibration, state or protection signal is overridden.
module tb_start_runtime;
    reg clk = 0;
    always #10 clk = ~clk;
    reg rst_n = 0;
    reg [3:0] key_sw = 4'b1111;
    reg uart_rx = 1;
    reg [9:0] adc = 240;
    wire uart_tx, adc_clk, adc_oe_n;
    wire an1, an2, pwma, bn1, bn2, pwmb;
    top #(.TELEMETRY_EXTENDED(0)) dut (
        .clk_50m(clk), .rst_n(rst_n), .key_sw(key_sw),
        .uart_rx(uart_rx), .uart_tx(uart_tx), .adc_data_in(adc), .adc_otr(1'b0),
        .adc_clk(adc_clk), .adc_oe_n(adc_oe_n), .enc1_a(1'b0), .enc1_b(1'b0),
        .AN1(an1), .AN2(an2), .PWMA(pwma), .BN1(bn1), .BN2(bn2), .PWMB(pwmb));
    wire [7:0] rx_data;
    wire rx_valid, rx_error;
    uart_rx_byte monitor (
        .clk(clk), .rst_n(rst_n), .rx(uart_tx), .data(rx_data),
        .valid(rx_valid), .framing_error(rx_error));
    reg [7:0] packet [0:23];
    reg [7:0] last_state = 0, last_fault = 0, last_diagnostic = 0;
    reg [15:0] crc_work, previous_sequence = 0;
    integer packet_index = 0, packet_count = 0, k;
    integer clocks = 0, last_adc_clock = 0, last_packet_clock = 0;
    integer saved_packet_count, fault_packet_count, heartbeat_deadline;
    integer calibration_bad_windows=0;
    reg observe_calibration_move=0;
    reg [9:0] saved_adc;
    reg signed [15:0] saved_theta;

    function [15:0] crc_byte(input [15:0] crc, input [7:0] value);
        reg [15:0] work;
        integer n;
        begin
            work=crc^{value,8'd0};
            for(n=0;n<8;n=n+1) work=work[15] ? (work<<1)^16'h1021 : work<<1;
            crc_byte=work;
        end
    endfunction
    always @(posedge clk) begin
        clocks = clocks+1;
        if(rst_n) begin
            if({an1,an2,pwma} !== 3'b001)
                $fatal(1,"unused XH2 / TB6612 A must remain high impedance");
            if(bn1 && bn2) $fatal(1,"XH1 / TB6612 B invalid direction");
            if(dut.adc_valid) begin
                if(last_adc_clock && clocks-last_adc_clock!=50000)
                    $fatal(1,"production ADC cadence is not 1 kHz");
                last_adc_clock=clocks;
                if(observe_calibration_move && dut.sample_bad)
                    calibration_bad_windows=calibration_bad_windows+1;
            end
            if(observe_calibration_move && (dut.sensor_fault || dut.telemetry_fault))
                $fatal(1,"one isolated calibration movement window became a sticky fault");
            if(rx_error) $fatal(1,"production telemetry UART framing error");
            if(rx_valid) begin
                packet[packet_index]=rx_data;
                if(packet_index==23) begin
                    if({packet[0],packet[1],packet[2],packet[3]}!==32'haa550118)
                        $fatal(1,"production v1 frame changed");
                    crc_work=16'hffff;
                    for(k=0;k<22;k=k+1) crc_work=crc_byte(crc_work,packet[k]);
                    if({packet[23],packet[22]}!==crc_work)
                        $fatal(1,"production telemetry CRC mismatch");
                    if(packet_count && {packet[5],packet[4]}!=previous_sequence+16'd1)
                        $fatal(1,"production telemetry sequence skipped");
                    if(last_packet_clock && clocks-last_packet_clock>1000100)
                        $fatal(1,"fault heartbeat exceeded 20 ms plus 2 us tolerance");
                    previous_sequence={packet[5],packet[4]};
                    last_packet_clock=clocks;
                    last_state=packet[18]; last_fault=packet[20]; last_diagnostic=packet[21];
                    packet_count=packet_count+1; packet_index=0;
                end else packet_index=packet_index+1;
            end
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
                ticks(50000);
            end
            ticks(1600000);
        end
    endtask
    task uart_bit(input bit value);
        begin uart_rx=value; repeat(434) @(negedge clk); end
    endtask
    task send(input [7:0] value);
        integer bit_index;
        begin
            @(negedge clk); uart_bit(0);
            for(bit_index=0;bit_index<8;bit_index=bit_index+1) uart_bit(value[bit_index]);
            uart_bit(1); ticks(20);
        end
    endtask
    task assert_drive;
        integer high_count, count;
        begin
            ticks(55000);
            if(dut.state!==1 || !dut.motor_enable || dut.start_result!==1 ||
               dut.command!==667 || dut.motor_command!==-667 || {bn1,bn2}!==2'b01)
                $fatal(1,"accepted start did not drive XH1: state=%0d fault=%h cmd=%0d B=%b%b",
                       dut.state,dut.fault,dut.command,bn1,bn2);
            high_count=0;
            for(count=0;count<2500;count=count+1) begin
                ticks(1); if(pwmb) high_count=high_count+1;
            end
            if(high_count!=1667) $fatal(1,"XH1 667 permille PWM expected1667 got%0d",high_count);
        end
    endtask
    initial begin #300000000; $fatal(1,"tb_start_runtime timeout"); end
    initial begin
        ticks(5); @(negedge clk); rst_n=1; ticks(105000);
        send("D");
        if(dut.u_estimator.down_code!==240*16) $fatal(1,"field down calibration");
        // A rapid manual move crosses one untrusted window. It is explicitly
        // rejected, then clean sampling recovers without inventing a fault.
        @(negedge clk); observe_calibration_move=1; adc=770; ticks(105000);
        observe_calibration_move=0;
        if(calibration_bad_windows!=1 || dut.sensor_fault || dut.sample_bad || dut.control_q4!=770*16)
            $fatal(1,"calibration move did not isolate one bad window and recover: bad=%0d",calibration_bad_windows);
        send("U"); ticks(55000);
        if(!dut.calibrated || dut.sensor_fault || dut.theta!==0)
            $fatal(1,"field up calibration");
        move_adc(240);
        if(!dut.measurement_ready) $fatal(1,"production measurement did not mature");
        if(dut.theta<3150 || dut.theta>3217) $fatal(1,"field down angle");
        send("G"); assert_drive;
        send("S"); ticks(3);
        if(dut.state!==0 || {bn1,bn2,pwmb}!==3'b001) $fatal(1,"UART S did not stop XH1");

        // A 10 ms pulse is rejected; a fresh hold crossing 20 ms starts once.
        @(negedge clk); key_sw[1]=0; ticks(500000);
        @(negedge clk); key_sw[1]=1; ticks(10);
        if(dut.state!==0) $fatal(1,"short SW2 pulse bypassed production debounce");
        @(negedge clk); key_sw[1]=0; ticks(1000010);
        assert_drive;
        @(negedge clk); key_sw[1]=1; key_sw[2]=0; #1;
        if({bn1,bn2,pwmb}!==3'b001) $fatal(1,"raw SW3 did not stop XH1 immediately");
        ticks(5); @(negedge clk); key_sw[2]=1; ticks(100);
        if(dut.state!==0) $fatal(1,"SW3 release restarted drive");

        send("G"); assert_drive;
        // Fault injection stops acquisition completions, as a failed sampler
        // would. Clock, UART, controller watchdog and protection keep running.
        @(negedge clk); force dut.adc_valid=1'b0;
        ticks(20); saved_adc=dut.telemetry_adc; saved_theta=dut.theta;
        ticks(100010);
        if(dut.state!==3 || dut.fault!==8'h04 || {bn1,bn2,pwmb}!==3'b001)
            $fatal(1,"sample outage did not latch watchdog and coast XH1");
        ticks(110000); // Drain a frame that began before watchdog assertion.
        saved_packet_count=packet_count; fault_packet_count=0; heartbeat_deadline=clocks+4000000;
        while(fault_packet_count<3 && clocks<heartbeat_deadline) begin
            ticks(100);
            if(packet_count!=saved_packet_count) begin
                saved_packet_count=packet_count;
                if(last_state!==3 || last_fault!==8'h04 || last_diagnostic[6:4]!==1)
                    $fatal(1,"watchdog fault not visible in live v1 telemetry");
                if({packet[7],packet[6]}!=={6'd0,saved_adc} ||
                   {packet[9],packet[8]}!==saved_theta)
                    $fatal(1,"fault heartbeat did not retain completed ADC/state pair");
                fault_packet_count=fault_packet_count+1;
            end
        end
        if(fault_packet_count!=3) $fatal(1,"sampling outage suppressed fault telemetry");
        send("S"); ticks(100);
        if(dut.state!==0 || {bn1,bn2,pwmb}!==3'b001)
            $fatal(1,"UART stop unavailable during sample outage");
        release dut.adc_valid;
        $display("PASS tb_start_runtime: production SW2/G/ADC/PWM/UART/field calibration/XH1 routing/watchdog heartbeat");
        $finish;
    end
endmodule
