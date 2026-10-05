`timescale 1ns/1ps
// Actual command UART, quadrature input, PWM pads and extended telemetry.
// Shorter test deadlines bound simulation; production limits are unit-checked.
module tb_motor_test_top;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0, rx=1, otr=0, enc_a=0, enc_b=0;
    reg [3:0] keys=4'b1111;
    reg [9:0] adc=240;
    wire tx, adc_clk, oe, an1, an2, pwma, bn1, bn2, pwmb;
    top #(.KEY_CYCLES(4),.ADC_SAMPLE_CYCLES(10),
          .MOTOR_TEST_CYCLES(60000),.MOTOR_TEST_NO_MOTION_CYCLES(50000),
          .MOTOR_TEST_MAX_TRAVEL(8),.SAMPLE_WATCHDOG_CYCLES(10000)) dut(
        .clk_50m(clk),.rst_n(rst_n),.key_sw(keys),.uart_rx(rx),.uart_tx(tx),
        .adc_data_in(adc),.adc_otr(otr),.adc_clk(adc_clk),.adc_oe_n(oe),
        .enc1_a(enc_a),.enc1_b(enc_b),.AN1(an1),.AN2(an2),.PWMA(pwma),
        .BN1(bn1),.BN2(bn2),.PWMB(pwmb));
    defparam dut.u_telemetry.BAUD=5000000;
    wire [7:0] byte_data;
    wire byte_valid, byte_error;
    uart_rx_byte #(.BAUD(5000000)) monitor(clk,rst_n,tx,byte_data,byte_valid,byte_error);
    reg [7:0] packet[0:218];
    integer index=0, packets=0, k;
    reg [7:0] wire_state=0, wire_status=0;
    reg signed [31:0] wire_delta=0;
    reg [15:0] crc;
    function [15:0] crcbyte(input [15:0] previous,input [7:0] value);
        reg [15:0] work; integer b;
        begin
            work=previous^{value,8'd0};
            for(b=0;b<8;b=b+1) work=work[15]?(work<<1)^16'h1021:work<<1;
            crcbyte=work;
        end
    endfunction
    always @(posedge clk) if(rst_n) begin
        if({an1,an2,pwma}!==3'b001 || (bn1 && bn2)) $fatal(1,"motor pad routing");
        if(byte_error) $fatal(1,"motor test telemetry UART framing");
        if(byte_valid) begin
            packet[index]=byte_data;
            if(index==218) begin
                crc=16'hffff;
                for(k=0;k<217;k=k+1) crc=crcbyte(crc,packet[k]);
                if({packet[0],packet[1],packet[2],packet[3]}!==32'haa5502db ||
                   {packet[218],packet[217]}!==crc) $fatal(1,"motor test wire header/CRC");
                if(packet[213]!==34 || packet[214]!==2)
                    $fatal(1,"missing control ADC TLV");
                if(packet[197]!==32 || packet[198]!==7)
                    $fatal(1,"missing H detail TLV");
                for(k=199;k<206;k=k+1)
                    if(packet[k]!==0) $fatal(1,"manual motor test inherited H control detail");
                if(packet[206]!==33 || packet[207]!==5 ||
                   {packet[212],packet[211],packet[210],packet[209],packet[208]}!==40'd0)
                    $fatal(1,"manual motor test invented an H capture");
                if(packet[188]!=30 || packet[189]!=1 || packet[191]!=31 || packet[192]!=4)
                    $fatal(1,"motor test TLV contract");
                wire_state=packet[18]; wire_status=packet[190];
                wire_delta={packet[196],packet[195],packet[194],packet[193]};
                packets=packets+1; index=0;
            end else index=index+1;
        end
    end
    task ticks(input integer n); begin repeat(n) begin @(posedge clk);#2;end end endtask
    task uart_bit(input bit value); begin rx=value;repeat(434)@(negedge clk);end endtask
    task send(input [7:0] value);
        integer bit_index;
        begin
            @(negedge clk);uart_bit(0);
            for(bit_index=0;bit_index<8;bit_index=bit_index+1) uart_bit(value[bit_index]);
            uart_bit(1);ticks(20);
        end
    endtask
    task move_adc(input integer target);
        integer remaining;
        begin
            while(adc!=target) begin
                @(negedge clk);remaining=target-$signed({1'b0,adc});
                adc=remaining>16?adc+16:remaining < -16?adc-16:target;
                ticks(160);
            end
            ticks(5120);
        end
    endtask
    task quadrature(input bit positive);
        begin
            @(negedge clk);{enc_a,enc_b}=positive?2'b10:2'b01;ticks(20);
            @(negedge clk);{enc_a,enc_b}=2'b11;ticks(20);
            @(negedge clk);{enc_a,enc_b}=positive?2'b01:2'b10;ticks(20);
            @(negedge clk);{enc_a,enc_b}=2'b00;ticks(20);
        end
    endtask
    task pwm_count(input integer expected,input bit forward_direction);
        integer high_count,n;
        begin
            ticks(5000);high_count=0;
            for(n=0;n<2500;n=n+1) begin
                ticks(1);
                if(!dut.test_active || {bn1,bn2}!==(forward_direction?2'b01:2'b10) ||
                   dut.motor_command !== -dut.test_command)
                    $fatal(1,"fixed-duty test direction/admission");
                if(pwmb) high_count=high_count+1;
            end
            if(high_count!=expected) $fatal(1,"fixed duty expected%0d got%0d",expected,high_count);
        end
    endtask
    task await_status(input integer value);
        integer timeout;
        begin
            timeout=0;
            while(dut.motor_test_status!=value && timeout<80000) begin ticks(10);timeout=timeout+10;end
            if(dut.motor_test_status!=value) $fatal(1,"test end status want%0d got%0d",value,dut.motor_test_status);
            ticks(3);
            if(dut.test_active || dut.requested_command || dut.drive_enabled || {bn1,bn2,pwmb}!==3'b001)
                $fatal(1,"finished test still drives pads");
        end
    endtask
    task await_wire(input integer state_value,input integer status_value,input integer delta_value);
        integer before_count,timeout;
        begin
            before_count=packets;timeout=0;
            while((packets==before_count || wire_state!=state_value || wire_status!=status_value ||
                   wire_delta!=delta_value) && timeout<100000) begin ticks(10);timeout=timeout+10;end
            if(timeout>=100000) $fatal(1,"motor test wire status/delta missing");
        end
    endtask
    initial begin #30000000;$fatal(1,"tb_motor_test_top timeout");end
    initial begin
        ticks(3);@(negedge clk);rst_n=1;ticks(1000);
        send("J");await_status(7);await_wire(0,7,0);
        send("D");move_adc(800);send("U");move_adc(240);
        if(!dut.measurement_ready || dut.sensor_fault) $fatal(1,"test calibration setup");
        send("J");pwm_count(375,1);
        // Synthetic installed-actuator response: negative hardware command
        // increases raw counts. This checks mapping, not a physical plant.
        quadrature(1);
        await_wire(5,1,4);
        send("G");if(dut.state || dut.start_result!=6) $fatal(1,"G entered controller during test");
        send("B");if(dut.jogging) $fatal(1,"F/B overlapped independent test");
        send("M");if(dut.test_command!=150) $fatal(1,"busy request reversed test");
        await_status(2);await_wire(0,2,4);
        send("K");pwm_count(375,0);quadrature(0);quadrature(0);
        await_status(3);await_wire(0,3,-8);
        quadrature(0);await_wire(0,3,-12); // Coasting remains visible.
        send("L");pwm_count(550,1);await_status(6);await_wire(0,6,0);
        send("M");pwm_count(550,0);
        @(negedge clk);keys[2]=0;#1;
        if({bn1,bn2,pwmb}!==3'b001) $fatal(1,"raw SW3 test gate");
        ticks(3);@(negedge clk);keys[2]=1;ticks(20);await_status(4);await_wire(0,4,0);
        send("J");send("D");await_status(4);
        if(!dut.calibrated || dut.adc_down_q4!=240*16) $fatal(1,"D recorded while motor test active");
        send("M");ticks(3000);@(negedge clk);otr=1;ticks(30);await_status(5);
        send("L");await_status(7);
        @(negedge clk);otr=0;ticks(5120);send("R");ticks(5120);
        if(dut.sensor_fault || !dut.calibrated) $fatal(1,"R recovery for watchdog test");
        send("L");if(!dut.test_active) $fatal(1,"post-recovery new test");
        @(negedge clk);force dut.adc_valid=1'b0;ticks(10030);await_status(5);
        if(!dut.sample_timeout) $fatal(1,"sample outage did not stop test");
        release dut.adc_valid;ticks(5120);
        if(dut.test_active || dut.drive_enabled) $fatal(1,"sample recovery restarted test");
        $display("PASS tb_motor_test_top: real UART/219-byte CRC state+reason+signed delta/PWM levels/XH1/mode exclusion/deadline/travel/coasting/no-motion/raw SW3/calibration/OTR/watchdog/no restart");
        $finish;
    end
endmodule
