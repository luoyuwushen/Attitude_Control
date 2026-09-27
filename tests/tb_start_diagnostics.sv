`timescale 1ns/1ps
// Start outcomes must survive the telemetry interval. Stimuli use real pins,
// UART and ADC samples; no internal state is forced or protection bypassed.
module tb_start_diagnostics;
    reg clk = 0;
    always #10 clk = ~clk;
    reg rst_n = 0;
    reg [3:0] key_sw = 4'b1111;
    reg uart_rx = 1;
    reg [9:0] adc = 624;
    reg otr = 0;
    wire uart_tx, adc_clk, adc_oe_n;
    wire an1, an2, pwma, bn1, bn2, pwmb;
    top #(.KEY_CYCLES(4), .ADC_SAMPLE_CYCLES(10), .JOG_CYCLES(100000)) dut (
        .clk_50m(clk), .rst_n(rst_n), .key_sw(key_sw),
        .uart_rx(uart_rx), .uart_tx(uart_tx), .adc_data_in(adc), .adc_otr(otr),
        .adc_clk(adc_clk), .adc_oe_n(adc_oe_n), .enc1_a(1'b0), .enc1_b(1'b0),
        .AN1(an1), .AN2(an2), .PWMA(pwma), .BN1(bn1), .BN2(bn2), .PWMB(pwmb));
    // Only TX baud is accelerated. Incoming control commands retain 115200.
    defparam dut.u_telemetry.BAUD = 1000000;
    wire [7:0] rx_data;
    wire rx_valid, rx_error;
    uart_rx_byte #(.CLK_HZ(50000000), .BAUD(1000000)) monitor (
        .clk(clk), .rst_n(rst_n), .rx(uart_tx), .data(rx_data),
        .valid(rx_valid), .framing_error(rx_error));

    reg [7:0] packet [0:23];
    integer packet_index = 0, packet_count = 0;
    reg [7:0] last_diagnostic, last_fault, last_state, last_cal;
    reg [15:0] crc_work;
    integer k;
    integer collision_count = 0;
    integer previous_collisions;
    integer heartbeat_index, jog_timeout;

    function [15:0] crc_byte(input [15:0] crc, input [7:0] value);
        reg [15:0] result;
        integer bit_index;
        begin
            result = crc ^ {value,8'd0};
            for (bit_index=0; bit_index<8; bit_index=bit_index+1)
                result = result[15] ? (result<<1)^16'h1021 : result<<1;
            crc_byte = result;
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            packet_index = 0; packet_count = 0;
            last_diagnostic = 0; last_fault = 0; last_state = 0; last_cal = 0;
        end else begin
            if (rx_error) $fatal(1,"diagnostic telemetry UART framing error");
            if (rx_valid) begin
                packet[packet_index] = rx_data;
                if (packet_index == 23) begin
                    if ({packet[0],packet[1],packet[2],packet[3]} !== 32'hAA550118)
                        $fatal(1,"diagnostics changed v1 header/length");
                    crc_work = 16'hffff;
                    for(k=0;k<22;k=k+1) crc_work = crc_byte(crc_work,packet[k]);
                    if ({packet[23],packet[22]} !== crc_work)
                        $fatal(1,"diagnostic byte was not included in snapshot CRC");
                    if (!packet[21][7]) $fatal(1,"diagnostic support marker missing");
                    last_diagnostic = packet[21]; last_fault = packet[20];
                    last_state = packet[18]; last_cal = packet[19];
                    packet_count = packet_count+1; packet_index = 0;
                end else packet_index = packet_index+1;
            end
        end
    end

    always @(posedge clk) begin
        if (rst_n && dut.request_start && (dut.request_forward || dut.request_backward)) begin
            collision_count = collision_count+1;
            #3;
            if (dut.jogging || dut.state !== 1 || dut.start_result !== 1)
                $fatal(1,"same-cycle SW2/F/B admitted jog or lost start priority");
        end
        if (rst_n && {bn1,bn2,pwmb} !== 3'b001)
            $fatal(1,"diagnostics changed motor B high impedance");
    end

    task ticks(input integer cycles);
        begin repeat(cycles) begin @(posedge clk); #2; end end
    endtask
    task key(input integer index);
        begin
            @(negedge clk); key_sw[index]=0; ticks(10);
            @(negedge clk); key_sw[index]=1; ticks(10);
        end
    endtask
    task reset_board;
        begin
            @(negedge clk); rst_n=0; key_sw=4'b1111; uart_rx=1; otr=0; adc=624;
            ticks(5); @(negedge clk); rst_n=1; ticks(1000);
            if(dut.start_result !== 0) $fatal(1,"reset did not clear start result");
        end
    endtask
    task uart_bit(input bit value);
        begin uart_rx=value; repeat(434) @(negedge clk); end
    endtask
    task uart_command(input [7:0] value);
        integer bit_index;
        begin
            @(negedge clk); uart_bit(0);
            for(bit_index=0;bit_index<8;bit_index=bit_index+1) uart_bit(value[bit_index]);
            uart_bit(1); ticks(20);
        end
    endtask
    task calibrate(input integer down_code, input integer up_code);
        begin
            adc=down_code; ticks(350); key(0);
            adc=up_code; ticks(350); key(3); ticks(100);
            if (!dut.calibrated || dut.sensor_fault || dut.start_result !== 0)
                $fatal(1,"test calibration failed or did not reset diagnostics");
        end
    endtask
    task healthy_calibration;
        begin calibrate(624,800); adc=624; ticks(350); end
    endtask
    task assert_idle;
        begin
            if(dut.state !== 0 || dut.requested_command !== 0 || {an1,an2,pwma} !== 3'b001)
                $fatal(1,"rejected/finished start did not stay IDLE/zero/high impedance");
        end
    endtask
    task await_packet(input [7:0] diagnostic, input [7:0] fault,
                      input [7:0] state, input [7:0] cal);
        integer previous_packet, timeout;
        begin
            previous_packet=packet_count; timeout=0;
            while ((packet_count==previous_packet || last_diagnostic!==diagnostic ||
                    last_fault!==fault || last_state!==state || last_cal!==cal) && timeout<100000) begin
                ticks(1); timeout=timeout+1;
            end
            if(timeout==100000)
                $fatal(1,"diagnostic packet timeout want=%h/%h/%0d/%0d got=%h/%h/%0d/%0d",
                    diagnostic,fault,state,cal,last_diagnostic,last_fault,last_state,last_cal);
        end
    endtask
    task command_with_sw2(input [7:0] value);
        integer timeout;
        begin
            previous_collisions=collision_count;
            fork
                uart_command(value);
                begin
                    timeout=0;
                    // Four stable low cycles plus two synchronizer cycles put
                    // press[1] on the same cycle as UART valid's stop sample.
                    while (!(dut.u_rx.state==3 && dut.u_rx.bit_timer==5) && timeout<5000) begin
                        ticks(1); timeout=timeout+1;
                    end
                    if(timeout==5000) $fatal(1,"could not align SW2 with UART stop bit");
                    @(negedge clk); key_sw[1]=0; ticks(10);
                    @(negedge clk); key_sw[1]=1; ticks(10);
                end
            join
        end
    endtask

    initial begin #20000000; $fatal(1,"tb_start_diagnostics global timeout"); end
    initial begin
        reset_board;
        // A low pulse shorter than debounce must not produce an event.
        @(negedge clk); key_sw[1]=0; ticks(2);
        @(negedge clk); key_sw[1]=1; ticks(10);
        if(dut.start_result!==0) $fatal(1,"short SW2 pulse recorded as a start");
        await_packet(8'h80,0,0,0);
        key(1); assert_idle;
        if(dut.start_result!==2) $fatal(1,"uncalibrated SW2 rejection reason");
        await_packet(8'ha0,0,0,0);
        uart_command("S"); uart_command("R");
        if(dut.start_result!==2) $fatal(1,"S/R erased uncalibrated rejection evidence");

        healthy_calibration; await_packet(8'h80,0,0,1);
        key(1); ticks(3000);
        if(dut.start_result!==1 || dut.state!==1 || dut.requested_command!==667 ||
           !dut.motor_permission || !(an1 || an2))
            $fatal(1,"healthy SW2 not admitted or original drive chain changed");
        await_packet(8'h90,0,1,1);
        uart_command("G");
        if(dut.start_result!==7 || dut.state!==1) $fatal(1,"busy G was accepted");
        await_packet(8'hf0,0,1,1);
        uart_command("S"); assert_idle;
        if(dut.start_result!==7) $fatal(1,"stop erased busy-start result");

        key_sw[2]=0; ticks(20); key(1); assert_idle;
        if(dut.start_result!==5) $fatal(1,"held SW3/SW2 did not record stop priority");
        await_packet(8'hd8,0,0,1);
        key_sw[2]=1; ticks(20); assert_idle;
        await_packet(8'hd0,0,0,1);
        uart_command("R");
        if(dut.start_result!==5) $fatal(1,"R erased stop-priority history");

        reset_board; calibrate(0,512); adc=0; ticks(350); key(1); assert_idle;
        if(!dut.calibrated || !dut.sensor_fault || dut.fault!==0 || dut.start_result!==3)
            $fatal(1,"calibrated rail fault did not prove sticky sensor rejection");
        await_packet(8'hb1,1,0,1);
        uart_command("R");
        if(!dut.sensor_fault || dut.start_result!==3) $fatal(1,"R cleared sensor/history");
        await_packet(8'hb1,1,0,1);
        // Require multiple newly received fault snapshots, not a stale packet.
        // Every frame is independently checked for marker, length and CRC above.
        for(heartbeat_index=0;heartbeat_index<3;heartbeat_index=heartbeat_index+1)
            await_packet(8'hb1,1,0,1);

        reset_board; healthy_calibration; otr=1; ticks(30); key(1); assert_idle;
        if(dut.start_result!==4 || !dut.over_range || dut.fault!==0)
            $fatal(1,"IDLE OTR start rejection reason");
        await_packet(8'hc4,1,0,1);
        otr=0; ticks(30); assert_idle;
        await_packet(8'hc0,0,0,1);
        // An OTR during jog also latches input_fault after the live OTR ends.
        uart_command("F");
        if(!dut.jogging) $fatal(1,"healthy F did not enter jog");
        otr=1; ticks(30); otr=0; ticks(30); uart_command("G"); assert_idle;
        if(!dut.input_fault || dut.sensor_fault || dut.start_result!==4)
            $fatal(1,"latched input fault was not distinguished");
        await_packet(8'hc2,1,0,1);

        reset_board; healthy_calibration; uart_command("F"); uart_command("G");
        if(!dut.jogging || dut.state!==0 || dut.start_result!==6)
            $fatal(1,"G during jog was accepted or event lost");
        await_packet(8'he0,0,4,1);
        jog_timeout=0;
        while(dut.jogging && jog_timeout<100010) begin
            ticks(1); jog_timeout=jog_timeout+1;
        end
        if(jog_timeout==100010) $fatal(1,"jog timeout did not expire");
        ticks(3); assert_idle; await_packet(8'he0,0,0,1);

        reset_board; healthy_calibration;
        // Simultaneous D and SW2: stop wins, rather than D clearing the outcome.
        @(negedge clk); key_sw[0]=0; key_sw[1]=0; ticks(10);
        @(negedge clk); key_sw[0]=1; key_sw[1]=1; ticks(10); assert_idle;
        if(dut.start_result!==5) $fatal(1,"D/SW2 same-cycle precedence");
        await_packet(8'hd0,0,0,0);

        reset_board; healthy_calibration; command_with_sw2("R"); assert_idle;
        if(dut.start_result!==7) $fatal(1,"R/SW2 same-cycle precedence");
        await_packet(8'hf0,0,0,1);

        reset_board; healthy_calibration; command_with_sw2("F"); ticks(3000);
        if(collision_count!==previous_collisions+1 || dut.jogging || dut.start_result!==1)
            $fatal(1,"SW2/F collision was not exercised or start lost");
        await_packet(8'h90,0,1,1);
        reset_board; healthy_calibration; command_with_sw2("B"); ticks(3000);
        if(collision_count!==previous_collisions+1 || dut.jogging || dut.start_result!==1)
            $fatal(1,"SW2/B collision was not exercised or start lost");
        await_packet(8'h90,0,1,1);
        $display("PASS tb_start_diagnostics: event persistence/rejection/live flags/CRC/same-cycle priority");
        $finish;
    end
endmodule
