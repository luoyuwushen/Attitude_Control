`timescale 1ns/1ps
// Real UART D/U/H admission and the complete CRC-protected v2 wire format.
// Only acquisition-completion pulses are suppressed for the outage test;
// calibration, controller state and safety logic are never forced.
module tb_top_v2;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0, uart_rx=1, adc_otr=0;
    reg [3:0] key_sw=4'b1111;
    reg [9:0] adc=240;
    wire uart_tx, adc_clk, adc_oe_n, an1, an2, pwma, bn1, bn2, pwmb;
    top #(.KEY_CYCLES(4),.ADC_SAMPLE_CYCLES(10)) dut(
        .clk_50m(clk),.rst_n(rst_n),.key_sw(key_sw),.uart_rx(uart_rx),.uart_tx(uart_tx),
        .adc_data_in(adc),.adc_otr(adc_otr),.adc_clk(adc_clk),.adc_oe_n(adc_oe_n),
        .enc1_a(1'b0),.enc1_b(1'b0),.AN1(an1),.AN2(an2),.PWMA(pwma),
        .BN1(bn1),.BN2(bn2),.PWMB(pwmb));
    defparam dut.u_telemetry.BAUD=1000000;
    wire [7:0] rx_data;
    wire rx_valid, rx_error;
    uart_rx_byte #(.BAUD(1000000)) monitor(
        .clk(clk),.rst_n(rst_n),.rx(uart_tx),.data(rx_data),.valid(rx_valid),.framing_error(rx_error));
    reg [7:0] packet[0:218];
    integer packet_index=0, packet_count=0, k, clear_observed=0;
    reg [15:0] crc_work, last_sequence=0;
    reg [7:0] last_state=0, last_fault=0, last_diagnostic=0, last_cal=0, last_first=0;
    reg [15:0] last_adc=0, last_flags=0, last_down=0, last_up=0;
    reg [15:0] last_raw=0, last_min=0, last_max=0, last_mean=0;
    reg [7:0] last_quality=0;
    reg [15:0] last_sensor_reason=0, last_fault_min=0, last_fault_max=0, last_fault_mean=0;
    reg [271:0] last_detail=0, last_fault_detail=0, expected_fault_detail=0;
    reg [31:0] last_fault_time=0, last_fault_counter=0;
    reg [31:0] expected_fault_time=0, expected_fault_counter=0;
    reg capture_expected=0, expected_valid=0, observed_sensor_fault=0;
    reg prohibit_restart=0;
    reg signed [15:0] last_theta=0, last_command=0, last_motor=0;
    reg [31:0] last_time=0, last_counter=0, saved_counter;
    reg [55:0] last_h_detail=0;
    function [15:0] crc_byte(input [15:0] crc,input [7:0] value);
        reg [15:0] work;
        integer n;
        begin
            work=crc^{value,8'd0};
            for(n=0;n<8;n=n+1) work=work[15] ? (work<<1)^16'h1021 : work<<1;
            crc_byte=work;
        end
    endfunction
    task tag(input integer offset,input integer id,input integer length);
        begin
            if(packet[offset]!==id || packet[offset+1]!==length)
                $fatal(1,"v2 TLV boundary/type/length at offset %0d",offset);
        end
    endtask
    always @(posedge clk) begin
        if(rst_n && rx_error) $fatal(1,"v2 UART framing error");
        if(rst_n && rx_valid) begin
            packet[packet_index]=rx_data;
            if(packet_index==218) begin
                if({packet[0],packet[1],packet[2],packet[3]}!==32'haa5502db)
                    $fatal(1,"top default must emit version 2, length 219");
                crc_work=16'hffff;
                for(k=0;k<217;k=k+1) crc_work=crc_byte(crc_work,packet[k]);
                if({packet[218],packet[217]}!==crc_work) $fatal(1,"v2 full-frame CRC");
                if(packet_count && {packet[5],packet[4]}!==last_sequence+16'd1)
                    $fatal(1,"v2 sequence lost/repeated");
                last_sequence={packet[5],packet[4]};
                tag(22,1,4); tag(28,2,4); tag(34,9,2); tag(38,10,2);
                tag(42,11,2); tag(46,12,2); tag(50,13,2); tag(54,14,2);
                tag(58,15,2); tag(62,16,2); tag(66,17,2); tag(70,18,1);
                tag(73,19,4); tag(79,20,4);
                tag(85,21,1); tag(88,22,2); tag(92,23,2); tag(96,24,2); tag(100,25,2);
                tag(104,26,34); tag(140,27,34); tag(176,28,4); tag(182,29,4); tag(188,30,1); tag(191,31,4);
                tag(197,32,7); tag(206,33,5); tag(213,34,2);
                for(k=0;k<7;k=k+1) last_h_detail[k*8 +: 8]=packet[199+k];
                if(packet[18]!=2 && last_h_detail!==0)
                    $fatal(1,"H detail persisted after handover stopped");
                if(packet[18]==2 && (!last_h_detail[48] || last_h_detail[15:0]!==0 ||
                                   last_h_detail[31:16]!==0 || last_h_detail[47:32]>512))
                    $fatal(1,"H detail missing or invalid for zero-position upright test");
                if(packet[190]!==0 || {packet[196],packet[195],packet[194],packet[193]}!==0)
                    $fatal(1,"unused motor test fields must remain zero");
                if({packet[78],packet[77],packet[76],packet[75]}!==32'h00030000 ||
                   {packet[37],packet[36]}!==16'd1000 ||
                   {packet[33],packet[32],packet[31],packet[30]}!==32'd0)
                    $fatal(1,"v2 firmware version/control rate/raw encoder");
                last_adc={packet[7],packet[6]}; last_theta={packet[9],packet[8]};
                last_command={packet[17],packet[16]};
                last_state=packet[18]; last_cal=packet[19]; last_fault=packet[20]; last_diagnostic=packet[21];
                last_time={packet[27],packet[26],packet[25],packet[24]};
                last_flags={packet[41],packet[40]}; last_down={packet[45],packet[44]}; last_up={packet[49],packet[48]};
                last_raw={packet[53],packet[52]}; last_min={packet[57],packet[56]}; last_max={packet[61],packet[60]};
                last_mean={packet[65],packet[64]}; last_motor={packet[69],packet[68]}; last_first=packet[72];
                last_counter={packet[84],packet[83],packet[82],packet[81]};
                last_quality=packet[87]; last_sensor_reason={packet[91],packet[90]};
                last_fault_min={packet[95],packet[94]}; last_fault_max={packet[99],packet[98]};
                last_fault_mean={packet[103],packet[102]};
                for(k=0;k<34;k=k+1) begin
                    last_detail[k*8 +: 8]=packet[106+k];
                    last_fault_detail[k*8 +: 8]=packet[142+k];
                end
                last_fault_time={packet[181],packet[180],packet[179],packet[178]};
                last_fault_counter={packet[187],packet[186],packet[185],packet[184]};
                packet_count=packet_count+1; packet_index=0;
            end else packet_index=packet_index+1;
        end
        if(rst_n && dut.request_clear && dut.stopped) begin
            #1;
            if(dut.first_fault!==0) $fatal(1,"stopped UART R did not clear first-fault latch");
            clear_observed=clear_observed+1;
        end
    end
    // Keep an independent source-side scoreboard at the first observed fault.
    // Compare subsequent wire frames against this original window, even after
    // the live diagnostics and later fault sources have changed.
    always @(posedge clk) begin
        if(!rst_n) observed_sensor_fault=0;
        else begin
            if(capture_expected && dut.sensor_fault && !observed_sensor_fault) begin
                if(expected_valid) $fatal(1,"unexpected replacement sensor-fault capture");
                expected_fault_detail=dut.adc_window_detail;
                expected_fault_time=dut.device_time_ms;
                expected_fault_counter=dut.sample_counter;
                expected_valid=1;
            end
            observed_sensor_fault=dut.sensor_fault;
        end
    end
    // Every cycle is checked: a transient SWING/kick is forbidden on H.
    always @(posedge clk) begin
        #2;
        if(rst_n && dut.state==1) $fatal(1,"UART H briefly entered SWING");
        if(rst_n && {an1,an2,pwma}!==3'b001) $fatal(1,"unused XH2 drive enabled");
        if(prohibit_restart && (dut.state!=0 || dut.motor_enable || dut.command || dut.drive_enabled))
            $fatal(1,"measurement recovery or R restarted the motor without a fresh motion request");
    end
    task ticks(input integer count);
        begin repeat(count) begin @(posedge clk); #3; end end
    endtask
    task uart_bit(input bit value);
        begin uart_rx=value; repeat(434) @(negedge clk); end
    endtask
    task send(input [7:0] value);
        integer n;
        begin
            @(negedge clk); uart_bit(0);
            for(n=0;n<8;n=n+1) uart_bit(value[n]);
            uart_bit(1); ticks(20);
        end
    endtask
    task move_adc(input integer target);
        integer delta;
        begin
            while(adc!=target) begin
                @(negedge clk); delta=target-$signed({1'b0,adc});
                if(delta>16) adc=adc+16;
                else if(delta < -16) adc=adc-16;
                else adc=target;
                ticks(160);
            end
            ticks(5120);
        end
    endtask
    task await_frame(input integer state_value,input integer fault_value,
                     input integer diagnostic_value,input integer adc_value);
        integer before_count, timeout;
        begin
            before_count=packet_count; timeout=0;
            while((packet_count==before_count || last_state!==state_value || last_fault!==fault_value ||
                   last_diagnostic!==diagnostic_value || last_adc!==adc_value) && timeout<300000) begin
                ticks(10); timeout=timeout+10;
            end
            if(timeout>=300000)
                $fatal(1,"v2 frame timeout want=%0d/%h/%h/%0d got=%0d/%h/%h/%0d",
                    state_value,fault_value,diagnostic_value,adc_value,last_state,last_fault,last_diagnostic,last_adc);
        end
    endtask
    task trusted_frame(input integer code);
        begin
            if(last_cal!=1 || last_flags!=16'h000c || last_down!=240 || last_up!=800 ||
               last_raw!=code || last_min!=code || last_max!=code || last_mean!=code*16 || !last_counter ||
               last_quality || last_sensor_reason || last_fault_min || last_fault_max || last_fault_mean ||
               last_fault_detail || last_fault_time || last_fault_counter)
                $fatal(1,"v2 trusted state/calibration/raw-window/mean/counter/fault-clear mismatch");
            if(last_detail[15:0]!=code*16 || last_detail[31:16]!=code ||
               last_detail[47:32]!=code || last_detail[63:48]!=code || last_detail[79:64]!=code ||
               last_detail[95:80] || last_detail[111:96] || last_detail[127:112] ||
               last_detail[223:208]!=16 || last_detail[271:240]!=code*16)
                $fatal(1,"v2 healthy live diagnostic baseline/range/step/count/sum mismatch");
        end
    endtask
    task retained_evidence;
        begin
            // Confirmed non-rail OTR is the first hard fault. Later healthy
            // input, outages and fresh OTR must not replace its exact evidence.
            if(last_sensor_reason!=16'h0001 || last_fault_min!=788 || last_fault_max!=788 || last_fault_mean!=788*16)
                $fatal(1,"first sensor evidence lost: reason=%h min=%0d max=%0d mean=%0d",
                    last_sensor_reason,last_fault_min,last_fault_max,last_fault_mean);
            if(!expected_valid || last_fault_detail!==expected_fault_detail ||
               last_fault_time!==expected_fault_time || last_fault_counter!==expected_fault_counter ||
               !last_fault_time || !last_fault_counter || last_fault_counter>=last_counter)
                $fatal(1,"first sensor bundle/time/sample counter was lost or overwritten");
            if(last_fault_detail[15:0]!=788*16 || last_fault_detail[31:16]!=788 ||
               last_fault_detail[47:32]!=788 || last_fault_detail[95:80]!=0 ||
               last_fault_detail[111:96]!=0 || last_fault_detail[223:208]!=16 ||
               last_fault_detail[271:240]!=16*788)
                $fatal(1,"triggering window diagnostic baseline/range/step/count/sum mismatch: %h",last_fault_detail);
            if(last_detail===last_fault_detail)
                $fatal(1,"live diagnostic windows never diverged from latched fault evidence");
        end
    endtask
    initial begin #100000000; $fatal(1,"tb_top_v2 timeout"); end
    initial begin
        ticks(5); @(negedge clk); rst_n=1; ticks(1000);
        send("D");
        if(dut.adc_down_q4!=240*16 || dut.calibrated) $fatal(1,"real UART D failed");
        // An uncalibrated hand movement can produce a rejected window. It
        // must recover without creating a sticky fault or altering an endpoint.
        @(negedge clk); adc=800; ticks(1000);
        if(dut.sensor_fault || dut.sample_bad) $fatal(1,"uncalibrated hand movement must recover without sticky fault");
        send("U"); ticks(3000);
        if(!dut.calibrated || !dut.measurement_ready || dut.sensor_fault || dut.adc_up_q4!=800*16)
            $fatal(1,"fresh UART U did not calibrate/clear fault/mature measurement");
        await_frame(0,0,8'h80,800); trusted_frame(800);
        if(last_first!=0 || last_theta!=0) $fatal(1,"calibration did not reset origin/first-fault");
        send("H");
        if(dut.state!=2 || dut.start_result!=1 || dut.command!=0 || !dut.motor_enable)
            $fatal(1,"real UART H did not hand directly to BALANCE");
        await_frame(2,0,8'h90,800); trusted_frame(800);
        if(last_command || last_motor) $fatal(1,"zero-error H generated a kick");
        send("S"); move_adc(740);
        if(dut.sensor_fault || !dut.measurement_ready) $fatal(1,"angle rejection setup became invalid");
        send("H");
        if(dut.state!=0 || dut.start_result!=7 || dut.motor_enable || dut.command)
            $fatal(1,"out-of-angle H fell back to swing or enabled drive");
        await_frame(0,0,8'hf0,740); trusted_frame(740);
        move_adc(800); send("H"); move_adc(788);
        await_frame(2,0,8'h90,788); trusted_frame(788);
        if(last_theta<=0 || last_command>=0 || last_motor!=-last_command)
            $fatal(1,"v2 nonzero upright request/applied-command sign path");
        capture_expected=1;
        @(negedge clk); adc_otr=1; ticks(4000);
        if(dut.state!=3 || dut.first_fault!=1 || dut.telemetry_fault!=1)
            $fatal(1,"persistent non-rail OTR was not the first fault");
        @(negedge clk); adc_otr=0; move_adc(730);
        await_frame(3,1,8'h93,730);
        if(last_first!=1 || last_command || last_motor) $fatal(1,"v2 first fault or stopped output missing");
        retained_evidence;
        if(last_quality || last_min!=730 || last_max!=730 || last_mean!=730*16)
            $fatal(1,"healthy live window was not distinct from the retained triggering window");
        @(negedge clk); force dut.adc_valid=1'b0;
        ticks(100030);
        await_frame(3,5,8'h93,730);
        if(last_first!=1 || last_flags[3:2]!=0) $fatal(1,"timeout overwrote first fault or left freshness flags set");
        retained_evidence;
        saved_counter=last_counter;
        await_frame(3,5,8'h93,730);
        if(last_counter!=saved_counter || last_first!=1 || last_time==0)
            $fatal(1,"outage heartbeat counter/time/first-fault semantics");
        send("S"); await_frame(0,5,8'h93,730);
        if(last_first!=1) $fatal(1,"S erased first fault");
        send("R"); await_frame(0,5,8'h93,730);
        // A frame already in flight at R legitimately contains the old latch.
        if(last_first!=5) await_frame(0,5,8'h93,730);
        // The monitor observes R clearing the latch. The still-present 0x05
        // fault is allowed, and required, to be recorded again afterwards.
        if(clear_observed!=1 || last_first!=5)
            $fatal(1,"R clear/ongoing-fault relatch semantics: clears=%0d first=%h",clear_observed,last_first);
        retained_evidence;
        prohibit_restart=1;
        release dut.adc_valid;
        ticks(4000);
        if(!dut.sensor_fault || !dut.fault_clear_ready || !dut.calibrated)
            $fatal(1,"fresh measurements auto-cleared the latched sensor fault or failed recovery readiness");
        // Explicit R still cannot acknowledge an active raw OTR condition.
        @(negedge clk); adc_otr=1; ticks(4000); send("R");
        if(!dut.sensor_fault || dut.fault_clear_ready) $fatal(1,"R cleared an ongoing raw OTR fault");
        await_frame(0,1,8'h97,730); retained_evidence;
        if(last_quality!=1 || last_first!=1) $fatal(1,"ongoing OTR quality/aggregate cause missing");
        @(negedge clk); adc_otr=0; ticks(4000);
        if(!dut.sensor_fault || !dut.fault_clear_ready) $fatal(1,"sensor recovery must await explicit R");
        send("R"); ticks(100);
        if(dut.sensor_fault || dut.sensor_fault_reason || dut.telemetry_fault || !dut.calibrated ||
           dut.adc_down_q4!=240*16 || dut.adc_up_q4!=800*16 || clear_observed!=3)
            $fatal(1,"healthy R failed to clear fault while preserving calibration");
        await_frame(0,0,8'h90,730); trusted_frame(730);
        if(last_first) $fatal(1,"healthy R left a first-fault latch");
        move_adc(800);
        // Recovery and repositioning remain stopped; only this new H starts.
        prohibit_restart=0; send("H");
        if(dut.state!=2 || !dut.motor_enable) $fatal(1,"new H after explicit recovery failed");
        await_frame(2,0,8'h90,800); trusted_frame(800);
        $display("PASS tb_top_v2: real UART D/U/H/no SWING/219-byte CRC+TLVs/full diagnostic first-sensor bundle/time/counter evidence/outage/OTR/R recovery retains calibration and requires new H");
        $finish;
    end
endmodule
