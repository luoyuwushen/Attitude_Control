`timescale 1ns/1ps
// External ADC, key and command UART stimuli; no forced estimator/controller state.
module tb_blindzone_top;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0, rx=1, otr=0;
    reg [9:0] adc=220;
    reg [3:0] keys=15;
    wire tx, adc_clk, oe, an1, an2, pwma, bn1, bn2, pwmb;
    top #(.KEY_CYCLES(4),.ADC_SAMPLE_CYCLES(80),.UART_BAUD(5000000),
          .TRACE_ADDRESS_BITS(4)) dut(
        .clk_50m(clk),.rst_n(rst_n),.key_sw(keys),.uart_rx(rx),.uart_tx(tx),
        .adc_data_in(adc),.adc_otr(otr),.adc_clk(adc_clk),.adc_oe_n(oe),
        .enc1_a(1'b0),.enc1_b(1'b0),.AN1(an1),.AN2(an2),.PWMA(pwma),
        .BN1(bn1),.BN2(bn2),.PWMB(pwmb));
    task ticks(input integer n); begin repeat(n) begin @(posedge clk); #2; end end endtask
    task key(input integer index);
        begin @(negedge clk); keys[index]=0; ticks(12);
              @(negedge clk); keys[index]=1; ticks(12); end
    endtask
    task command(input [7:0] value);
        integer k;
        begin
            @(negedge clk); rx=0; repeat(10) @(negedge clk);
            for(k=0;k<8;k=k+1) begin rx=value[k]; repeat(10) @(negedge clk); end
            rx=1; repeat(10) @(negedge clk); ticks(20);
        end
    endtask
    task settle(input integer code);
        begin @(negedge clk); adc=code; ticks(1280*24); end
    endtask
    task expect_idle;
        begin
            if(dut.state!=0 || dut.telemetry_fault || dut.drive_enabled || dut.command)
                $fatal(1,"blind/recovery must be idle without 0x01 or drive: state=%d fault=%h",dut.state,dut.telemetry_fault);
        end
    endtask
    integer i;
    reg guard_blind_scan=0;
    always @(posedge clk) begin
        #3;
        if(guard_blind_scan && (dut.telemetry_fault || dut.sensor_fault || dut.input_fault || dut.first_fault))
            $fatal(1,"blind scan transient fault: telemetry=%h first=%h",dut.telemetry_fault,dut.first_fault);
    end
    initial begin #100000000; $fatal(1,"blind-zone top timeout"); end
    initial begin
        ticks(5); @(negedge clk); rst_n=1;
        settle(220); key(0); settle(760); key(3); ticks(1280*24);
        if(!dut.calibrated || !dut.measurement_ready || dut.sensor_fault)
            $fatal(1,"D/U calibration failed");
        guard_blind_scan=1;
        // Both manual paths: enter a rail, cross to the other rail, then return
        // upright. Real raw OTR at the rail is a known unobservable region.
        for(i=0;i<2;i=i+1) begin
            @(negedge clk); otr=1;
            settle(i==0 ? 0 : 1023);
            if(!dut.sample_blind || dut.measurement_ready || dut.measurement_valid)
                $fatal(1,"rail did not invalidate measurement");
            expect_idle;
            command("H"); expect_idle;
            key(0); key(3);
            if(dut.adc_down_q4!=220*16 || dut.adc_up_q4!=760*16)
                $fatal(1,"blind-zone D/U overwrote calibration");
            settle(i==0 ? 1023 : 0); expect_idle;
            @(negedge clk); otr=0;
            settle(760); expect_idle;
            if(!dut.measurement_ready || !dut.measurement_valid || dut.omega!=0)
                $fatal(1,"automatic recovery did not reprime derivatives and mature");
            command("H"); ticks(50);
            if(dut.state!=2 || !dut.motor_enable) $fatal(1,"fresh H after blind recovery rejected");
            // Entering a blind zone while running removes drive and requires a
            // fresh H, even after clean measurements return.
            settle(i==0 ? 0 : 1023); expect_idle;
            settle(760); expect_idle;
        end
        command("H"); ticks(50);
        // A single raw rail/OTR spike must not defeat median rejection and
        // directly disable a healthy control loop.
        @(negedge adc_clk); adc=0; otr=1;
        @(negedge adc_clk); adc=760; otr=0;
        ticks(1280*4);
        if(dut.state!=2 || dut.telemetry_fault || !dut.measurement_ready)
            $fatal(1,"rejected raw spike bypassed qualification");
        // Longer than median7 can reject, shorter than the confirmation bound.
        @(negedge adc_clk); adc=920;
        repeat(32) @(negedge adc_clk);
        adc=760; ticks(1280*4);
        if(dut.state!=2 || dut.control_q4!=760*16 || dut.telemetry_fault || !dut.measurement_ready)
            $fatal(1,"32-conversion non-rail spike reached the controller");
        guard_blind_scan=0;
        // A pre-existing controller fault must survive automatic blind-zone
        // handling, until the operator explicitly acknowledges it.
        for(i=759;i>=625;i=i-1) begin @(negedge clk); adc=i; ticks(1280); end
        if(dut.state!=3 || dut.fault!=8'h20) $fatal(1,"fall protection setup: %h",dut.fault);
        settle(0); settle(1023); settle(760);
        if(dut.state!=3 || dut.fault!=8'h20 || dut.drive_enabled)
            $fatal(1,"blind zone automatically cleared a pre-existing fall fault");
        command("S"); command("R"); command("H"); ticks(50);
        // Non-rail, sustained OTR still has its own real fault path.
        @(negedge clk); otr=1; ticks(1280*6);
        if(!dut.sample_fault || !dut.sensor_fault || !(dut.telemetry_fault & 1) || dut.drive_enabled)
            $fatal(1,"persistent non-rail OTR was hidden");
        command("S"); @(negedge clk); otr=0; settle(760);
        if(!dut.sensor_fault) $fatal(1,"hard fault cleared automatically");
        command("R"); ticks(20); expect_idle;
        if(dut.adc_down_q4!=220*16 || dut.adc_up_q4!=760*16)
            $fatal(1,"hard-fault recovery altered calibration");
        $display("PASS tb_blindzone_top: both rail directions, no false 0x01, no blind calibration/start, automatic measurement recovery, no motor restart, raw spike rejection, sustained OTR protection");
        $finish;
    end
endmodule
