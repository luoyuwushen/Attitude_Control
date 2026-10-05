`timescale 1ns/1ps
module tb_sensor_recovery;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0, sample_valid=0, sample_bad=0, cal_down=0, cal_up=0, stopped=1, clear_fault=0;
    reg [7:0] reason=0;
    reg [13:0] sample_q4=220*16;
    wire calibrated, sensor_fault, valid, measurement_ready, measurement_valid, fault_clear_ready;
    wire [15:0] sensor_fault_reason;
    wire [13:0] down_q4, up_q4;
    state_estimator #(.SAMPLE_FRESH_CYCLES(200)) dut(
        .clk(clk),.rst_n(rst_n),.sample_valid(sample_valid),.sample_q4(sample_q4),
        .sample_bad(sample_bad),.sample_blind(1'b0),.sample_fault(1'b0),.sample_quality_reason(reason),.position(32'sd0),
        .cal_down(cal_down),.cal_up(cal_up),.stopped(stopped),.clear_fault(clear_fault),
        .calibrated(calibrated),.sensor_fault(sensor_fault),.valid(valid),
        .sensor_fault_reason(sensor_fault_reason),.fault_clear_ready(fault_clear_ready),
        .adc_down_q4(down_q4),.adc_up_q4(up_q4),
        .measurement_ready(measurement_ready),.measurement_valid(measurement_valid));
    task ticks(input integer n);
        begin repeat(n) begin @(posedge clk); #1; end end
    endtask
    task sample(input integer code,input integer bad,input [7:0] why);
        begin
            @(negedge clk); sample_q4=code*16; sample_bad=bad; reason=why; sample_valid=1;
            @(negedge clk); sample_valid=0; ticks(12);
        end
    endtask
    task acknowledge;
        begin @(negedge clk); clear_fault=1; ticks(1); @(negedge clk); clear_fault=0; ticks(1); end
    endtask
    task endpoint(input integer up,input integer code);
        begin
            sample(code,0,0);
            @(negedge clk); cal_up=up; cal_down=!up;
            @(negedge clk); cal_up=0; cal_down=0; ticks(40);
        end
    endtask
    integer i;
    initial begin #2000000; $fatal(1,"sensor recovery timeout"); end
    initial begin
        ticks(3); @(negedge clk); rst_n=1;
        // Transients are unavailable, sustained corruption remains latched.
        // R only clears after 16 fresh good windows, without inventing cal.
        sample(220,1,8'h04);
        if(sensor_fault) $fatal(1,"one noisy window was latched as failure");
        for(i=1;i<8;i=i+1) sample(220,1,8'h04);
        if(!sensor_fault || sensor_fault_reason!=4) $fatal(1,"missing first-window reason");
        sample(220,1,8'h01);
        if(sensor_fault_reason!=4) $fatal(1,"later fault overwrote first sensor reason");
        acknowledge;
        if(!sensor_fault || fault_clear_ready) $fatal(1,"R cleared ongoing bad window");
        for(i=0;i<15;i=i+1) sample(220,0,0);
        acknowledge;
        if(!sensor_fault || fault_clear_ready) $fatal(1,"R accepted before 16 good windows");
        sample(220,0,0); acknowledge;
        if(sensor_fault || sensor_fault_reason || calibrated) $fatal(1,"stopped uncalibrated recovery");
        endpoint(0,220); endpoint(1,760);
        if(!calibrated || sensor_fault) $fatal(1,"healthy calibration failed");
        for(i=0;i<16;i=i+1) sample(760,0,0);
        for(i=0;i<8;i=i+1) sample(760,1,8'h08);
        if(!sensor_fault || sensor_fault_reason!=8 || measurement_ready) $fatal(1,"filtered step fault");
        for(i=0;i<16;i=i+1) sample(760,0,0);
        if(!sensor_fault || !fault_clear_ready) $fatal(1,"sensor recovery auto-cleared or never matured");
        stopped=0; acknowledge;
        if(!sensor_fault || fault_clear_ready) $fatal(1,"running R cleared sensor fault");
        stopped=1; ticks(201); acknowledge;
        if(!sensor_fault || fault_clear_ready) $fatal(1,"stale R cleared sensor fault");
        for(i=0;i<16;i=i+1) sample(760,0,0);
        acknowledge;
        if(sensor_fault || sensor_fault_reason || !calibrated || down_q4!=220*16 || up_q4!=760*16)
            $fatal(1,"R altered healthy calibration or retained acknowledged reason");
        // Raw-span warning is informational; hard samples and estimator
        // discontinuities still retain their own exact reason.
        sample(760,0,8'h20);
        if(sensor_fault || !measurement_ready) $fatal(1,"raw-span warning became sensor fault");
        sample(800,0,0);
        if(sensor_fault || measurement_ready || measurement_valid) $fatal(1,"single discontinuity was not quarantined");
        for(i=1;i<8;i=i+1) sample(i[0] ? 760 : 800,0,0);
        if(!sensor_fault || sensor_fault_reason!=16'h0100) $fatal(1,"mean discontinuity reason");
        $display("PASS tb_sensor_recovery: explicit R/fresh16/stopped/calibration retained/reasons/no auto clear");
        $finish;
    end
endmodule
