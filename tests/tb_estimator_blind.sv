`timescale 1ns/1ps
module tb_estimator_blind;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0, sample_valid=0, sample_bad=0, sample_blind=0, sample_fault=0;
    reg [13:0] sample_q4=0;
    reg [7:0] quality=0;
    reg signed [31:0] position=0;
    reg down=0, up=0, stopped=1, clear_fault=0;
    wire calibrated, fault, valid, ready, measurement_valid, clear_ready;
    wire [15:0] reason;
    wire signed [15:0] theta, omega, arm, arm_speed;
    state_estimator dut(.clk(clk),.rst_n(rst_n),.sample_valid(sample_valid),
        .sample_q4(sample_q4),.sample_bad(sample_bad),.sample_blind(sample_blind),
        .sample_fault(sample_fault),.sample_quality_reason(quality),.position(position),
        .cal_down(down),.cal_up(up),.stopped(stopped),.clear_fault(clear_fault),
        .calibrated(calibrated),.sensor_fault(fault),.sensor_fault_reason(reason),
        .fault_clear_ready(clear_ready),.valid(valid),.measurement_ready(ready),
        .measurement_valid(measurement_valid),.theta(theta),.omega(omega),.arm(arm),.arm_speed(arm_speed));
    task ticks(input integer n);
        begin repeat(n) begin @(posedge clk); #1; end end
    endtask
    task sample(input integer code,input integer bad,input integer blind,input integer hard,input [7:0] raw_quality);
        integer timeout;
        begin
            @(negedge clk); sample_q4=code*16; sample_bad=bad; sample_blind=blind;
            sample_fault=hard; quality=raw_quality; sample_valid=1;
            @(negedge clk); sample_valid=0; timeout=0;
            while(!valid && timeout<12) begin ticks(1); timeout=timeout+1; end
            if(!valid) $fatal(1,"blind or rejected sample lost its freshness pulse");
            ticks(2);
        end
    endtask
    task endpoint(input integer is_up,input integer code);
        begin
            @(negedge clk); sample_q4=code*16; sample_valid=1; up=is_up; down=!is_up;
            @(negedge clk); sample_valid=0; up=0; down=0; ticks(40);
        end
    endtask
    integer i;
    reg signed [15:0] saved_theta;
    initial begin #2000000; $fatal(1,"estimator blind timeout"); end
    initial begin
        ticks(3); @(negedge clk); rst_n=1;
        endpoint(0,200); endpoint(1,800);
        if(!calibrated || fault) $fatal(1,"calibration");
        for(i=0;i<16;i=i+1) sample(800,0,0,0,0);
        if(!ready) $fatal(1,"initial maturity");
        // Raw rail and OTR warnings were rejected before control averaging.
        sample(799,0,0,0,8'h23);
        if(!measurement_valid || !ready || fault || omega==0)
            $fatal(1,"raw diagnostics bypassed the qualified control sample");
        saved_theta=theta; stopped=0;
        // Seam mixture near the ADC middle cannot silently look upright.
        for(i=0;i<20;i=i+1) begin
            position=position+1;
            sample(i[0] ? 0 : 512,1,1,0,8'h6f);
            if(fault || measurement_valid || ready || theta!=saved_theta || omega!=0 || arm_speed!=0)
                $fatal(1,"blind region polluted state or latched a fault");
        end
        sample(1023,1,1,0,8'h03);
        sample(800,0,0,0,0);
        if(fault || !measurement_valid || ready || theta!=0 || omega!=0 || arm_speed!=0)
            $fatal(1,"blind exit manufactured a derivative");
        for(i=1;i<16;i=i+1) begin
            sample(800,0,0,0,0);
            if(i<15 && ready) $fatal(1,"blind recovery became ready before 16 healthy windows");
        end
        if(!ready || fault) $fatal(1,"blind recovery required a fault acknowledge");
        // A qualified electrical fault retains immediate latching semantics.
        sample(800,1,0,1,8'h01);
        if(!fault || !reason[0] || measurement_valid || ready) $fatal(1,"qualified hardware failure lost");
        for(i=0;i<16;i=i+1) sample(800,0,0,0,0);
        if(!fault || clear_ready) $fatal(1,"running recovery auto-cleared or allowed R");
        stopped=1; ticks(1);
        if(!clear_ready) $fatal(1,"stopped healthy recovery cannot be acknowledged");
        @(negedge clk); clear_fault=1;
        @(negedge clk); clear_fault=0; ticks(2);
        if(fault || reason!=0 || !calibrated) $fatal(1,"explicit recovery failed");
        $display("PASS tb_estimator_blind: raw diagnostic separation, seam quarantine, freshness, derivative reset, 16-window recovery and hard fault");
        $finish;
    end
endmodule
