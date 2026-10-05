`timescale 1ns/1ps
module tb_estimator_motion;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0, sample_valid=0, sample_bad=0;
    reg [13:0] sample_q4=0;
    reg signed [31:0] position=0;
    reg cal_down=0, cal_up=0, stopped=1;
    wire calibrated, sensor_fault, valid, measurement_ready, measurement_valid;
    wire signed [15:0] theta, omega, arm, arm_speed;
    wire [13:0] adc_down_q4, adc_up_q4;
    state_estimator #(.SAMPLE_FRESH_CYCLES(200)) dut(
        .clk(clk),.rst_n(rst_n),.sample_valid(sample_valid),.sample_q4(sample_q4),
        .sample_bad(sample_bad),.sample_blind(1'b0),.sample_fault(1'b0),.sample_quality_reason(8'd0),.clear_fault(1'b0),.position(position),.cal_down(cal_down),.cal_up(cal_up),
        .stopped(stopped),.calibrated(calibrated),.sensor_fault(sensor_fault),.valid(valid),
        .theta(theta),.omega(omega),.arm(arm),.arm_speed(arm_speed),
        .adc_down_q4(adc_down_q4),.adc_up_q4(adc_up_q4),
        .measurement_ready(measurement_ready),.measurement_valid(measurement_valid));
    task tick(input integer n);
        begin repeat(n) begin @(posedge clk); #1; end end
    endtask
    // Sample pulses represent consecutive 1 ms measurements. The digital
    // pipeline and freshness timeout are accelerated independently in this TB.
    task sample(input integer q4, input integer counts, input integer bad);
        integer timeout;
        begin
            @(negedge clk); sample_q4=q4; position=counts; sample_bad=bad; sample_valid=1;
            @(negedge clk); sample_valid=0;
            timeout=0;
            while(!valid && timeout<20) begin tick(1); timeout=timeout+1; end
            if(!valid) $fatal(1,"sample completion missing, including rejected samples");
            tick(2);
        end
    endtask
    task endpoint(input integer up, input integer q4, input integer bad);
        begin
            @(negedge clk); sample_q4=q4; sample_bad=bad; sample_valid=1;
            cal_up=up; cal_down=!up;
            @(negedge clk); sample_valid=0; cal_up=0; cal_down=0;
            tick(40);
        end
    endtask
    integer i;
    reg signed [15:0] saved_theta, saved_omega;
    initial begin #2000000; $fatal(1,"tb_estimator_motion timeout"); end
    initial begin
        tick(3); @(negedge clk); rst_n=1;
        // No sample has arrived: an endpoint request is not evidence of freshness.
        @(negedge clk); sample_q4=240*16; cal_down=1;
        @(negedge clk); cal_down=0; tick(2);
        if(adc_down_q4!=0 || !sensor_fault || measurement_valid || measurement_ready)
            $fatal(1,"calibration accepted a never-observed ADC window");
        endpoint(0,240*16,0); endpoint(1,800*16,0);
        if(!calibrated || sensor_fault || adc_down_q4!=240*16 || adc_up_q4!=800*16)
            $fatal(1,"trusted calibration/endpoints");
        if(measurement_valid || measurement_ready) $fatal(1,"calibration invented a measurement");
        sample(800*16,0,0);
        if(!measurement_valid || measurement_ready || omega!=0) $fatal(1,"first sample validity/maturity");
        for(i=1;i<16;i=i+1) begin
            sample((800-i)*16,i,0);
            if(i<15 && measurement_ready) $fatal(1,"velocity declared mature before 16 samples");
        end
        if(!measurement_ready || !measurement_valid || omega<=3584 || arm_speed<=3584)
            $fatal(1,"IDLE motion hidden or incorrectly immature: omega=%0d speed=%0d",omega,arm_speed);
        saved_omega=omega;
        stopped=0; sample(784*16,16,0);
        if(omega<saved_omega*3/4 || !measurement_ready) $fatal(1,"start reset continuous velocity");
        tick(201);
        if(!measurement_ready || !measurement_valid)
            $fatal(1,"age prematurely changed stored quality before the top-level watchdog");
        sample(780*16,40,0);
        if(measurement_ready || !measurement_valid || omega!=0 || arm_speed!=0 || sensor_fault)
            $fatal(1,"resumed sample interpreted a long gap as a 1ms derivative");
        for(i=1;i<16;i=i+1) begin
            sample((780-i)*16,40+i,0);
            if(i<15 && measurement_ready) $fatal(1,"gap recovery became mature too early");
        end
        if(!measurement_ready) $fatal(1,"gap recovery never matured");
        stopped=1; saved_theta=theta;
        sample(784*16,16,1);
        if(sensor_fault || measurement_valid || measurement_ready || theta!=saved_theta || omega!=0)
            $fatal(1,"isolated bad window was not safely quarantined");
        for(i=1;i<8;i=i+1) sample(784*16,16,1);
        if(!sensor_fault || measurement_valid || measurement_ready || theta!=saved_theta || omega!=0)
            $fatal(1,"bad window promoted to a valid/mature state");
        for(i=0;i<16;i=i+1) begin
            sample((784-i)*16,16+i,0);
            if(i<15 && measurement_ready) $fatal(1,"post-fault maturity too early");
        end
        if(!sensor_fault || !measurement_valid || !measurement_ready || omega<=3584)
            $fatal(1,"trusted recovery must retain fault but resume continuous measurement");
        endpoint(0,400*16,1); endpoint(1,700*16,1);
        if(adc_down_q4!=240*16 || adc_up_q4!=800*16 || !sensor_fault || measurement_ready)
            $fatal(1,"bad D/U overwrote endpoints or cleared fault");
        sample(769*16,31,0); tick(201);
        if(!measurement_valid || measurement_ready)
            $fatal(1,"sample age changed stored quality; top-level owns freshness gating");
        @(negedge clk); cal_down=1;
        @(negedge clk); cal_down=0; tick(2);
        @(negedge clk); cal_up=1;
        @(negedge clk); cal_up=0; tick(2);
        if(adc_down_q4!=240*16 || adc_up_q4!=800*16 || !sensor_fault)
            $fatal(1,"stale D/U overwrote endpoints or cleared fault");
        // Keep the Q4 mean, including its fractional part, as the endpoint.
        endpoint(0,3847,0); endpoint(1,12809,0);
        if(!calibrated || sensor_fault || adc_down_q4!=3847 || adc_up_q4!=12809)
            $fatal(1,"calibration truncated or used something other than the trusted mean");
        sample(12809,31,0);
        if(theta!=0 || !measurement_valid || measurement_ready) $fatal(1,"fractional mean up reference");
        endpoint(0,0,0);
        if(adc_down_q4!=3847 || sensor_fault) $fatal(1,"blind rail endpoint accepted or latched a fault");
        $display("PASS tb_estimator_motion: continuous idle velocity/maturity/quality/fresh calibration/recovery");
        $finish;
    end
endmodule
