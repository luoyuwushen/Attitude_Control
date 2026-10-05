`timescale 1ns/1ps
module tb_estimator;
    reg clk = 0;
    always #10 clk = ~clk;
    reg rst_n = 0;

    reg [9:0] adc_data = 0;
    reg adc_otr = 0;
    wire adc_clock, adc_oe_n, adc_valid;
    wire adc_over_range;
    wire [13:0] adc_sample;
    adc_sampler #(.SAMPLE_CYCLES(10)) adc_dut (
        .clk(clk), .rst_n(rst_n), .data_in(adc_data), .otr(adc_otr), .motor_observe(4'd0), .adc_clk(adc_clock),
        .adc_oe_n(adc_oe_n), .sample_q4(adc_sample), .valid(adc_valid),
        .over_range(adc_over_range));
    reg adc_source_active = 0;
    integer source_code = 600;
    time last_rise = 0, previous_rise = 0, previous_sample = 0;
    integer adc_samples = 0;
    // Datasheet Fig.22 references output changes to the falling edge;
    // 25 ns is typical tOD, not a guaranteed maximum.
    always @(negedge adc_clock) begin
        if (adc_source_active) begin
            if (previous_rise != 0 && $time-previous_rise != 200)
                $fatal(1, "ADC clock must be 5MHz");
            previous_rise = $time;
            last_rise = $time;
            #25; adc_data = source_code;
        end
    end
    always @(posedge clk) begin
        #1;
        if (rst_n && adc_valid) begin
            adc_samples = adc_samples + 1;
            if (adc_sample !== 14'd9600) $fatal(1, "ADC coherent 16-sample sum: %0d", adc_sample);
            if (previous_sample != 0 && $time-previous_sample != 3200)
                $fatal(1, "ADC decimation interval");
            previous_sample = $time;
        end
    end

    reg div_start = 0;
    reg [13:0] denominator = 0;
    wire [31:0] quotient;
    wire div_busy, div_done;
    calibration_divider div_dut (
        .clk(clk), .rst_n(rst_n), .start(div_start), .denominator(denominator),
        .quotient(quotient), .busy(div_busy), .done(div_done));

    reg sample_valid = 0;
    reg [13:0] sample_q4 = 0;
    reg sample_bad = 0;
    reg signed [31:0] position = 0;
    reg cal_down = 0, cal_up = 0, stopped = 1;
    wire calibrated, sensor_fault, valid;
    wire signed [15:0] theta, omega, arm, arm_speed;
    wire calibrated_neg, sensor_fault_neg, valid_neg;
    wire signed [15:0] theta_neg, omega_neg, arm_neg, arm_speed_neg;
    state_estimator estimator_dut (
        .clk(clk), .rst_n(rst_n), .sample_valid(sample_valid), .sample_q4(sample_q4), .sample_bad(sample_bad),
        .sample_blind(1'b0),.sample_fault(1'b0),.sample_quality_reason(8'd0), .clear_fault(1'b0), .position(position), .cal_down(cal_down), .cal_up(cal_up), .stopped(stopped),
        .calibrated(calibrated), .sensor_fault(sensor_fault), .valid(valid),
        .theta(theta), .omega(omega), .arm(arm), .arm_speed(arm_speed));
    state_estimator #(.THETA_SIGN(-1), .ENCODER_SIGN(-1)) reverse_dut (
        .clk(clk), .rst_n(rst_n), .sample_valid(sample_valid), .sample_q4(sample_q4), .sample_bad(sample_bad),
        .sample_blind(1'b0),.sample_fault(1'b0),.sample_quality_reason(8'd0), .clear_fault(1'b0), .position(position), .cal_down(cal_down), .cal_up(cal_up), .stopped(stopped),
        .calibrated(calibrated_neg), .sensor_fault(sensor_fault_neg), .valid(valid_neg),
        .theta(theta_neg), .omega(omega_neg), .arm(arm_neg), .arm_speed(arm_speed_neg));
    integer estimator_samples = 0;
    integer saved_samples;
    reg signed [15:0] saved_theta, saved_theta_neg;
    reg previous_valid = 0, previous_done = 0;
    always @(posedge clk) begin
        #2;
        if (rst_n) begin
            if (valid) estimator_samples = estimator_samples + 1;
            if (valid && previous_valid) $fatal(1, "estimator valid pulse width");
            if (div_done && previous_done) $fatal(1, "divider done pulse width");
            if (valid !== valid_neg || calibrated !== calibrated_neg || sensor_fault !== sensor_fault_neg)
                $fatal(1, "sign parameters altered estimator sequencing");
        end
        previous_valid = valid;
        previous_done = div_done;
    end

    task tick(input integer cycles);
        begin repeat(cycles) begin @(posedge clk); #3; end end
    endtask
    task check_division(input integer value);
        integer timeout;
        begin
            @(negedge clk); denominator = value; div_start = 1;
            @(negedge clk); div_start = 0;
            // 输入在运行途中改变，结果必须仍对应已捕获的除数。
            denominator = 14'd7;
            timeout = 0;
            while (!div_done && timeout < 40) begin tick(1); timeout = timeout + 1; end
            if (!div_done || div_busy || quotient !== 3294199/value)
                $fatal(1, "divider d=%0d q=%0d", value, quotient);
            tick(2);
        end
    endtask
    task calibrate(input integer down, input integer up);
        begin
            stopped = 1;
            @(negedge clk); sample_q4 = down*16; sample_valid = 1; cal_down = 1;
            @(negedge clk); sample_valid = 0; cal_down = 0;
            tick(2);
            @(negedge clk); sample_q4 = up*16; sample_valid = 1; cal_up = 1;
            @(negedge clk); sample_valid = 0; cal_up = 0;
            tick(40);
            if (!calibrated || sensor_fault) $fatal(1, "two-point calibration rejected");
        end
    endtask
    task sample(input integer q4, input integer counts);
        integer before_samples, timeout;
        begin
            before_samples = estimator_samples;
            @(negedge clk); sample_q4 = q4; position = counts; sample_valid = 1;
            @(negedge clk); sample_valid = 0;
            timeout = 0;
            while (estimator_samples == before_samples && timeout < 20) begin
                tick(1); timeout = timeout + 1;
            end
            if (estimator_samples != before_samples+1) $fatal(1, "estimator sample completion");
            tick(2);
        end
    endtask
    initial begin #2000000; $fatal(1, "tb_estimator timeout"); end
    initial begin
        tick(2);
        @(negedge clk); rst_n = 1; adc_source_active = 1;
        tick(900);
        if (adc_samples < 1 || adc_oe_n !== 0) $fatal(1, "ADC startup/output enable");
        @(negedge clk); adc_otr = 1; tick(20);
        if (adc_over_range !== 1) $fatal(1, "ADC OTR assertion");
        @(negedge clk); adc_otr = 0; tick(20);
        if (adc_over_range !== 0) $fatal(1, "ADC OTR release");
        check_division(1); check_division(2816); check_division(512);
        check_division(16383); check_division(777);
        @(negedge clk); denominator = 0; div_start = 1;
        @(negedge clk); div_start = 0;
        tick(40);
        if (div_busy || div_done) $fatal(1, "zero denominator should not start");

        calibrate(624,800);
        sample(800*16,0);
        if (theta !== 0 || omega !== 0 || arm !== 0 || arm_speed !== 0)
            $fatal(1, "upper reference/origin/startup speed");
        stopped = 0;
        sample(790*16,1);
        if (theta < 180 || theta > 185 || omega < 22000 || omega > 24000 ||
            arm <= 0 || arm_speed <= 0 || theta_neg >= 0 || omega_neg >= 0 ||
            arm_neg >= 0 || arm_speed_neg >= 0)
            $fatal(1, "Q10 angle/velocity or sign parameters");
        stopped = 1;
        sample(785*16,2);
        if (omega <= 0 || arm_speed <= 0) $fatal(1, "IDLE motion was replaced by zero velocity");

        // 下点附近跨 ±pi，角速度必须取最短角度差。
        calibrate(624,800);
        sample(624*16+16,2);
        if (theta < 3190 || theta > 3217) $fatal(1, "down reference should approach +pi");
        stopped = 0;
        sample(624*16-16,2);
        if (theta > -3180 || theta < -3217 || omega < 3000 || omega > 6000 || sensor_fault)
            $fatal(1, "angle wrap/shortest derivative");

        // 运行中校准请求必须无效。
        @(negedge clk); cal_down = 1;
        @(negedge clk); cal_down = 0; tick(3);
        if (!calibrated) $fatal(1, "running calibration changed state");
        sample(800*16,2);
        if (sensor_fault || estimator_dut.measurement_valid || omega!=0)
            $fatal(1, "isolated ADC discontinuity must be quarantined without a latched fault");
        stopped = 1;
        calibrate(624,800);
        sample(0,2);
        if (sensor_fault || estimator_dut.measurement_valid || omega!=0)
            $fatal(1, "ADC blind rail must remain unmeasurable without a latched fault");

        // 校准请求取消尚在流水线中的样本，旧样本不能恢复 primed/valid。
        calibrate(624,800);
        sample(800*16,2);
        saved_samples = estimator_samples;
        @(negedge clk); sample_q4 = 790*16; sample_valid = 1;
        @(negedge clk); sample_valid = 0; sample_q4 = 700*16; cal_down = 1;
        @(negedge clk); cal_down = 0;
        tick(15);
        if (calibrated || estimator_samples != saved_samples)
            $fatal(1, "calibration did not cancel in-flight sample");

        // 下点重设与除法完成同时到来时，新下点必须优先撤销校准。
        @(negedge clk); sample_q4 = 800*16; cal_up = 1;
        @(negedge clk); cal_up = 0;
        tick(33);
        @(negedge clk); sample_q4 = 624*16; cal_down = 1;
        @(negedge clk); cal_down = 0;
        tick(3);
        if (calibrated) $fatal(1, "divider completion overrode new calibration");

        // 相反的传感器电压斜率仍应把下点映射到 +pi。
        calibrate(800,624);
        sample(624*16,2);
        if (theta !== 0) $fatal(1, "negative-span upper reference");
        calibrate(800,624);
        sample(800*16,2);
        if (theta < 3200 || theta > 3217 || theta_neg > -3200 || theta_neg < -3217)
            $fatal(1, "negative-span down reference");

        // 无效标定跨度应撤销校准并置故障。
        @(negedge clk); sample_q4 = 700*16; cal_down = 1;
        @(negedge clk); cal_down = 0;
        @(negedge clk); sample_q4 = 701*16; cal_up = 1;
        @(negedge clk); cal_up = 0; tick(40);
        if (calibrated || !sensor_fault) $fatal(1, "invalid calibration span accepted");

        // 超出标定传感器可接受范围的样本报故障并保留上次可靠角度。
        calibrate(640,672);
        saved_theta = theta; saved_theta_neg = theta_neg;
        sample(256*16,2);
        if (!sensor_fault || theta !== saved_theta || theta_neg !== saved_theta_neg ||
            omega !== 0 || arm_speed !== 0)
            $fatal(1, "out-of-range angle not rejected safely");
        tick(200);
        if (adc_samples < 2) $fatal(1, "ADC periodic samples missing");
        $display("PASS tb_estimator: ADC timing/average/divider/calibration/Q10/wrap/fault");
        $finish;
    end
endmodule
