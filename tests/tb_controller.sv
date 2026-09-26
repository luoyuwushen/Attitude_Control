`timescale 1ns/1ps
module tb_controller;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst_n = 0, sample_valid = 0, calibrated = 1, sensor_fault = 0;
    reg start = 0, stop = 0, clear = 0;
    reg signed [15:0] theta = 3217, omega = 0, arm = 0, arm_speed = 0;
    wire [2:0] state;
    wire [7:0] fault;
    wire signed [15:0] command;
    wire enable;
    // 保留默认40ms启动/15s超时参数，仅缩短看门狗模拟时长。
    attitude_controller #(.WATCHDOG_CYCLES(30)) dut (
        .clk(clk), .rst_n(rst_n), .sample_valid(sample_valid), .calibrated(calibrated),
        .sensor_fault(sensor_fault), .start(start), .stop(stop), .clear(clear),
        .theta(theta), .omega(omega), .arm(arm), .arm_speed(arm_speed),
        .state(state), .fault(fault), .command(command), .enable(enable));
    task tick(input integer cycles);
        begin repeat(cycles) begin @(posedge clk); #1; end end
    endtask
    task start_run;
        begin
            @(negedge clk); start = 1;
            @(negedge clk); start = 0; tick(1);
            if (state !== 1 || !enable) $fatal(1, "controller valid start");
        end
    endtask
    task stop_run;
        begin
            @(negedge clk); stop = 1; #1;
            if (enable !== 0) $fatal(1, "stop must immediately gate enable");
            tick(1);
            if (state !== 0 || fault !== 0 || command !== 0) $fatal(1, "stop priority/state");
            @(negedge clk); stop = 0;
        end
    endtask
    task sample(input integer angle, input integer angular_speed,
                input integer arm_angle, input integer arm_velocity);
        begin
            @(negedge clk);
            theta = angle; omega = angular_speed; arm = arm_angle; arm_speed = arm_velocity;
            sample_valid = 1;
            @(negedge clk); sample_valid = 0;
            tick(6);
            if (command > 1000 || command < -1000) $fatal(1, "controller command outside bounds");
        end
    endtask
    task expect_fault(input [7:0] mask);
        begin
            if (state !== 3 || fault !== mask || enable !== 0 || command !== 0)
                $fatal(1, "fault expected %02x got state=%0d fault=%02x cmd=%0d",mask,state,fault,command);
        end
    endtask
    task enter_balance;
        begin
            start_run; sample(0,0,0,0);
            if (state !== 2) $fatal(1, "upright sample did not capture balance");
        end
    endtask
    integer k, expected_index, expected_cos;
    real cosine_reference;
    initial begin #5000000; $fatal(1, "tb_controller timeout"); end
    initial begin
        tick(2);
        if (state !== 0 || command !== 0 || enable !== 0) $fatal(1, "controller reset");
        @(negedge clk); rst_n = 1;
        calibrated = 0; start = 1;
        tick(2); @(negedge clk); start = 0;
        if (state !== 0 || enable) $fatal(1, "uncalibrated start accepted");
        calibrated = 1; sensor_fault = 1;
        @(negedge clk); start = 1; tick(2);
        @(negedge clk); start = 0;
        if (state !== 0 || enable) $fatal(1, "sensor-fault start accepted");
        // stop 必须同时覆盖 start/sensor_fault/clear。
        @(negedge clk); start = 1; stop = 1; clear = 1;
        tick(1);
        if (state !== 0 || fault !== 0 || command !== 0) $fatal(1, "stop highest priority");
        @(negedge clk); start = 0; stop = 0; clear = 0; sensor_fault = 0;

        start_run;
        for (k=0; k<40; k=k+1) begin
            sample(3217,-1000,0,0);
            if (command !== 667 || state !== 1) $fatal(1, "40ms starting kick");
        end
        // 能量不足时 omega*cos(theta) 为负则反向；能量过量时方向反转。
        sample(3217,1000,0,0); if (command !== -667) $fatal(1, "energy positive/negative pump");
        sample(3217,-1000,0,0); if (command !== 667) $fatal(1, "energy positive/positive pump");
        sample(1000,1000,0,0); if (command !== 667) $fatal(1, "energy cos positive");
        sample(1000,25000,0,0); if (command !== -667) $fatal(1, "energy excess sign");
        sample(3217,25000,0,0); if (command !== 667) $fatal(1, "energy excess negative cosine");
        // 20rad/s时动能项已超过32位乘积容量，必须保持正确的过能方向。
        sample(3217,20480,0,0);
        if (dut.energy_deficit !== -32'sd39 || command !== 667)
            $fatal(1, "20rad/s energy overflow or sign");
        sample(3217,-20480,0,0);
        if (dut.energy_deficit !== -32'sd39 || command !== -667)
            $fatal(1, "negative 20rad/s energy overflow or sign");
        sample(3217,-1000,-6000,-20000);
        if (command !== 800) $fatal(1, "swing positive command limit");
        sample(3217,1000,6000,20000);
        if (command !== -800) $fatal(1, "swing negative command limit");
        // 停止打断尚未提交的计算，流水线后续不能恢复旧力矩。
        @(negedge clk); theta = 3217; omega = -1000; arm = 0; arm_speed = 0; sample_valid = 1;
        @(negedge clk); sample_valid = 0;
        stop_run;
        tick(5);
        if (command !== 0 || enable) $fatal(1, "stop did not cancel in-flight command");
        tick(40);
        if (state !== 0 || enable) $fatal(1, "stop release auto-restarted");

        // 与精确整数除法逐值对照，确保时序优化未改变LUT分段边界。
        start_run;
        for (k=0; k<=3217; k=k+1) begin
            expected_index = k*32/3217;
            cosine_reference = 1024.0*$cos(expected_index*3.141592653589793/32.0);
            expected_cos = $rtoi(cosine_reference+(cosine_reference < 0 ? -0.5 : 0.5));
            sample(k,4000,0,0);
            if (dut.lut_index !== expected_index || dut.cos_theta !== expected_cos)
                $fatal(1, "positive LUT floor/cos theta=%0d index=%0d expected=%0d",k,dut.lut_index,expected_index);
            sample(-k,4000,0,0);
            if (dut.lut_index !== expected_index || dut.cos_theta !== expected_cos)
                $fatal(1, "negative LUT absolute angle theta=%0d",k);
        end
        stop_run;

        start_run;
        sample(268,0,0,0); if (state !== 1) $fatal(1, "capture theta boundary");
        sample(267,3584,0,0); if (state !== 1) $fatal(1, "capture omega boundary");
        sample(267,3583,0,0); if (state !== 2) $fatal(1, "capture in permitted region");
        stop_run;

        enter_balance;
        sample(100,0,0,0);
        if (command < -230 || command > -220) $fatal(1, "positive theta feedback");
        sample(-100,0,0,0);
        if (command < 220 || command > 230) $fatal(1, "negative theta feedback");
        sample(0,0,1000,0);
        if (command < 20 || command > 35) $fatal(1, "arm restoring gain during capture");
        sample(0,0,0,1000);
        if (command < 105 || command > 120) $fatal(1, "arm speed damping sign");
        sample(600,10000,0,0); if (command !== -1000) $fatal(1, "LQR negative saturation");
        sample(-600,-10000,0,0); if (command !== 1000) $fatal(1, "LQR positive saturation");
        @(negedge clk); clear = 1; tick(1);
        if (state !== 2) $fatal(1, "clear interrupted running controller");
        @(negedge clk); clear = 0;
        sample(716,0,0,0); expect_fault(8'h20);
        @(negedge clk); start = 1; tick(1);
        if (state !== 3) $fatal(1, "fault bypassed by start");
        @(negedge clk); start = 0; clear = 1; tick(1);
        if (state !== 0 || fault !== 0) $fatal(1, "fault clear");
        @(negedge clk); clear = 0;

        start_run; sample(3217,0,6145,0); expect_fault(8'h08); stop_run;
        start_run; sample(3217,0,-6145,0); expect_fault(8'h08); stop_run;
        start_run; sample(3217,0,0,20481); expect_fault(8'h08); stop_run;
        start_run; sample(3217,-30721,0,0); expect_fault(8'h08); stop_run;
        start_run; @(negedge clk); sensor_fault = 1; tick(1);
        expect_fault(8'h01); stop_run; sensor_fault = 0;
        start_run; @(negedge clk); calibrated = 0; tick(1);
        expect_fault(8'h02); stop_run; calibrated = 1;
        start_run; tick(31); expect_fault(8'h04); stop_run;

        // 默认15000个1kHz样本为15s；仿真压缩样本间距，保持状态计数含义。
        start_run;
        for (k=0; k<15000; k=k+1) begin
            sample(3217,0,0,0);
            if (state !== 1) $fatal(1, "swing timeout fired too early sample=%0d",k);
        end
        sample(3217,0,0,0); expect_fault(8'h10); stop_run;
        $display("PASS tb_controller: stop/calibration/energy/LQR/capture/limits/watchdog/timeout");
        $finish;
    end
endmodule
