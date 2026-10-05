`timescale 1ns/1ps
// Independent H admission contract. No internal state or protection is forced.
module tb_balance_entry;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst_n = 0, sample_valid = 0, calibrated = 1, sensor_fault = 0;
    reg start = 0, start_balance = 0, measurement_ready = 1;
    reg stop = 0, clear = 0;
    reg signed [15:0] theta = 0, omega = 0, arm = 0, arm_speed = 0;
    wire [2:0] state;
    wire [7:0] fault;
    wire signed [15:0] command;
    wire enable;
    attitude_controller #(.WATCHDOG_CYCLES(64)) dut (
        .clk(clk), .rst_n(rst_n), .sample_valid(sample_valid),
        .calibrated(calibrated), .sensor_fault(sensor_fault),
        .start(start), .start_balance(start_balance), .measurement_ready(measurement_ready),
        .stop(stop), .clear(clear), .theta(theta), .omega(omega),
        .arm(arm), .arm_speed(arm_speed), .state(state), .fault(fault),
        .command(command), .enable(enable));

    task ticks(input integer count);
        begin repeat(count) begin @(posedge clk); #1; end end
    endtask
    task boot;
        begin
            @(negedge clk);
            rst_n=0; sample_valid=0; calibrated=1; sensor_fault=0;
            start=0; start_balance=0; measurement_ready=1; stop=0; clear=0;
            theta=0; omega=0; arm=0; arm_speed=0;
            ticks(2); @(negedge clk); rst_n=1; ticks(1);
        end
    endtask
    task request(input bit automatic_start, input bit balance_start);
        begin
            @(negedge clk); start=automatic_start; start_balance=balance_start;
            ticks(1);
            @(negedge clk); start=0; start_balance=0;
        end
    endtask
    task idle_required;
        begin
            if(state!==0 || fault!==0 || command!==0 || enable!==0)
                $fatal(1,"rejected H/G moved state=%0d fault=%02x command=%0d",state,fault,command);
        end
    endtask
    task balance_required;
        begin
            if(state!==2 || fault!==0 || enable!==1)
                $fatal(1,"H must enter BALANCE directly, state=%0d fault=%02x",state,fault);
        end
    endtask
    task fault_required(input [7:0] expected);
        begin
            if(state!==3 || fault!==expected || command!==0 || enable!==0)
                $fatal(1,"H protection expected %02x, got state=%0d fault=%02x command=%0d",
                       expected,state,fault,command);
        end
    endtask
    task sample(input integer angle, input integer speed,
                input integer position, input integer velocity);
        begin
            @(negedge clk);
            theta=angle; omega=speed; arm=position; arm_speed=velocity; sample_valid=1;
            @(negedge clk); sample_valid=0;
            ticks(6);
        end
    endtask
    task reject_limits(input integer position, input integer velocity, input integer pendulum_velocity);
        begin
            boot; arm=position; arm_speed=velocity; omega=pendulum_velocity;
            request(1,0); idle_required;
            request(0,1); idle_required;
            request(1,1); idle_required;
            // Fresh samples after rejection cannot resurrect a queued start.
            sample(0,pendulum_velocity,position,velocity); idle_required;
        end
    endtask

    initial begin #200000; $fatal(1,"tb_balance_entry timeout"); end
    initial begin
        // Both entry commands require mature, trustworthy measurements.
        boot; measurement_ready=0; theta=3217; request(1,0); idle_required;
        theta=0; request(0,1); idle_required;
        calibrated=0; measurement_ready=1; request(0,1); idle_required;
        calibrated=1; sensor_fault=1; request(0,1); idle_required;

        // Admission uses the same limits as running protection, including the
        // unsigned magnitude of the most-negative signed 16-bit input.
        reject_limits(6145,0,0); reject_limits(-6145,0,0);
        reject_limits(0,20481,0); reject_limits(0,-20481,0);
        reject_limits(0,0,30721); reject_limits(0,0,-30721);
        reject_limits(-32768,0,0); reject_limits(0,-32768,0);
        reject_limits(0,0,-32768);
        boot; theta=3217; arm=6144; arm_speed=20480; omega=30720;
        request(1,0);
        if(state!==1 || !enable) $fatal(1,"positive safety boundary wrongly rejected");
        sample(3217,30720,6144,20480);
        if(state!==1 || fault) $fatal(1,"positive safety boundary failed during run");
        boot; theta=3217; arm=-6144; arm_speed=-20480; omega=-30720;
        request(1,0);
        if(state!==1 || !enable) $fatal(1,"negative safety boundary wrongly rejected");
        sample(3217,-30720,-6144,-20480);
        if(state!==1 || fault) $fatal(1,"negative safety boundary failed during run");
        boot; arm=6144; arm_speed=20480; request(0,1); balance_required;
        boot; arm=-6144; arm_speed=-20480; request(0,1); balance_required;

        // Acknowledge/stop cannot recenter a still-outside coordinate. Returning
        // inside without a new request must remain idle; then new G is allowed.
        boot; arm=-6145; request(1,0); idle_required;
        @(negedge clk); clear=1; start=1; ticks(1); idle_required;
        @(negedge clk); clear=0; start=0;
        if(arm!==-6145) $fatal(1,"clear changed external position");
        request(1,0); idle_required;
        @(negedge clk); stop=1; start_balance=1; sensor_fault=1; calibrated=0;
        ticks(1); idle_required;
        @(negedge clk); stop=0; start_balance=0; sensor_fault=0; calibrated=1;
        arm=-6144; ticks(2); idle_required;
        request(1,0);
        if(state!==1 || !enable) $fatal(1,"returned-inside new request was rejected");
        sample(3217,0,-6145,0); fault_required(8'h08);

        // H has strict capture limits on both signs; rejection never becomes G.
        boot; theta=268; request(0,1); idle_required;
        theta=-268; request(0,1); idle_required;
        theta=0; omega=3584; request(0,1); idle_required;
        omega=-3584; request(0,1); idle_required;
        omega=0; theta=3217; request(1,1); idle_required;
        theta=0; measurement_ready=0; request(1,1); idle_required;

        boot; theta=267; omega=3583; request(0,1); balance_required;
        boot; theta=-267; omega=-3583; request(0,1); balance_required;
        boot; request(1,1); balance_required;

        // Capture the current arm position without any swing kick or stale output.
        boot; arm=2000; request(0,1); balance_required;
        if(command!==0) $fatal(1,"H entry emitted output before a controller sample");
        ticks(8);
        if(state!==2 || command!==0) $fatal(1,"H fell through into kick or swing");
        sample(0,0,2000,0);
        if(command!==0) $fatal(1,"H failed to capture current arm reference");
        sample(18,0,2000,0);
        if(command>=0 || command < -100) $fatal(1,"H first feedback must be bounded LQR, not kick");
        sample(0,0,3000,0);
        if(command!==9) $fatal(1,"H captured arm restoring feedback missing");
        request(0,1); balance_required;
        sample(0,0,3000,0);
        if(command!==9) $fatal(1,"repeated H recaptured running arm origin");

        // Readiness loss gates output immediately and faults without a new sample.
        @(negedge clk); measurement_ready=0; #1;
        if(enable!==0) $fatal(1,"measurement_ready must gate enable combinationally");
        ticks(1); fault_required(8'h01);
        @(negedge clk); measurement_ready=1;
        request(0,1); fault_required(8'h01);

        boot; request(0,1); balance_required;
        @(negedge clk); sensor_fault=1; #1;
        if(enable!==0) $fatal(1,"sensor fault must gate H output immediately");
        ticks(1); fault_required(8'h01);
        boot; request(0,1); @(negedge clk); calibrated=0;
        ticks(1); fault_required(8'h02);
        boot; request(0,1); ticks(66); fault_required(8'h04);
        boot; request(0,1); sample(716,0,0,0); fault_required(8'h20);
        boot; request(0,1); sample(0,0,6145,0); fault_required(8'h08);
        boot; request(0,1); sample(0,0,0,20481); fault_required(8'h08);

        // Stop wins over both entries, clear, and a simultaneous sensor fault.
        boot; request(0,1); sample(18,0,0,0);
        @(negedge clk); start=1; start_balance=1; stop=1; clear=1;
        measurement_ready=0; sensor_fault=1; #1;
        if(enable!==0) $fatal(1,"stop did not immediately disable H");
        ticks(1); idle_required;
        @(negedge clk); start=0; start_balance=0; stop=0; clear=0;
        sensor_fault=0; measurement_ready=1;
        ticks(8); idle_required;

        // Existing G remains automatic: down starts with kick and then captures.
        boot; theta=3217; request(1,0);
        if(state!==1 || enable!==1) $fatal(1,"G no longer enters SWING");
        sample(3217,0,0,0);
        if(state!==1 || command!==667) $fatal(1,"G starting kick changed");
        request(0,1);
        if(state!==1) $fatal(1,"H hijacked running automatic swing");
        sample(267,3583,0,0); balance_required;
        $display("PASS tb_balance_entry: H no-kick/reference/G priority/mature measurement/shared admission limits/signed extrema/R no recenter/stop/protection");
        $finish;
    end
endmodule
