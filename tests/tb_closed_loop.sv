`timescale 1ns/1ps
`default_nettype none
// Real RTL feedback drives a nonlinear Furuta model. The clock is accelerated:
// 1000 RTL cycles represent 1 ms; 50-cycle PWM still represents 20 kHz.
// Mechanical parameters and the 210-degree sensor installation are hypotheses,
// matching tools/control_model.py. This test is not physical-board acceptance.
module tb_closed_loop;
    localparam real PI = 3.14159265358979323846;
    localparam real ARM_LENGTH = 0.152, ARM_MASS = 0.090;
    localparam real PEND_LENGTH = 0.150, PEND_MASS = 0.090;
    localparam real JP = PEND_MASS*PEND_LENGTH*PEND_LENGTH/3.0;
    localparam real J0 = ARM_MASS*ARM_LENGTH*ARM_LENGTH/3.0 +
                        PEND_MASS*ARM_LENGTH*ARM_LENGTH;
    localparam real COUPLING = PEND_MASS*ARM_LENGTH*PEND_LENGTH/2.0;
    localparam real GRAVITY = PEND_MASS*9.81*PEND_LENGTH/2.0;
    localparam real STALL_TORQUE = 6.6*0.0980665;
    localparam real MOTOR_K = STALL_TORQUE/12.0;
    localparam real MOTOR_B = STALL_TORQUE/(549.0*2.0*PI/60.0);
    localparam real ADC_OFFSET = 1023.0/2.0;
    localparam real ADC_SPAN = 1023.0*3.3/5.0/2.0;
    localparam real ELECTRICAL_SPAN = 345.0*PI/180.0;
    localparam real SENSOR_UP_PHASE = 30.0*PI/180.0;
    localparam real STEP = 0.001/16.0;
    localparam integer DURATION_MS = 12000;

    reg clk = 0;
    always #5 clk = ~clk;
    reg rst_n = 0, sample_valid = 0, cal_down = 0, cal_up = 0;
    reg start = 0, stop = 0;
    reg [13:0] sample_q4 = 0;
    reg signed [31:0] position = 0;
    wire calibrated, sensor_fault, estimator_valid;
    wire signed [15:0] theta, omega, arm, arm_speed, command;
    wire [2:0] state;
    wire [7:0] fault;
    wire enable, in1, in2, pwm;
    wire stopped = state == 0 || state == 3;

    state_estimator #(.THETA_SIGN(-1), .COUNTS_PER_REV(1040)) estimator (
        .clk(clk), .rst_n(rst_n), .sample_valid(sample_valid), .sample_q4(sample_q4),
        .position(position), .cal_down(cal_down), .cal_up(cal_up), .stopped(stopped),
        .calibrated(calibrated), .sensor_fault(sensor_fault), .valid(estimator_valid),
        .theta(theta), .omega(omega), .arm(arm), .arm_speed(arm_speed));
    attitude_controller controller (
        .clk(clk), .rst_n(rst_n), .sample_valid(estimator_valid), .calibrated(calibrated),
        .sensor_fault(sensor_fault), .start(start), .stop(stop), .clear(1'b0),
        .theta(theta), .omega(omega), .arm(arm), .arm_speed(arm_speed),
        .state(state), .fault(fault), .command(command), .enable(enable));
    motor_pwm #(.PERIOD_CYCLES(50), .DEAD_CYCLES(2)) motor (
        .clk(clk), .rst_n(rst_n), .enable(enable), .command(command),
        .in1(in1), .in2(in2), .pwm(pwm));

    real x_theta = PI, x_omega = 0.0, x_arm = 0.0, x_speed = 0.0;
    real k1[0:3], k2[0:3], k3[0:3], k4[0:3];
    real capture_seconds = -1.0, final_max_degrees = 0.0;
    real maximum_arm_radians = 0.0, disturbance_max_degrees = 0.0;
    real recovery_seconds = -1.0, error_degrees, voltage, push;
    integer ms, segment, cycle, segment_cycles, pwm_sum, adc_sum;
    integer captures = 0, previous_state = 0, stable_after_push_ms = 0;
    integer code_down, code_up;

    function real wrap(input real angle);
        wrap = angle - 2.0*PI*$floor((angle+PI)/(2.0*PI));
    endfunction
    function real magnitude(input real value);
        magnitude = value < 0.0 ? -value : value;
    endfunction
    function integer nearest_integer(input real value);
        nearest_integer = value >= 0.0 ? $rtoi(value+0.5) : -$rtoi(-value+0.5);
    endfunction
    function real sensor_phase(input real angle);
        real phase;
        begin
            phase = SENSOR_UP_PHASE-angle;
            sensor_phase = phase-2.0*PI*$floor(phase/(2.0*PI));
        end
    endfunction
    function integer sensor_code(input real phase);
        sensor_code = nearest_integer(ADC_OFFSET+phase/ELECTRICAL_SPAN*ADC_SPAN);
    endfunction

    // Euler-Lagrange equations. Positive voltage accelerates the arm and upright
    // pendulum in the positive coordinate; no synthetic state feedback is used.
    task derivatives(input real th, om, ar, sp, volts, external_torque,
                     output real dth, dom, dar, dsp);
        real sn, cs, a, b, d, rhs_arm, rhs_pend, determinant;
        begin
            sn = $sin(th); cs = $cos(th);
            a = J0+JP*sn*sn; b = -COUPLING*cs; d = JP;
            rhs_arm = MOTOR_K*volts-(0.001+MOTOR_B)*sp -
                      2.0*JP*sn*cs*sp*om-COUPLING*sn*om*om;
            rhs_pend = GRAVITY*sn-0.0001*om+JP*sn*cs*sp*sp+external_torque;
            determinant = a*d-b*b;
            dth = om; dar = sp;
            dom = (a*rhs_pend-b*rhs_arm)/determinant;
            dsp = (d*rhs_arm-b*rhs_pend)/determinant;
        end
    endtask
    task integrate(input real volts, external_torque);
        begin
            derivatives(x_theta,x_omega,x_arm,x_speed,volts,external_torque,
                        k1[0],k1[1],k1[2],k1[3]);
            derivatives(x_theta+STEP*k1[0]/2.0,x_omega+STEP*k1[1]/2.0,
                        x_arm+STEP*k1[2]/2.0,x_speed+STEP*k1[3]/2.0,volts,external_torque,
                        k2[0],k2[1],k2[2],k2[3]);
            derivatives(x_theta+STEP*k2[0]/2.0,x_omega+STEP*k2[1]/2.0,
                        x_arm+STEP*k2[2]/2.0,x_speed+STEP*k2[3]/2.0,volts,external_torque,
                        k3[0],k3[1],k3[2],k3[3]);
            derivatives(x_theta+STEP*k3[0],x_omega+STEP*k3[1],
                        x_arm+STEP*k3[2],x_speed+STEP*k3[3],volts,external_torque,
                        k4[0],k4[1],k4[2],k4[3]);
            x_theta = x_theta+STEP*(k1[0]+2.0*k2[0]+2.0*k3[0]+k4[0])/6.0;
            x_omega = x_omega+STEP*(k1[1]+2.0*k2[1]+2.0*k3[1]+k4[1])/6.0;
            x_arm = x_arm+STEP*(k1[2]+2.0*k2[2]+2.0*k3[2]+k4[2])/6.0;
            x_speed = x_speed+STEP*(k1[3]+2.0*k2[3]+2.0*k3[3]+k4[3])/6.0;
        end
    endtask
    task tick(input integer count);
        begin repeat(count) begin @(posedge clk); #1; end end
    endtask

    initial begin
        #125000000;
        $fatal(1,"tb_closed_loop simulated-time timeout");
    end
    initial begin
        // Separate stationary down/up captures calibrate the actual quantized
        // transfer, then return to rest at pi before starting automatically.
        code_down = sensor_code(210.0*PI/180.0);
        code_up = sensor_code(SENSOR_UP_PHASE);
        tick(5);
        @(negedge clk); rst_n = 1;
        tick(3);
        @(negedge clk); sample_q4 = code_down*16; cal_down = 1;
        @(negedge clk); cal_down = 0;
        tick(3);
        @(negedge clk); sample_q4 = code_up*16; cal_up = 1;
        @(negedge clk); cal_up = 0;
        tick(40);
        if (!calibrated || sensor_fault) $fatal(1,"closed-loop calibration failed");
        @(negedge clk); sample_q4 = code_down*16; sample_valid = 1;
        @(negedge clk); sample_valid = 0;
        tick(15);
        if (magnitude($itor($signed(theta))/1024.0) < 3.0 || omega != 0)
            $fatal(1,"model must start at stationary hanging-down position");
        @(negedge clk); start = 1;
        @(negedge clk); start = 0;
        $display("CLOSED_LOOP model=Furuta initial=pi down_adc=%0d up_adc=%0d sensor_sign=-1",
                 code_down,code_up);
        for (ms=0; ms<DURATION_MS; ms=ms+1) begin
            @(negedge clk); sample_valid = 1;
            adc_sum = 0;
            push = ms >= 2000 && ms < 2050 ? 0.01 : 0.0;
            // Each physical ADC sample is taken after a nonlinear RK4 step.
            // Alternating 62/63 clock intervals gives exactly 1000 cycles/ms.
            for (segment=0; segment<16; segment=segment+1) begin
                segment_cycles = segment%2 == 0 ? 62 : 63;
                pwm_sum = 0;
                for (cycle=0; cycle<segment_cycles; cycle=cycle+1) begin
                    tick(1);
                    if (segment==0 && cycle==0) sample_valid = 0;
                    if (sensor_fault || fault != 0 || state == 3)
                        $fatal(1,"closed-loop protected stop at %0d ms sensor=%b fault=%h true_theta=%f arm=%f estimated=[%0d,%0d,%0d,%0d]",
                               ms,sensor_fault,fault,wrap(x_theta),x_arm,theta,omega,arm,arm_speed);
                    if (in1 && in2) $fatal(1,"invalid motor direction outputs");
                    if (pwm && in1 && !in2) pwm_sum = pwm_sum+1;
                    else if (pwm && in2 && !in1) pwm_sum = pwm_sum-1;
                    if (state==2 && previous_state!=2) begin
                        captures = captures+1;
                        if (capture_seconds < 0.0) capture_seconds = ms/1000.0;
                        $display("CLOSED_LOOP capture time=%f s theta=%f deg arm=%f rad",
                                 ms/1000.0,wrap(x_theta)*180.0/PI,x_arm);
                    end
                    previous_state = state;
                end
                voltage = 12.0*pwm_sum/segment_cycles;
                integrate(voltage,push);
                if (sensor_phase(x_theta)>ELECTRICAL_SPAN)
                    $fatal(1,"actual sensor blind sector entered at %0d ms phase=%f deg theta=%f deg",
                           ms,sensor_phase(x_theta)*180.0/PI,wrap(x_theta)*180.0/PI);
                adc_sum = adc_sum+sensor_code(sensor_phase(x_theta));
                if (magnitude(x_arm)>6.0)
                    $fatal(1,"physical arm soft limit exceeded at %0d ms: %f rad",ms,x_arm);
            end
            sample_q4 = adc_sum;
            position = nearest_integer(x_arm*1040.0/(2.0*PI));
            error_degrees = magnitude(wrap(x_theta))*180.0/PI;
            if (magnitude(x_arm)>maximum_arm_radians) maximum_arm_radians = magnitude(x_arm);
            if (ms>=10000 && error_degrees>final_max_degrees) final_max_degrees=error_degrees;
            if (ms>=2000 && ms<4000 && error_degrees>disturbance_max_degrees)
                disturbance_max_degrees=error_degrees;
            if (ms>=2050 && error_degrees<=2.0 && state==2)
                stable_after_push_ms=stable_after_push_ms+1;
            else stable_after_push_ms=0;
            if (stable_after_push_ms==500 && recovery_seconds<0.0)
                recovery_seconds=(ms-499-2050)/1000.0;
            if (ms%1000==999)
                $display("CLOSED_LOOP t=%f s theta=%f deg omega=%f arm=%f state=%0d command=%0d",
                         (ms+1)/1000.0,wrap(x_theta)*180.0/PI,x_omega,x_arm,state,command);
        end
        $display("CLOSED_LOOP metrics capture_s=%f captures=%0d final_deg=%f last_2s_max_deg=%f disturbance_peak_deg=%f recovery_s=%f max_arm_rad=%f",
                 capture_seconds,captures,wrap(x_theta)*180.0/PI,final_max_degrees,
                 disturbance_max_degrees,recovery_seconds,maximum_arm_radians);
        if (captures!=1 || state!=2) $fatal(1,"automatic swing-up/single sustained capture failed");
        if (final_max_degrees>2.0) $fatal(1,"last two seconds exceed +/-2 degrees");
        if (recovery_seconds<0.0 || recovery_seconds>1.0)
            $fatal(1,"light push did not recover into +/-2 degrees within 1 s for 500 ms");
        if (sensor_fault || fault!=0) $fatal(1,"protection must remain active and untriggered");
        @(negedge clk); stop=1;
        tick(2);
        if (enable || in1 || in2 || pwm!==1'b1) $fatal(1,"closed-loop stop did not coast");
        $display("PASS tb_closed_loop: nonlinear plant/physical ADC quantization/RTL feedback/PWM/automatic swing-up/balance/push recovery");
        $finish;
    end
endmodule
`default_nettype wire
