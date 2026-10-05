`timescale 1ns/1ps
`default_nettype none
// Production estimator + controller form the feedback loop. No trace is fed in.
// 64 RTL clock cycles represent one physical 1ms observation window; watchdog
// and freshness are scaled by the same factor. The 50MHz pipeline executes in
// full, followed by a numerical averaged-command plant step. Motor PWM/pads,
// inductance, ADC raw5MHz conversion and switching deadtime are NOT simulated.
// The assumed30us aggregate command latency is explicit in the physical model.
module tb_handover_closed_loop;
    localparam real PI = 3.14159265358979323846;
    localparam integer CYCLES_PER_MS = 64;
    localparam real ARM_LENGTH=0.152, ARM_MASS=0.090, PEND_LENGTH=0.150, PEND_MASS=0.090;
    localparam real JP=PEND_MASS*PEND_LENGTH*PEND_LENGTH/3.0;
    localparam real J0=ARM_MASS*ARM_LENGTH*ARM_LENGTH/3.0+PEND_MASS*ARM_LENGTH*ARM_LENGTH;
    localparam real COUPLING=PEND_MASS*ARM_LENGTH*PEND_LENGTH/2.0;
    localparam real GRAVITY=PEND_MASS*9.81*PEND_LENGTH/2.0;
    localparam real STALL=6.6*0.0980665, MOTOR_K=STALL/12.0;
    localparam real MOTOR_B=STALL/(549.0*2.0*PI/60.0);
    localparam integer ADC_DOWN=220, ADC_UP=764;
    localparam real ADC_SPAN=544.0;

    reg clk=0;
    always #10 clk=~clk;
    reg rst_n=0, sample_valid=0, cal_down=0, cal_up=0;
    reg start_balance=0, stop=0;
    reg [13:0] sample_q4=0;
    reg signed [31:0] position=0;
    wire calibrated,sensor_fault,estimator_valid,measurement_ready,measurement_valid;
    wire signed [15:0] theta,omega,arm,arm_speed,command;
    wire [2:0] state;
    wire [7:0] fault;
    wire enable;
    wire [55:0] handover_detail;
    wire stopped=state==0 || state==3;
    state_estimator #(.THETA_SIGN(1),.COUNTS_PER_REV(1040),
                      .SAMPLE_FRESH_CYCLES(2*CYCLES_PER_MS)) estimator (
        .clk(clk),.rst_n(rst_n),.sample_valid(sample_valid),.sample_q4(sample_q4),
        .sample_bad(1'b0),.sample_blind(1'b0),.sample_fault(1'b0),.sample_quality_reason(8'd0),.position(position),
        .cal_down(cal_down),.cal_up(cal_up),.stopped(stopped),.clear_fault(1'b0),
        .calibrated(calibrated),.sensor_fault(sensor_fault),.valid(estimator_valid),
        .measurement_ready(measurement_ready),.measurement_valid(measurement_valid),
        .theta(theta),.omega(omega),.arm(arm),.arm_speed(arm_speed));
    attitude_controller #(.WATCHDOG_CYCLES(2*CYCLES_PER_MS)) controller (
        .clk(clk),.rst_n(rst_n),.sample_valid(estimator_valid),.calibrated(calibrated),
        .sensor_fault(sensor_fault),.measurement_ready(measurement_ready),
        .start(1'b0),.start_balance(start_balance),.stop(stop),.clear(1'b0),
        .theta(theta),.omega(omega),.arm(arm),.arm_speed(arm_speed),
        .state(state),.fault(fault),.command(command),.enable(enable),
        .handover_detail(handover_detail));

    real x_theta=0.0,x_omega=0.0,x_arm=0.0,x_speed=0.0;
    real bias_deg=0.0,initial_deg=5.0;
    real k1[0:3],k2[0:3],k3[0:3],k4[0:3];
    real mean_integral,before_theta,mean_theta,step,voltage;
    real tail_theta_sq=0.0,tail_arm_sum=0.0,tail_arm_min=1e30,tail_arm_max=-1e30;
    real max_arm=0.0,max_theta=0.0,tail_pwm_sq=0.0,zero_seconds=0.0;
    real theta_rms,arm_mean,arm_p2p,pwm_rms;
    integer duration_ms=30000,ms,segment,old_command,new_command,ignored;
    integer estimator_updates=0,tail_count=0,saturated_samples=0;
    integer trace_fd=0;
    reg [2047:0] trace_path;

    always @(posedge clk) if (rst_n && estimator_valid && state==2)
        estimator_updates=estimator_updates+1;

    function real magnitude(input real x);
        magnitude=x<0 ? -x : x;
    endfunction
    function integer nearest(input real x);
        nearest=x>=0 ? $rtoi(x+0.5) : -$rtoi(-x+0.5);
    endfunction
    function integer adc_q4(input real actual_mean_theta);
        adc_q4=nearest((ADC_UP-(actual_mean_theta+bias_deg*PI/180.0)*ADC_SPAN/PI)*16.0);
    endfunction
    task tick(input integer count);
        begin repeat(count) begin @(posedge clk); #1; end end
    endtask
    task publish_held_sample;
        begin
            @(negedge clk); sample_valid=1;
            tick(1);
            @(negedge clk); sample_valid=0;
            tick(CYCLES_PER_MS-1);
        end
    endtask
    task derivatives(input real th,om,ar,sp,volts,
                     output real dth,dom,dar,dsp);
        real sn,cs,a,b,d,ra,rp,det,damping;
        begin
            sn=$sin(th);cs=$cos(th);a=J0+JP*sn*sn;b=-COUPLING*cs;d=JP;
            // Actual applied averaged zero is high impedance: no electrical
            // motor damping. Mechanical damping remains; motion is not forced0.
            damping=volts==0.0 ? 0.001 : 0.001+MOTOR_B;
            ra=MOTOR_K*volts-damping*sp-2.0*JP*sn*cs*sp*om-COUPLING*sn*om*om;
            rp=GRAVITY*sn-0.0001*om+JP*sn*cs*sp*sp;
            det=a*d-b*b;dth=om;dar=sp;
            dom=(a*rp-b*ra)/det;dsp=(d*ra-b*rp)/det;
        end
    endtask
    task integrate(input real h,volts);
        begin
            derivatives(x_theta,x_omega,x_arm,x_speed,volts,k1[0],k1[1],k1[2],k1[3]);
            derivatives(x_theta+h*k1[0]/2,x_omega+h*k1[1]/2,
                        x_arm+h*k1[2]/2,x_speed+h*k1[3]/2,volts,k2[0],k2[1],k2[2],k2[3]);
            derivatives(x_theta+h*k2[0]/2,x_omega+h*k2[1]/2,
                        x_arm+h*k2[2]/2,x_speed+h*k2[3]/2,volts,k3[0],k3[1],k3[2],k3[3]);
            derivatives(x_theta+h*k3[0],x_omega+h*k3[1],
                        x_arm+h*k3[2],x_speed+h*k3[3],volts,k4[0],k4[1],k4[2],k4[3]);
            x_theta=x_theta+h*(k1[0]+2*k2[0]+2*k3[0]+k4[0])/6;
            x_omega=x_omega+h*(k1[1]+2*k2[1]+2*k3[1]+k4[1])/6;
            x_arm=x_arm+h*(k1[2]+2*k2[2]+2*k3[2]+k4[2])/6;
            x_speed=x_speed+h*(k1[3]+2*k2[3]+2*k3[3]+k4[3])/6;
        end
    endtask

    initial begin
        #50000000;
        $fatal(1,"H RTL closed-loop simulated-time timeout");
    end
    initial begin
        ignored=$value$plusargs("BIAS_DEG=%f",bias_deg);
        ignored=$value$plusargs("THETA_DEG=%f",initial_deg);
        ignored=$value$plusargs("DURATION_MS=%d",duration_ms);
        if (duration_ms<5000 || duration_ms>30000) $fatal(1,"duration must be5000.0.30000ms");
        if ($value$plusargs("TRACE=%s",trace_path)) begin
            trace_fd=$fopen(trace_path,"w");
            if (!trace_fd) $fatal(1,"cannot open trace output");
            $fdisplay(trace_fd,"time_s,true_theta_rad,true_omega,arm_rad,arm_speed,theta_q10,omega_q10,arm_q10,speed_q10,command_permille,integral_q8,flags");
        end
        x_theta=initial_deg*PI/180.0;
        tick(5);
        @(negedge clk);rst_n=1;
        tick(3);
        @(negedge clk);sample_q4=ADC_DOWN*16;sample_valid=1;
        @(negedge clk);sample_valid=0;cal_down=1;
        @(negedge clk);cal_down=0;
        tick(3);
        @(negedge clk);sample_q4=ADC_UP*16;sample_valid=1;
        @(negedge clk);sample_valid=0;cal_up=1;
        @(negedge clk);cal_up=0;
        tick(40);
        if (!calibrated || sensor_fault) $fatal(1,"H model calibration failed");
        // Hold the real plant beforeH, allowing actual RTL IIR to settle. Bias
        // represents hypothetical wrong angle calibration, not field inference.
        sample_q4=adc_q4(x_theta);position=0;
        repeat(100) publish_held_sample;
        if (!measurement_ready || !measurement_valid || sensor_fault)
            $fatal(1,"H model measurement maturity missing");
        @(negedge clk);start_balance=1;
        @(negedge clk);start_balance=0;
        tick(2);
        if (state!=2 || !enable) $fatal(1,"H request rejected theta=%0d omega=%0d",theta,omega);
        $display("H_RTL_LOOP bias_deg=%f initial_deg=%f duration_ms=%0d D220U764 cpr1040 coast=yes average_command_not_PWM cycles_per_ms=64",
                 bias_deg,initial_deg,duration_ms);
        for (ms=0;ms<duration_ms;ms=ms+1) begin
            @(negedge clk);sample_valid=1;
            old_command=$signed(command);
            tick(1);
            @(negedge clk);sample_valid=0;
            tick(19); // estimator and controller pipelines complete normally.
            if (state!=2 || sensor_fault || fault!=0 || !enable || !measurement_valid)
                $fatal(1,"H_RTL_LOOP protected ms=%0d sensor=%b fault=%h true_theta=%f arm=%f measured=[%0d,%0d,%0d,%0d]",
                       ms,sensor_fault,fault,x_theta,x_arm,theta,omega,arm,arm_speed);
            if (!handover_detail[48]) $fatal(1,"H detail/profile absent");
            new_command=$signed(command);
            if (new_command>=1000 || new_command<=-1000) saturated_samples=saturated_samples+1;
            mean_integral=0.0;
            for (segment=0;segment<17;segment=segment+1) begin
                step=segment==0 ? 0.000030 : 0.000970/16.0;
                voltage=0.012*(segment==0 ? old_command : new_command);
                if (voltage==0.0) zero_seconds=zero_seconds+step;
                before_theta=x_theta;
                integrate(step,voltage);
                mean_integral=mean_integral+step*(before_theta+x_theta)/2.0;
            end
            mean_theta=mean_integral/0.001;
            // Numerical quadrature approximates the full1ms continuous mean;
            // it is not a16-sparse-code ADC model or a precomputed state trace.
            sample_q4=adc_q4(mean_theta);
            position=nearest(x_arm*1040.0/(2.0*PI));
            if (magnitude(x_arm)>max_arm) max_arm=magnitude(x_arm);
            if (magnitude(x_theta)>max_theta) max_theta=magnitude(x_theta);
            if (ms>=duration_ms-5000) begin
                tail_count=tail_count+1;tail_theta_sq=tail_theta_sq+x_theta*x_theta;
                tail_arm_sum=tail_arm_sum+x_arm;tail_pwm_sq=tail_pwm_sq+new_command*new_command;
                if (x_arm<tail_arm_min) tail_arm_min=x_arm;
                if (x_arm>tail_arm_max) tail_arm_max=x_arm;
            end
            if (trace_fd && ms%20==19)
                $fdisplay(trace_fd,"%f,%f,%f,%f,%f,%0d,%0d,%0d,%0d,%0d,%0d,%0d",
                    (ms+1)/1000.0,x_theta,x_omega,x_arm,x_speed,theta,omega,arm,arm_speed,
                    command,$signed(handover_detail[15:0]),handover_detail[55:48]);
            if (ms%5000==4999)
                $display("H_RTL_LOOP t=%f theta_deg=%f arm_rad=%f u=%0d integral_q8=%0d",
                         (ms+1)/1000.0,x_theta*180.0/PI,x_arm,command,$signed(handover_detail[15:0]));
            tick(CYCLES_PER_MS-20);
        end
        theta_rms=$sqrt(tail_theta_sq/tail_count)*180.0/PI;
        arm_mean=tail_arm_sum/tail_count;arm_p2p=tail_arm_max-tail_arm_min;
        pwm_rms=$sqrt(tail_pwm_sq/tail_count);
        $display("H_RTL_LOOP metrics bias=%f initial=%f samples=%0d theta_rms_deg=%f arm_mean_rad=%f arm_p2p_rad=%f max_arm_rad=%f max_theta_deg=%f pwm_rms_permille=%f saturation_samples=%0d coast_fraction=%f",
                 bias_deg,initial_deg,estimator_updates,theta_rms,arm_mean,arm_p2p,max_arm,max_theta*180.0/PI,
                 pwm_rms,saturated_samples,zero_seconds/(duration_ms/1000.0));
        if (estimator_updates!=duration_ms) $fatal(1,"feedback sample count mismatch");
        if (theta_rms>2.0 || magnitude(arm_mean)>0.25 || arm_p2p>0.25 || max_arm>=6.0)
            $fatal(1,"nominal H model centering/balance acceptance failed");
        @(negedge clk);stop=1;
        tick(2);
        if (enable || command!=0 || handover_detail!=0) $fatal(1,"H stop did not clear command/integral");
        if (trace_fd) $fclose(trace_fd);
        $display("PASS tb_handover_closed_loop: realRTL estimator+H LQI feedback nonlinear averaged-command plant, coast, biased meanADC, 30s nominal only");
        $finish;
    end
endmodule
`default_nettype wire
