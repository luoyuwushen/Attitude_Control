`timescale 1ns/1ps
// Independent signed-integer oracle; no forced state, gain or capture register.
// Controller-only vectors compress the sample interval, retaining its counter
// semantics. The separate estimator comparison uses real 1ms sample spacing.
module tb_handover_feedback;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0, sample_valid=0, calibrated=1, sensor_fault=0;
    reg start=0, start_balance=0, stop=0, clear=0, measurement_ready=1;
    reg signed [15:0] theta=0, omega=0, arm=0, arm_speed=0;
    wire [2:0] state, positive_state;
    wire [7:0] fault, positive_fault;
    wire signed [15:0] command, positive_command;
    wire enable, positive_enable;
    wire [55:0] handover_detail, positive_detail;
    attitude_controller dut(.*, .trace_h_active(), .trace_capture_arm(),
        .trace_sample_accept(), .trace_command_commit(), .trace_states(), .trace_integral_q24());
    // Opposite-sign H arm coefficient exercises magnitude/sign selection;
    // both profiles still use the production G arm coefficient.
    attitude_controller #(.H_K_ARM(34213)) positive_arm(
        .clk(clk),.rst_n(rst_n),.sample_valid(sample_valid),.calibrated(calibrated),
        .sensor_fault(sensor_fault),.start(start),.start_balance(start_balance),
        .stop(stop),.clear(clear),.measurement_ready(measurement_ready),
        .theta(theta),.omega(omega),.arm(arm),.arm_speed(arm_speed),
        .state(positive_state),.fault(positive_fault),.command(positive_command),.enable(positive_enable),
        .handover_detail(positive_detail));
    integer checks=0, n, j, t, w, a, v;
    bit vectors_done=0, estimator_done=0;
    reg signed [63:0] reference_acc=0, reference_positive_acc=0;

    function automatic signed [63:0] floor_div(input signed [63:0] value,input integer denominator);
        if(value>=0) floor_div=value/denominator;
        else floor_div=-((-value+denominator-1)/denominator);
    endfunction
    function automatic signed [63:0] feedback_effort(input bit h,input bit positive_h_arm,
        input integer angle,input integer speed,input integer position,
        input integer velocity,input integer origin,input integer sample_number);
        reg signed [63:0] kt,kw,ka,kv,p,sum,effort;
        begin
            kt=h ? 904758 : 2362613; kw=h ? 77909 : 299007;
            ka=h ? (positive_h_arm ? 34213 : -34213) : -109597;
            kv=h ? -61768 : -118252;
            p=ka*(position-origin);
            if(sample_number<128) p=floor_div(p,4);
            else if(sample_number<256) p=floor_div(p,2);
            else if(sample_number<384) p=floor_div(p,2)+floor_div(p,4);
            sum=kt*angle+kw*speed+kv*velocity+p;
            effort=-floor_div(sum,1048576);
            feedback_effort=effort;
        end
    endfunction
    function automatic integer round_reference(input signed [63:0] value);
        if(value<0)round_reference=-((-value+8388608)/16777216);
        else round_reference=(value+8388608)/16777216;
    endfunction
    function automatic integer bounded(input signed [63:0] value);
        bounded=value>1000 ? 1000 : value < -1000 ? -1000 : value;
    endfunction
    task ticks(input integer count);
        begin repeat(count)begin @(posedge clk);#1;end end
    endtask
    task boot;
        begin
            @(negedge clk);rst_n=0;sample_valid=0;start=0;start_balance=0;
            stop=0;clear=0;calibrated=1;sensor_fault=0;measurement_ready=1;
            theta=0;omega=0;arm=0;arm_speed=0;ticks(2);
            reference_acc=0;reference_positive_acc=0;
            @(negedge clk);rst_n=1;ticks(1);
        end
    endtask
    task request(input bit g,input bit h);
        begin @(negedge clk);start=g;start_balance=h;ticks(1);
            @(negedge clk);start=0;start_balance=0;end
    endtask
    task stop_here;
        begin
            stop=1;#1;
            if(enable || positive_enable)$fatal(1,"stop gate delayed");
            ticks(1);
            if(state!==0 || command!==0 || dut.handover_profile!==0 || dut.stage!==0)
                $fatal(1,"S did not clear state/command/profile/pipeline");
            if(dut.integral_accumulator!==0 || handover_detail!==0 || positive_detail!==0)
                $fatal(1,"S retained H integral/detail");
            reference_acc=0;reference_positive_acc=0;
            @(negedge clk);stop=0;ticks(8);
            if(state!==0 || command!==0 || positive_command!==0)$fatal(1,"old work restarted after S");
        end
    endtask
    task stop_now;
        begin @(negedge clk);stop_here;end
    endtask
    task checked_sample(input integer angle,input integer speed,input integer position,
        input integer velocity,input bit h,input integer origin,input integer sample_number);
        integer expected,expected_positive;
        reg signed [63:0] total,total_positive,delta,old_acc,old_positive;
        reg allowed,frozen,frozen_positive;
        reg [7:0] flags,flags_positive;
        reg [55:0] expected_detail,expected_positive_detail;
        begin
            old_acc=reference_acc;old_positive=reference_positive_acc;
            total=feedback_effort(h,0,angle,speed,position,velocity,origin,sample_number)+(h ? round_reference(old_acc) : 0);
            total_positive=feedback_effort(h,1,angle,speed,position,velocity,origin,sample_number)+(h ? round_reference(old_positive) : 0);
            expected=bounded(total);expected_positive=bounded(total_positive);
            allowed=h && sample_number>=384 && angle>-268 && angle<268 &&
                    speed>-3584 && speed<3584 && velocity>-10240 && velocity<10240;
            delta=237*(position-origin);
            if(delta>335544)delta=335544;
            if(delta < -335544)delta=-335544;
            frozen=allowed && ((total>=1000 && delta>0)||(total<=-1000 && delta<0));
            frozen_positive=allowed && ((total_positive>=1000 && delta>0)||(total_positive<=-1000 && delta<0));
            flags=h ? 1 : 0;flags_positive=flags;
            flags[1]=allowed&&!frozen;flags[2]=frozen;
            flags[3]=h && (old_acc==1677721600 || old_acc==-1677721600);
            flags_positive[1]=allowed&&!frozen_positive;flags_positive[2]=frozen_positive;
            flags_positive[3]=h && (old_positive==1677721600 || old_positive==-1677721600);
            expected_detail=h ? {flags,16'(sample_number>512 ? 512 : sample_number),16'(origin),16'(floor_div(old_acc,65536))} : 56'd0;
            expected_positive_detail=h ? {flags_positive,16'(sample_number>512 ? 512 : sample_number),16'(origin),16'(floor_div(old_positive,65536))} : 56'd0;
            if(!h)begin reference_acc=0;reference_positive_acc=0;end
            else begin
                if(allowed&&!frozen)reference_acc=old_acc+delta;
                if(allowed&&!frozen_positive)reference_positive_acc=old_positive+delta;
                if(reference_acc>1677721600)reference_acc=1677721600;
                if(reference_acc < -1677721600)reference_acc=-1677721600;
                if(reference_positive_acc>1677721600)reference_positive_acc=1677721600;
                if(reference_positive_acc < -1677721600)reference_positive_acc=-1677721600;
            end
            @(negedge clk);theta=angle;omega=speed;arm=position;arm_speed=velocity;sample_valid=1;
            @(negedge clk);sample_valid=0;
            // A repeated H/G during an in-flight calculation must not recapture
            // the arm or change either gain profile or stiffness counter.
            if(sample_number==17 || sample_number==18)begin
                start=sample_number==18;start_balance=sample_number==17;ticks(2);
                @(negedge clk);start=0;start_balance=0;ticks(4);
            end else ticks(6);
            if(state!==2 || fault!==0 || !enable || positive_state!==2 || positive_fault!==0)
                $fatal(1,"qualified vector changed state/fault");
            if(command!==expected || positive_command!==expected_positive)
                $fatal(1,"numeric profile=%0d n=%0d t=%0d w=%0d a=%0d v=%0d got=%0d/%0d expected=%0d/%0d",
                    h,sample_number,angle,speed,position,velocity,command,positive_command,expected,expected_positive);
            if(handover_detail!==expected_detail || positive_detail!==expected_positive_detail ||
               $signed(dut.integral_accumulator)!==reference_acc || $signed(positive_arm.integral_accumulator)!==reference_positive_acc)
                $fatal(1,"integral oracle/detail mismatch at n=%0d",sample_number);
            if(dut.handover_profile!==h || dut.capture_arm!==origin ||
               dut.catch_ms!==(sample_number>512 ? 512 : sample_number))
                $fatal(1,"profile/capture/counter changed by redundant request");
            checks=checks+1;
        end
    endtask
    initial begin
        boot;arm=700;request(0,1);
        if(state!==2 || command!==0 || !dut.handover_profile)$fatal(1,"H entry");
        for(n=1;n<=512;n=n+1)begin
            t=0;w=0;a=700;v=0;
            case(n%10)
                0:t=31; 1:t=-31; 2:w=123; 3:w=-123;
                4:a=829; 5:a=571; 6:v=333; 7:v=-333;
                8:begin t=37;w=-101;a=911;v=-477;end
                9:begin t=-37;w=101;a=489;v=477;end
            endcase
            if(n==127 || n==128 || n==255 || n==256 || n==383 || n==384)begin
                t=0;w=0;v=0;a=(n%2) ? -303 : 1703;
            end
            checked_sample(t,w,a,v,1,700,n);
            if(n==127 || n==128 || n==255 || n==256 || n==383 || n==384)
                $display("H stiffness boundary n=%0d command=%0d positive_arm=%0d",n,command,positive_command);
        end
        checked_sample(0,20000,700,0,1,700,512);
        if(command!==-1000)$fatal(1,"negative H saturation");
        checked_sample(0,-20000,700,0,1,700,512);
        if(command!==1000)$fatal(1,"positive H saturation");

        // H -> S -> G retains the original kick and captured-balance gains.
        stop_now;theta=3217;omega=0;arm=1000;arm_speed=0;request(1,0);
        if(state!==1 || dut.handover_profile!==0)$fatal(1,"G after H");
        request(0,1);
        if(state!==1 || dut.handover_profile!==0)$fatal(1,"H switched running G profile");
        @(negedge clk);sample_valid=1;@(negedge clk);sample_valid=0;ticks(6);
        if(command!==667)$fatal(1,"G kick changed");
        // First upright sample enters BALANCE and recaptures at 1050.
        checked_sample(60,40,1050,-50,0,1050,0);
        for(n=1;n<=18;n=n+1)checked_sample(-37,101,1251,-477,0,1050,n);
        stop_now;theta=0;omega=0;arm=-400;arm_speed=0;request(0,1);
        checked_sample(37,-101,-611,477,1,-400,1);

        // S must preempt every occupied computation stage; no result may leak.
        for(j=1;j<=5;j=j+1)begin
            stop_now;theta=0;omega=0;arm=500;arm_speed=0;request(0,1);
            @(negedge clk);theta=45;omega=123;sample_valid=1;
            @(negedge clk);sample_valid=0;
            if(j>1)ticks(j-1);
            if(dut.stage!==j)$fatal(1,"interruption stage not reached");
            if(j==1)stop_here;else stop_now;
        end

        // Rejected H and simultaneous H/G cannot silently select a profile.
        theta=268;omega=0;request(0,1);
        if(state!==0 || dut.handover_profile!==0)$fatal(1,"rejected H selected profile");
        request(1,1);
        if(state!==0 || dut.handover_profile!==0)$fatal(1,"rejected H fell back to G");
        theta=0;request(1,1);
        if(state!==2 || !dut.handover_profile)$fatal(1,"accepted H/G priority");
        // R remains ignored while running; a sensor fault still shuts down.
        @(negedge clk);clear=1;ticks(1);@(negedge clk);clear=0;
        if(state!==2 || !dut.handover_profile)$fatal(1,"running R changed profile");
        @(negedge clk);sensor_fault=1;ticks(1);
        if(state!==3 || fault!==1 || command!==0 || enable)$fatal(1,"H sensor protection relaxed");
        @(negedge clk);sensor_fault=0;clear=1;ticks(1);
        @(negedge clk);clear=0;
        if(state!==0 || dut.handover_profile!==0 || command!==0)$fatal(1,"stopped R profile clear");
        $display("PASS handover vectors: %0d oracle checks, H four gains and four ramp stages, signed arm gain, saturation, repeated requests, H/G transitions, every-stage S, R and sensor protection",checks);
        vectors_done=1;
    end

    reg e_rst_n=0,e_valid=0,e_down=0,e_up=0,e_h=0;
    reg [13:0] e_q4=219*16;
    wire e_cal,e_fault,e_state_valid,e_ready,e_measurement_valid;
    wire signed [15:0] e_theta,e_omega,e_arm,e_speed,e_command,e_old_command;
    wire [2:0] e_state,e_old_state;
    wire [7:0] e_controller_fault,e_old_fault;
    state_estimator estimator(.clk(clk),.rst_n(e_rst_n),.sample_valid(e_valid),.sample_q4(e_q4),
        .sample_bad(1'b0),.sample_blind(1'b0),.sample_fault(1'b0),.sample_quality_reason(8'd0),.position(32'sd0),
        .cal_down(e_down),.cal_up(e_up),.stopped(e_state==0),.clear_fault(1'b0),
        .calibrated(e_cal),.sensor_fault(e_fault),.valid(e_state_valid),
        .theta(e_theta),.omega(e_omega),.arm(e_arm),.arm_speed(e_speed),
        .measurement_ready(e_ready),.measurement_valid(e_measurement_valid));
    attitude_controller estimated_h(.clk(clk),.rst_n(e_rst_n),.sample_valid(e_state_valid),
        .calibrated(e_cal),.sensor_fault(e_fault),.measurement_ready(e_ready),
        .start(1'b0),.start_balance(e_h),.stop(1'b0),.clear(1'b0),
        .theta(e_theta),.omega(e_omega),.arm(e_arm),.arm_speed(e_speed),
        .state(e_state),.fault(e_controller_fault),.command(e_command));
    attitude_controller #(.H_K_THETA(2362613),.H_K_OMEGA(299007),
        .H_K_ARM(-109597),.H_K_SPEED(-118252)) estimated_old(
        .clk(clk),.rst_n(e_rst_n),.sample_valid(e_state_valid),
        .calibrated(e_cal),.sensor_fault(e_fault),.measurement_ready(e_ready),
        .start(1'b0),.start_balance(e_h),.stop(1'b0),.clear(1'b0),
        .theta(e_theta),.omega(e_omega),.arm(e_arm),.arm_speed(e_speed),
        .state(e_old_state),.fault(e_old_fault),.command(e_old_command));
    task e_window(input integer value);
        begin
            @(negedge clk);e_q4=value;e_valid=1;
            @(negedge clk);e_valid=0;
            repeat(49998)@(negedge clk);
        end
    endtask
    task prepare_estimator;
        integer k;
        begin
            @(negedge clk);e_rst_n=0;e_valid=0;e_down=0;e_up=0;e_h=0;e_q4=219*16;
            ticks(3);@(negedge clk);e_rst_n=1;e_window(219*16);
            @(negedge clk);e_down=1;@(negedge clk);e_down=0;
            e_window(779*16);
            @(negedge clk);e_up=1;@(negedge clk);e_up=0;
            for(k=0;k<20;k=k+1)e_window(779*16);
            if(!e_cal || !e_ready || e_fault || e_theta || e_omega)$fatal(1,"estimator real calibration");
            @(negedge clk);e_h=1;@(negedge clk);e_h=0;
            if(e_state!==2 || e_old_state!==2)$fatal(1,"estimated H entry");
        end
    endtask
    initial begin
        prepare_estimator;e_window(780*16);
        if(e_theta!==-6 || e_omega!==-750 || e_command!==61 || e_old_command!==228 || e_controller_fault || e_old_fault)
            $fatal(1,"+1code new/old expected61/228 got%0d/%0d",e_command,e_old_command);
        $display("Estimator +1code theta=%0d omega=%0d new_H=%0d old_H=%0d",e_theta,e_omega,e_command,e_old_command);
        prepare_estimator;e_window(784*16);
        if(e_theta!==-29 || e_omega!==-3625 || e_command!==295 || e_old_command!==1000 || e_controller_fault || e_old_fault)
            $fatal(1,"+5code new/old expected295/1000 got%0d/%0d",e_command,e_old_command);
        $display("Estimator +5code theta=%0d omega=%0d new_H=%0d old_H=%0d",e_theta,e_omega,e_command,e_old_command);
        estimator_done=1;
    end
    initial begin
        wait(vectors_done && estimator_done);
        $display("PASS tb_handover_feedback: independent fixed-point H/G profile and real-estimator comparison; no physical stability claim");
        $finish;
    end
    initial begin #100000000;$fatal(1,"tb_handover_feedback timeout");end
endmodule
