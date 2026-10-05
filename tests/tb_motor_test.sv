`timescale 1ns/1ps
module tb_motor_test;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0, request=0, permit=1, stop=0, fault=0;
    reg signed [15:0] requested_command=150;
    reg signed [31:0] position=0;
    wire active;
    wire signed [15:0] command;
    wire [7:0] status;
    wire signed [31:0] delta;
    motor_test #(.RUN_CYCLES(12),.NO_MOTION_CYCLES(7)) dut(
        .clk(clk),.rst_n(rst_n),.request(request),.requested_command(requested_command),
        .permit(permit),.stop(stop),.fault(fault),.position(position),
        .active(active),.command(command),.status(status),.delta(delta));
    // Exercise degenerate cycle parameters without any production-length run.
    reg tiny_request=0;
    reg signed [31:0] tiny_position=0;
    wire tiny_active;
    wire signed [15:0] tiny_command;
    wire [7:0] tiny_status;
    wire signed [31:0] tiny_delta;
    motor_test #(.RUN_CYCLES(0),.NO_MOTION_CYCLES(0),.MAX_TRAVEL(0)) tiny(
        .clk(clk),.rst_n(rst_n),.request(tiny_request),.requested_command(16'sd150),
        .permit(1'b1),.stop(1'b0),.fault(1'b0),.position(tiny_position),
        .active(tiny_active),.command(tiny_command),.status(tiny_status),.delta(tiny_delta));
    wire no_motion_tiny_active;
    wire signed [15:0] no_motion_tiny_command;
    wire [7:0] no_motion_tiny_status;
    wire signed [31:0] no_motion_tiny_delta;
    motor_test #(.RUN_CYCLES(4),.NO_MOTION_CYCLES(0)) no_motion_tiny(
        .clk(clk),.rst_n(rst_n),.request(tiny_request),.requested_command(-16'sd220),
        .permit(1'b1),.stop(1'b0),.fault(1'b0),.position(32'sd0),
        .active(no_motion_tiny_active),.command(no_motion_tiny_command),
        .status(no_motion_tiny_status),.delta(no_motion_tiny_delta));
    // Static default-parameter check; no production-duration simulation.
    motor_test defaults(.clk(clk),.rst_n(rst_n),.request(1'b0),.requested_command(16'sd0),
        .permit(1'b0),.stop(1'b0),.fault(1'b0),.position(32'sd0),
        .active(),.command(),.status(),.delta());
    task ticks(input integer n); begin repeat(n) begin @(posedge clk);#1;end end endtask
    task check(input integer running,input integer code,input integer why);
        begin
            if(active!==running || $signed(command)!==code || status!==why)
                $fatal(1,"motor test expected active/command/status=%0d/%0d/%0d got %0d/%0d/%0d",
                    running,code,why,active,$signed(command),status);
        end
    endtask
    task start(input integer code,input integer pos);
        begin
            @(negedge clk); requested_command=code;position=pos;request=1;
            ticks(1);check(1,code,1);
            if(delta!==0) $fatal(1,"new test did not reset relative origin");
            @(negedge clk);request=0;
        end
    endtask
    task stop_test;
        begin
            @(negedge clk);stop=1;ticks(1);check(0,0,4);
            @(negedge clk);stop=0;
        end
    endtask
    integer i, code;
    initial begin #200000; $fatal(1,"tb_motor_test timeout"); end
    initial begin
        if(defaults.RUN_CYCLES!=25000000 || defaults.NO_MOTION_CYCLES!=15000000 || defaults.MAX_TRAVEL!=128)
            $fatal(1,"production motor-test limits changed");
        ticks(3);@(negedge clk);rst_n=1;ticks(1);check(0,0,0);
        if(delta!==0) $fatal(1,"reset origin");
        @(negedge clk);position=77;permit=0;request=1;ticks(1);check(0,0,7);
        if(delta!==0) $fatal(1,"first rejection invented a test displacement");
        @(negedge clk);request=0;position=-90;ticks(1);
        if(delta!==0) $fatal(1,"unadmitted manual movement appeared as test displacement");
        permit=1;
        // All four fixed amplitudes run for exactly the configured duration.
        // Movement returning to origin still counts as encoder activity.
        for(i=0;i<4;i=i+1) begin
            case(i) 0:code=150;1:code=-150;2:code=220;default:code=-220;endcase
            start(code,1000+i*100);
            position=position+1;ticks(1);
            @(negedge clk);position=position-1;permit=0;
            ticks(10);check(1,code,1);
            ticks(1);check(0,0,2);
            @(negedge clk);position=position+5;ticks(1);
            if(delta!==5) $fatal(1,"coasting delta lost after normal completion");
            permit=1;
        end
        // Any observed change cancels only the no-motion timeout, not max run.
        start(150,-50);ticks(6);check(1,150,1);
        ticks(1);check(0,0,6);
        ticks(5);check(0,0,6);
        start(-220,-50);ticks(6);
        @(negedge clk);position=-49;ticks(1);check(1,-220,1);
        ticks(5);check(0,0,2);
        // Travel uses an inclusive +/-128 raw-count bound, independently of
        // whether the movement direction agrees with the requested polarity.
        start(150,1000);position=1127;ticks(1);check(1,150,1);
        @(negedge clk);position=1128;ticks(1);check(0,0,3);
        start(220,-1000);position=-1127;ticks(1);check(1,220,1);
        @(negedge clk);position=-1128;ticks(1);check(0,0,3);
        start(-150,0);position=50000;ticks(1);check(0,0,3);
        // Signed endpoint crossings look like +/-1 after 32-bit wrapping,
        // but the 33-bit safety difference must terminate either direction.
        start(150,32'sh7fffffff);position=32'sh80000000;ticks(1);check(0,0,3);
        if(delta!==32'sd1) $fatal(1,"wire delta is not the documented low 32 bits");
        start(-150,32'sh80000000);position=32'sh7fffffff;ticks(1);check(0,0,3);
        if(delta!==-32'sd1) $fatal(1,"negative wrap delta");
        // Repeated opposite requests and changing admission permission cannot
        // change the active command, origin, deadline, or restart on expiry.
        start(220,100);position=101;ticks(1);
        @(negedge clk);request=1;requested_command=-220;permit=0;position=102;
        ticks(10);check(1,220,1);
        if(delta!==2) $fatal(1,"busy request changed the origin");
        ticks(1);check(0,0,2);
        @(negedge clk);request=0;permit=1;ticks(2);check(0,0,2);
        // Manual stop wins over fault, travel, timeout and a simultaneous new
        // request; fault wins over remaining completion/admission conditions.
        start(150,0);position=1;ticks(11);
        @(negedge clk);position=128;stop=1;fault=1;request=1;requested_command=-150;
        ticks(1);check(0,0,4);
        @(negedge clk);stop=0;fault=0;request=0;ticks(2);check(0,0,4);
        @(negedge clk);stop=1;fault=1;ticks(2);check(0,0,4);
        @(negedge clk);stop=0;fault=0;
        start(-220,30);
        @(negedge clk);fault=1;request=1;requested_command=220;position=200;
        ticks(1);check(0,0,5);
        @(negedge clk);fault=0;request=0;ticks(2);check(0,0,5);
        // Idle admission failures leave the previous origin intact.
        position=35;permit=0;request=1;ticks(1);check(0,0,7);
        if(delta!==5) $fatal(1,"rejection changed the saved origin");
        @(negedge clk);permit=1;requested_command=100;ticks(1);check(0,0,7);
        @(negedge clk);requested_command=0;ticks(1);check(0,0,7);
        @(negedge clk);requested_command=1000;ticks(1);check(0,0,7);
        @(negedge clk);requested_command=150;stop=1;ticks(1);check(0,0,7);
        @(negedge clk);stop=0;fault=1;ticks(1);check(0,0,7);
        @(negedge clk);fault=0;request=0;ticks(2);check(0,0,7);
        // A successful new test clears prior no-motion bookkeeping.
        start(150,35);ticks(7);check(0,0,6);
        start(220,35);stop_test;ticks(10);check(0,0,4);
        @(negedge clk);tiny_request=1;ticks(1);
        if(!tiny_active || tiny_status!=1) $fatal(1,"tiny test admission");
        @(negedge clk);tiny_request=0;ticks(1);
        if(tiny_active || tiny_command || tiny_status!=2) $fatal(1,"zero parameter underflowed");
        if(no_motion_tiny_active || no_motion_tiny_command || no_motion_tiny_status!=6)
            $fatal(1,"zero no-motion deadline underflowed");
        @(negedge clk);tiny_request=1;ticks(1);
        @(negedge clk);tiny_request=0;tiny_position=1;ticks(1);
        if(tiny_active || tiny_command || tiny_status!=3) $fatal(1,"zero travel limit was not bounded to one");
        // Asynchronous reset always disables the output and clears evidence.
        start(-150,123);@(negedge clk);rst_n=0;#1;
        check(0,0,0);
        if(delta!==0) $fatal(1,"reset left a displacement from a previous test");
        $display("PASS tb_motor_test: fixed levels/exact deadline/travel/wrap/stop+fault priority/no-motion/busy ignore/rejection/coasting/reset/tiny parameters");
        $finish;
    end
endmodule
