`timescale 1ns/1ps
// Fixed-point boundary tests. Public requests/samples drive all controller
// states; only the pure rounding function is unit-tested with direct arguments.
module tb_handover_integral;
    reg clk=0;always #10 clk=~clk;
    reg rst_n=0,sample_valid=0,calibrated=1,sensor_fault=0;
    reg start=0,start_balance=0,stop=0,clear=0,measurement_ready=1;
    reg signed [15:0] theta=0,omega=0,arm=0,arm_speed=0;
    wire [2:0] state;
    wire [7:0] fault;
    wire signed [15:0] command;
    wire enable;
    wire [55:0] handover_detail;
    attitude_controller #(.WATCHDOG_CYCLES(64)) dut(.*, .trace_h_active(), .trace_capture_arm(),
        .trace_sample_accept(), .trace_command_commit(), .trace_states(), .trace_integral_q24());
    reg signed [63:0] accumulator=0;
    integer origin=0,age=0,checks=0,n,j,polarity;

    function automatic signed [63:0] floor_div(input signed [63:0] value,input integer denominator);
        if(value>=0)floor_div=value/denominator;
        else floor_div=-((-value+denominator-1)/denominator);
    endfunction
    function automatic integer rounded(input signed [63:0] value);
        if(value<0)rounded=-((-value+8388608)/16777216);
        else rounded=(value+8388608)/16777216;
    endfunction
    task ticks(input integer count);
        begin repeat(count)begin @(posedge clk);#1;end end
    endtask
    task boot;
        begin
            @(negedge clk);rst_n=0;sample_valid=0;calibrated=1;sensor_fault=0;
            start=0;start_balance=0;stop=0;clear=0;measurement_ready=1;
            theta=0;omega=0;arm=0;arm_speed=0;ticks(2);
            @(negedge clk);rst_n=1;ticks(1);
            accumulator=0;origin=0;age=0;
        end
    endtask
    task request(input bit g,input bit h);
        begin @(negedge clk);start=g;start_balance=h;ticks(1);
            @(negedge clk);start=0;start_balance=0;end
    endtask
    task enter_h(input integer capture);
        begin
            boot;arm=capture;request(0,1);origin=capture;
            if(state!==2 || !enable || command!==0 || handover_detail!==0 || dut.integral_accumulator!==0)
                $fatal(1,"H must begin with zero d and unpublished detail");
        end
    endtask
    task step(input integer angle,input integer velocity,input integer position,input integer speed);
        reg signed [63:0] old,delta,arm_product,request_value,total,next_acc;
        reg permitted,freeze;
        reg [7:0] flags;
        integer expected;
        begin
            old=accumulator;
            if(age<512)age=age+1;
            arm_product=-64'sd34213*(position-origin);
            if(age<128)arm_product=floor_div(arm_product,4);
            else if(age<256)arm_product=floor_div(arm_product,2);
            else if(age<384)arm_product=floor_div(arm_product,2)+floor_div(arm_product,4);
            total=64'sd904758*angle+64'sd77909*velocity+arm_product-64'sd61768*speed;
            request_value=-floor_div(total,1048576)+rounded(old);
            expected=request_value>1000 ? 1000 : request_value < -1000 ? -1000 : request_value;
            delta=64'sd237*(position-origin);
            if(delta>335544)delta=335544;
            if(delta < -335544)delta=-335544;
            permitted=age>=384 && angle>-268 && angle<268 && velocity>-3584 && velocity<3584 && speed>-10240 && speed<10240;
            freeze=permitted && ((request_value>=1000 && delta>0)||(request_value<=-1000 && delta<0));
            flags=1;flags[1]=permitted&&!freeze;flags[2]=freeze;
            flags[3]=(old==1677721600 || old==-1677721600);
            next_acc=old;
            if(permitted&&!freeze)next_acc=old+delta;
            if(next_acc>1677721600)next_acc=1677721600;
            if(next_acc < -1677721600)next_acc=-1677721600;
            @(negedge clk);theta=angle;omega=velocity;arm=position;arm_speed=speed;sample_valid=1;
            @(negedge clk);sample_valid=0;ticks(6);
            if(state!==2 || fault!==0 || !enable || command!==expected)
                $fatal(1,"integral command state=%0d fault=%h got=%0d expected=%0d age=%0d",state,fault,command,expected,age);
            if($signed(dut.integral_accumulator)!==next_acc)
                $fatal(1,"accumulator old=%0d delta=%0d got=%0d expected=%0d",old,delta,dut.integral_accumulator,next_acc);
            if($signed(handover_detail[15:0])!==floor_div(old,65536) ||
               $signed(handover_detail[31:16])!==origin || handover_detail[47:32]!==age ||
               handover_detail[55:48]!==flags)
                $fatal(1,"old-d/detail mismatch age=%0d got=%h flags expected=%h",age,handover_detail,flags);
            accumulator=next_acc;checks=checks+1;
        end
    endtask
    task mature;
        begin
            for(n=1;n<=383;n=n+1)step(0,0,origin+1,0);
            if(accumulator!==0 || handover_detail[49])$fatal(1,"integral updated before full stiffness");
            step(0,0,origin+1,0);
            if(accumulator!==237 || !handover_detail[49] || handover_detail[15:0]!==0)
                $fatal(1,"sample384 must use old zero d and then accumulate one fractional increment");
        end
    endtask
    task zero_required(input integer expected_state,input integer expected_fault);
        begin
            if(state!==expected_state || fault!==expected_fault || command!==0 || enable ||
               dut.integral_accumulator!==0 || handover_detail!==0)
                $fatal(1,"stop/fault did not clear d/detail state=%0d fault=%h d=%0d",state,fault,dut.integral_accumulator);
        end
    endtask
    task sample_fault(input integer angle,input integer velocity,input integer position,input integer speed,input integer expected_fault);
        begin
            @(negedge clk);theta=angle;omega=velocity;arm=position;arm_speed=speed;sample_valid=1;
            @(negedge clk);sample_valid=0;ticks(6);zero_required(3,expected_fault);
        end
    endtask
    task rounding_check(input signed [31:0] value,input integer expected);
        if($signed(dut.round_integral(value))!==expected)
            $fatal(1,"symmetric half-away rounding value=%0d expected=%0d got=%0d",value,expected,dut.round_integral(value));
    endtask
    reg signed [63:0] held;
    reg [55:0] held_detail;
    initial begin
        boot;
        rounding_check(0,0);rounding_check(8388607,0);rounding_check(-8388607,0);
        rounding_check(8388608,1);rounding_check(-8388608,-1);
        rounding_check(8388609,1);rounding_check(-8388609,-1);
        rounding_check(25165824,2);rounding_check(-25165824,-2);
        rounding_check(1677721600,100);rounding_check(-1677721600,-100);
        rounding_check(32'sh80000000,-128);rounding_check(32'sh7fffffff,128);

        // Preserve sub-permille increments and a nonzero capture coordinate.
        enter_h(500);mature;
        for(n=0;n<1000;n=n+1)step(0,0,501,0);
        if(accumulator!==237237 || handover_detail[15:0]===0)$fatal(1,"fractional integration dead zone");
        step(0,0,origin+1415,0);step(0,0,origin+1416,0);
        step(0,0,origin-1415,0);step(0,0,origin-1416,0);
        // All three gates are strict and symmetric. None of these invokes an
        // existing running fault, so a blocked integral must hold, not reset.
        step(267,0,501,0);held=accumulator;step(268,0,501,0);
        if(accumulator!==held)$fatal(1,"positive theta equality gate");
        step(-267,0,501,0);held=accumulator;step(-268,0,501,0);
        if(accumulator!==held)$fatal(1,"negative theta equality gate");
        step(0,3583,501,0);held=accumulator;step(0,3584,501,0);
        if(accumulator!==held)$fatal(1,"positive omega equality gate");
        step(0,-3583,501,0);held=accumulator;step(0,-3584,501,0);
        if(accumulator!==held)$fatal(1,"negative omega equality gate");
        step(0,0,501,10239);held=accumulator;step(0,0,501,10240);
        if(accumulator!==held)$fatal(1,"positive speed equality gate");
        step(0,0,501,-10239);held=accumulator;step(0,0,501,-10240);
        if(accumulator!==held)$fatal(1,"negative speed equality gate");
        held=accumulator;held_detail=handover_detail;
        request(1,0);request(0,1);
        @(negedge clk);clear=1;ticks(1);@(negedge clk);clear=0;
        if($signed(dut.integral_accumulator)!==held || handover_detail!==held_detail || state!==2)
            $fatal(1,"running H/G/R altered accumulated d");

        // Reach both bounds using real samples: roughly 5s of sample-count time,
        // compressed spacing for simulation. No accumulator or stage is forced.
        for(polarity=-1;polarity<=1;polarity=polarity+2)begin
            enter_h(0);mature;
            for(n=0;n<5002;n=n+1)step(0,0,polarity*2000,0);
            if(accumulator!==polarity*64'sd1677721600)$fatal(1,"100 permille bound not reached");
            step(0,0,polarity*2000,0);
            if(!handover_detail[51] || !handover_detail[49])$fatal(1,"at-limit attempted-update flag");
            held=accumulator;
            // Current feedback suddenly saturates; never use last command for
            // this decision. All three gate inputs are just inside their limits.
            step(-polarity*267,-polarity*3583,polarity*2000,polarity*10239);
            if(accumulator!==held || handover_detail[55:48]!==8'h0d)
                $fatal(1,"same-direction saturation did not freeze immediately");
            // Opposite error decreases |d| even while the output is saturated.
            step(-polarity*267,-polarity*3583,-polarity*2000,polarity*10239);
            if(accumulator===held || handover_detail[55:48]!==8'h0b)
                $fatal(1,"opposite integral could not release saturation");
            step(0,0,0,0);
            if(handover_detail[51])$fatal(1,"detail describes next d instead of actually-used old d");
        end

        // Maximum legal opposite coordinates exercise the 17-bit difference
        // and signed shift/add path; H capture does not move absolute limits.
        for(polarity=-1;polarity<=1;polarity=polarity+2)begin
            enter_h(polarity*6144);
            for(n=0;n<384;n=n+1)step(0,0,origin,0);
            step(0,0,-origin,0);
            if(accumulator!==-polarity*64'sd335544)$fatal(1,"large signed error/rate cap");
            sample_fault(0,0,-polarity*6145,0,8);
        end

        // Stop every stage with a live fractional accumulator; neither pending
        // output nor pending integral/detail may reappear after S is released.
        for(j=1;j<=5;j=j+1)begin
            enter_h(0);mature;
            @(negedge clk);arm=2000;sample_valid=1;
            @(negedge clk);sample_valid=0;
            if(j>1)begin ticks(j-1);@(negedge clk);end
            if(dut.stage!==j)$fatal(1,"S interruption missed requested stage");
            stop=1;#1;if(enable)$fatal(1,"S output gate not immediate");
            ticks(1);zero_required(0,0);
            @(negedge clk);stop=0;ticks(8);zero_required(0,0);
        end

        // All running fault classes clear d. Existing protection thresholds and
        // R's stopped-only semantics remain independent of integral readiness.
        for(j=0;j<8;j=j+1)begin
            enter_h(0);mature;
            case(j)
                0:begin @(negedge clk);sensor_fault=1;ticks(1);zero_required(3,1);end
                1:begin @(negedge clk);measurement_ready=0;ticks(1);zero_required(3,1);end
                2:begin @(negedge clk);calibrated=0;ticks(1);zero_required(3,2);end
                3:begin ticks(65);zero_required(3,4);end
                4:sample_fault(0,0,6145,0,8);
                5:sample_fault(0,0,0,20481,8);
                6:sample_fault(0,30721,0,0,8);
                7:sample_fault(716,0,0,0,32);
            endcase
            @(negedge clk);sensor_fault=0;measurement_ready=1;calibrated=1;
            theta=0;omega=0;arm=0;arm_speed=0;clear=1;ticks(1);zero_required(0,0);
            @(negedge clk);clear=0;ticks(8);zero_required(0,0);
        end
        enter_h(0);mature;
        @(negedge clk);rst_n=0;#1;zero_required(0,0);

        // G's kick and subsequent captured BALANCE never receive an H integral
        // or stale metadata, even with repeated H requests while running.
        boot;theta=3217;request(1,0);
        for(n=0;n<500;n=n+1)begin
            @(negedge clk);sample_valid=1;@(negedge clk);sample_valid=0;ticks(6);
            if(state!==1 || dut.integral_accumulator!==0 || handover_detail!==0)
                $fatal(1,"G swing retained H integral/detail");
        end
        theta=0;arm=100;
        @(negedge clk);sample_valid=1;@(negedge clk);sample_valid=0;ticks(6);
        request(0,1);
        for(n=0;n<20;n=n+1)begin
            @(negedge clk);arm=200;sample_valid=1;@(negedge clk);sample_valid=0;ticks(6);
            if(state!==2 || dut.handover_profile!==0 || dut.integral_accumulator!==0 || handover_detail!==0)
                $fatal(1,"G capture switched to H integral");
        end
        $display("PASS tb_handover_integral: %0d independent Q24/old-d/flags checks; signed rounding, fractional growth, strict gates, rate/accumulator bounds, freeze/release, every-stage stop, fault/reset/R clearing, G isolation",checks);
        $finish;
    end
    initial begin #50000000;$fatal(1,"tb_handover_integral timeout");end
endmodule
