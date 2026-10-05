`timescale 1ns/1ps
module tb_control_trace_probe;
    reg clk=0;always #10 clk=~clk;
    reg rst_n=0,sv=0,adc_valid=0,start_h=0,stop=0,bad=0;
    reg signed [15:0] theta=0,omega=0,arm=0,speed=0;
    reg [13:0] mean=12349;
    reg signed [31:0] encoder=0;
    reg [31:0] counter=0,ms=0;
    reg [7:0] quality=0;
    wire [2:0] state;wire [7:0] fault;wire signed [15:0] command;
    wire enable,active,accepted,commit;wire [63:0] states;
    wire signed [15:0] capture;
    wire signed [31:0] integral;
    wire [55:0] detail;
    reg signed [15:0] mapped=0;
    wire signed [15:0] gated=enable ? mapped : 16'sd0;
    reg rd=0;reg [3:0] index=0;
    wire rv,capturing,frozen,overwritten;wire [255:0] data;
    wire [15:0] id;wire [3:0] count;wire [31:0] total,freeze_ms;
    wire [7:0] reason;wire [13:0] down_saved,up_saved;wire signed [15:0] cap_saved;
    integer checks=0,i,k;
    reg [255:0] expected[0:439];
    reg [15:0] expected_theta,expected_mean;
    reg signed [15:0] last_submitted;
    reg signed [31:0] old_integral;
    attitude_controller dut(.clk(clk),.rst_n(rst_n),.sample_valid(sv),.calibrated(1'b1),
        .sensor_fault(bad),.start(1'b0),.start_balance(start_h),.stop(stop),.clear(1'b0),
        .measurement_ready(1'b1),.theta(theta),.omega(omega),.arm(arm),.arm_speed(speed),
        .state(state),.fault(fault),.command(command),.enable(enable),.handover_detail(detail),
        .trace_h_active(active),.trace_capture_arm(capture),.trace_sample_accept(accepted),
        .trace_command_commit(commit),.trace_states(states),.trace_integral_q24(integral));
    control_trace_probe #(.ADDRESS_BITS(3)) probe(.clk(clk),.rst_n(rst_n),
        .adc_valid(adc_valid),.adc_mean_q4(mean),.encoder_count(encoder),.sample_counter(counter),
        .device_time_ms(ms),.adc_quality(quality),.sensor_flags(8'h0c),.fault(fault),
        .down_q4(14'd3558),.up_q4(14'd12349),.h_active(active),.sample_accept(accepted),
        .command_commit(commit),.exit_fault(state==3),.control_states(states),.integral_q24(integral),
        .capture_arm(capture),.command(command),.gated_motor_command(gated),.handover_detail(detail),
        .read_request(rd),.read_index(index),.read_valid(rv),.read_data(data),
        .capturing(capturing),.frozen(frozen),.capture_id(id),.row_count(count),
        .total_committed(total),.overwritten(overwritten),.freeze_time_ms(freeze_ms),
        .freeze_reason(reason),.saved_down_q4(down_saved),.saved_up_q4(up_saved),
        .saved_capture_arm_q10(cap_saved));
    always @(posedge clk) begin
        if(adc_valid) counter<=counter+1;
        mapped<=-command;
        old_integral=dut.integral_accumulator;
        #1;
        if(rst_n&&commit) begin
            check(integral===old_integral,"trace uses old Q24 integral");
            check(detail[15:0]===integral[31:16],"trace integral and submitted H detail differ");
        end
    end
`ifdef TRACE_BASELINE
    wire [2:0] base_state;wire [7:0] base_fault;wire signed [15:0] base_command;
    wire base_enable;wire [55:0] base_detail;
    attitude_controller_baseline baseline(.clk(clk),.rst_n(rst_n),.sample_valid(sv),.calibrated(1'b1),
        .sensor_fault(bad),.start(1'b0),.start_balance(start_h),.stop(stop),.clear(1'b0),
        .measurement_ready(1'b1),.theta(theta),.omega(omega),.arm(arm),.arm_speed(speed),
        .state(base_state),.fault(base_fault),.command(base_command),.enable(base_enable),.handover_detail(base_detail));
    always @(posedge clk) begin
        #2;if(rst_n) check({state,fault,command,enable,detail}==={base_state,base_fault,base_command,base_enable,base_detail},
                          "observer changed production output versus frozen v0.2.8");
    end
`endif
    task check;
        input condition;input [1023:0] message;
        begin checks=checks+1;if(condition!==1'b1)$fatal(1,"%0s",message);end
    endtask
    task tick;begin @(posedge clk);#3;end endtask
    task begin_h;
        begin
            @(negedge clk);stop=1;sv=0;bad=0;theta=0;omega=0;arm=0;speed=0;tick();
            @(negedge clk);stop=0;start_h=1;tick();
            @(negedge clk);start_h=0;repeat(3)tick();
            check(active&&capturing&&!frozen&&count==0,"new H starts a clean capture");
        end
    endtask
    task sample_input;
        input integer n;
        begin
            @(negedge clk);adc_valid=1;mean=12300+n%20;encoder=1000+n;quality=n%2?8'h20:8'h00;ms=ms+1;
            tick();@(negedge clk);adc_valid=0;
            // Inputs can change outside the accept edge; they are not the row.
            mean=77;encoder=-777;quality=8'hff;
            repeat(6)tick();
            @(negedge clk);theta=10+n%3;omega=-20;arm=500;speed=30;sv=1;tick();
            @(negedge clk);sv=0;theta=-100;omega=100;arm=-200;speed=-100;
        end
    endtask
    initial begin
        repeat(2)tick();@(negedge clk);rst_n=1;begin_h();
        for(i=0;i<440;i=i+1) begin
            sample_input(i);
            wait(commit);#3;
            expected_theta=10+i%3;expected_mean=12300+i%20;
            check(states==={16'sd30,16'sd500,-16'sd20,expected_theta},"controller input snapshot changed mid-pipeline");
            expected[i]={8'd0,8'h0c,(i%2?8'h20:8'h00),detail[55:48],detail[47:32],
                         -command,command,integral,states,expected_mean,(32'd1000+i),counter};
            repeat(5)tick();
            check(total==i+1,"exactly one row per completed command");
        end
        check(integral!=0,"exercise nonzero integral with old/new distinction");
        @(negedge clk);stop=1;repeat(5)tick();
        check(frozen&&count==8&&total==440&&overwritten,"H stop freezes the retained suffix");
        for(i=0;i<8;i=i+1) begin
            @(negedge clk);rd=1;index=i;tick();
            check(rv&&data===expected[432+i],"ADC/state/integral/command row epoch mismatch");
            $display("TRACE_ROW %0d %064h",i,data);
        end
        @(negedge clk);rd=0;
        // Stop cancels work at every pre-commit pipeline stage.
        for(k=0;k<5;k=k+1) begin
            begin_h();sample_input(k);repeat(k)tick();
            @(negedge clk);stop=1;repeat(6)tick();
            check(frozen&&count==0&&total==0&&!commit,"cancelled pipeline must not become a trace row");
        end
        // A commit immediately preceding stop is real work; retain its request
        // with zero at the later motor gate, rather than inventing a cancelled row.
        begin_h();sample_input(33);wait(commit);#3;last_submitted=command;
        @(negedge clk);stop=1;repeat(6)tick();
        check(frozen&&count==1&&total==1,"stop after commit drains the pending completed row");
        @(negedge clk);rd=1;index=0;tick();
        check(rv&&data[191:176]===last_submitted&&data[207:192]===16'd0,"record submitted request and stopped motor gate separately");
        @(negedge clk);rd=0;
        // Board-key stop followed on the next clock by UART H may start a new
        // generation while the old stop is still waiting for its freeze edge.
        begin_h();sample_input(55);wait(commit);repeat(5)tick();
        begin_h();sample_input(56);wait(commit);repeat(5)tick();
        check(capturing&&!frozen&&count==1&&total==1,
              "immediate H restart must not inherit the old pending freeze");
        @(negedge clk);stop=1;repeat(6)tick();
        check(frozen&&count==1,"new generation freezes only on its own stop");
        begin_h();sample_input(88);repeat(2)tick();
        @(negedge clk);stop=1;bad=1;repeat(6)tick();
        check(frozen&&count==0&&reason==1&&state==0,
              "simultaneous stop and fault must retain actual stop exit reason");
        // Sensor fault also cancels pending computation and preserves its reason.
        begin_h();sample_input(99);repeat(2)tick();
        @(negedge clk);bad=1;repeat(6)tick();
        check(frozen&&count==0&&reason==2&&fault==1,"fault cancellation/freeze");
        $display("PASS control trace probe checks=%0d",checks);$finish;
    end
    initial begin #100000000;$fatal(1,"timeout");end
endmodule
