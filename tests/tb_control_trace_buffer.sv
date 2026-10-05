`timescale 1ns/1ps
module tb_control_trace_buffer;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0,start_capture=0,freeze_capture=0,row_valid=0,read_request=0;
    reg [7:0] reason=1;
    reg [31:0] time_ms=0;
    reg [13:0] down=3558,up=12349;
    reg signed [15:0] capture=-99;
    reg [255:0] row=0;
    reg [3:0] read_index=0;
    wire capturing,frozen,overwritten,read_valid;
    wire [15:0] id;
    wire [3:0] count;
    wire [31:0] total,freeze_time;
    wire [7:0] saved_reason;
    wire [13:0] saved_down,saved_up;
    wire signed [15:0] saved_capture;
    wire [255:0] read_data;
    integer checks=0,i;
    control_trace_buffer #(.ADDRESS_BITS(3)) dut(
        .clk(clk),.rst_n(rst_n),.start_capture(start_capture),
        .freeze_capture(freeze_capture),.freeze_reason_in(reason),
        .device_time_ms(time_ms),.down_q4(down),.up_q4(up),.capture_arm_q10(capture),
        .row_valid(row_valid),.row_data(row),.capturing(capturing),.frozen(frozen),
        .capture_id(id),.row_count(count),.total_committed(total),.overwritten(overwritten),
        .freeze_time_ms(freeze_time),.freeze_reason(saved_reason),
        .saved_down_q4(saved_down),.saved_up_q4(saved_up),.saved_capture_arm_q10(saved_capture),
        .read_request(read_request),.read_index(read_index),.read_valid(read_valid),.read_data(read_data));
    function [255:0] pattern;
        input integer n;
        begin pattern={32'hf0feaa55,32'h10203040,32'hdeadbeef,32'h80000001,
                       32'h01020304,32'hffffffff,32'h00000000,n[31:0]}; end
    endfunction
    task check;
        input condition;
        input [1023:0] message;
        begin checks=checks+1; if (condition!==1'b1) $fatal(1,"%0s",message); end
    endtask
    task tick;
        begin @(posedge clk); #1; end
    endtask
    task put;
        input integer n;
        begin @(negedge clk); row_valid=1;row=pattern(n);tick(); end
    endtask
    task get;
        input integer ordinal;
        input integer expected;
        begin
            @(negedge clk);row_valid=0;read_request=1;read_index=ordinal;
            tick();check(read_valid,"expected valid synchronous read");
            check(read_data===pattern(expected),"incorrect chronological row or payload corruption");
        end
    endtask
    initial begin
        repeat(2) tick(); @(negedge clk);rst_n=1;
        tick();check(!capturing&&!frozen&&!read_valid&&read_data==0,"reset state");
        @(negedge clk);start_capture=1;row_valid=1;row=pattern(999);tick();
        check(capturing&&id==1&&count==0&&total==0,"start discards unrelated same-cycle row");
        check(saved_down==3558&&saved_up==12349&&saved_capture==-99,"metadata snapshot");
        @(negedge clk);start_capture=0;row_valid=0;down=0;up=0;capture=0;
        for(i=0;i<19;i=i+1) begin
            put(i);
            check(total==i+1,"commit count");
            check(count==(i<8?i+1:8),"bounded row count");
            check(overwritten==(i>=8),"overwrite flag only after eviction");
        end
        @(negedge clk);row_valid=0;read_request=1;read_index=0;tick();
        check(!read_valid&&read_data==0,"cannot read a moving ring");
        @(negedge clk);row_valid=1;row=pattern(19);freeze_capture=1;time_ms=701;tick();
        check(frozen&&!capturing&&count==8&&total==20&&freeze_time==701&&saved_reason==1,"freeze includes completed row");
        @(negedge clk);freeze_capture=0;
        for(i=0;i<8;i=i+1)get(i,12+i);
        @(negedge clk);read_request=1;read_index=8;row_valid=1;row=pattern(900);tick();
        check(!read_valid&&read_data==0&&total==20,"out-of-range read and frozen write ignored");
        get(7,19);
        @(negedge clk);start_capture=1;read_request=1;read_index=0;tick();
        check(id==2&&count==0&&!frozen&&!read_valid&&read_data==0,"new generation invalidates old read");
        @(negedge clk);start_capture=0;read_request=0;row_valid=0;
        put(100);put(101);
        @(negedge clk);freeze_capture=1;row_valid=0;reason=2;time_ms=800;tick();
        check(count==2&&total==2&&!overwritten&&saved_reason==2,"partial capture freeze");
        @(negedge clk);freeze_capture=0;
        get(0,100);get(1,101);
        @(negedge clk);start_capture=1;freeze_capture=1;reason=3;time_ms=900;tick();
        check(id==3&&frozen&&!capturing&&count==0&&freeze_time==900,"empty cancelled capture");
        @(negedge clk);start_capture=0;freeze_capture=0;read_request=1;read_index=0;tick();
        check(!read_valid&&read_data==0,"empty buffer hides stale RAM");
        @(negedge clk);rst_n=0;#1;
        check(!read_valid&&read_data==0&&count==0&&id==0,"asynchronous reset hides RAM output");
        $display("PASS control trace buffer checks=%0d",checks);$finish;
    end
    initial begin #1000000;$fatal(1,"timeout");end
endmodule
