`timescale 1ns/1ps
module tb_adc_spike_blind;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0;
    reg [9:0] source=600, data_in=600;
    reg source_otr=0, otr=0;
    wire adc_clk, valid, bad, blind, fault, rejected;
    wire [13:0] raw_mean, control_mean;
    wire [7:0] reason;
    wire [271:0] detail;
    // Keep the production 5 MHz conversion clock and 64-conversion filter;
    // shorten only the reporting window to 200 conversions for fast coverage.
    adc_sampler #(.SAMPLE_CYCLES(125)) dut(
        .clk(clk),.rst_n(rst_n),.data_in(data_in),.otr(otr),.motor_observe(4'd0),
        .adc_clk(adc_clk),.valid(valid),.sample_q4(raw_mean),.control_q4(control_mean),
        .sample_bad(bad),.sample_blind(blind),.sample_fault(fault),.spike_rejected(rejected),
        .quality_reason(reason),.window_detail(detail));
    always @(negedge adc_clk) if(rst_n) begin #25; data_in=source; otr=source_otr; end
    task publish;
        begin @(posedge valid); #1; end
    endtask
    task settle(input integer code,input integer out_of_range);
        begin
            @(posedge adc_clk); #1; source=code; source_otr=out_of_range;
            repeat(4) publish;
        end
    endtask
    task pulse(input integer code,input integer conversions,input integer out_of_range);
        begin
            wait(dut.window_close); @(posedge clk); #1;
            repeat(20) @(posedge adc_clk);
            #1; source=code; source_otr=out_of_range;
            repeat(conversions) @(posedge adc_clk);
            #1; source=600; source_otr=0;
            publish;
        end
    endtask
    integer n, expected_mean, offset;
    reg boundary_guard=0, saw_last_candidate=0;
    always @(posedge clk) if(rst_n && boundary_guard && dut.window_close &&
        !dut.median_event && dut.candidate_count==63) saw_last_candidate=1;
    always @(posedge valid) begin
        #1;
        if(boundary_guard && (bad || blind || fault || control_mean!=9600))
            $fatal(1,"window boundary promoted an unconfirmed short pulse: control=%0d bad=%b",control_mean,bad);
    end
    initial begin #20000000; $fatal(1,"spike/blind timeout"); end
    initial begin
        repeat(3) @(negedge clk); rst_n=1;
        settle(600,0);
        if(bad || blind || fault || control_mean!=9600 || raw_mean!=9600)
            $fatal(1,"healthy baseline");
        // Median7 removes the first three lengths; the second stage removes
        // every remaining pulse shorter than 64 conversions on either side.
        for(n=1;n<=63;n=n+1) begin
            pulse(n[0] ? 700 : 500,n,0);
            if(control_mean!=9600 || bad || blind || fault || !rejected)
                $fatal(1,"short spike escaped: n=%0d raw=%0d control=%0d bad=%b blind=%b fault=%b",
                    n,raw_mean,control_mean,bad,blind,fault);
            expected_mean=9600;
            if(n>=4) expected_mean=9600+(n[0] ? 8*n : -8*n);
            if(raw_mean!=expected_mean || detail[223:208]!=200 ||
                raw_mean!=(detail[271:240]*16+100)/200)
                $fatal(1,"raw median/sum provenance changed n=%0d raw=%0d expected=%0d",n,raw_mean,expected_mean);
            publish;
        end
        // Move a 63-conversion pulse through the closing edge, including the
        // edge where a hypothetical next conversion would confirm it. Only
        // actual median events may contribute to the closing control span.
        boundary_guard=1;
        for(offset=125;offset<=142;offset=offset+1) begin
            wait(dut.window_close); @(posedge clk); #1;
            repeat(offset) @(posedge adc_clk);
            #1; source=700;
            repeat(63) @(posedge adc_clk);
            #1; source=600;
            publish; publish;
        end
        boundary_guard=0;
        if(!saw_last_candidate) $fatal(1,"63rd candidate / window-close boundary was not covered");
        pulse(0,1,1);
        if(control_mean!=9600 || bad || blind || fault || !rejected || (reason&3)!=3)
            $fatal(1,"filtered raw rail/OTR was promoted to failure");
        publish;
        pulse(600,1,1);
        if(control_mean!=9600 || bad || blind || fault || !reason[0])
            $fatal(1,"isolated nonrail OTR was promoted to failure");
        publish;
        pulse(600,64,1);
        if(!fault || !bad || blind || !reason[0]) $fatal(1,"sustained nonrail OTR was hidden");
        publish;
        if(fault || bad) $fatal(1,"qualified OTR leaked into next window");
        // True DC motion is admitted after bounded confirmation, not frozen.
        settle(700,0);
        if(control_mean!=11200 || bad || fault || blind) $fatal(1,"sustained step never admitted");
        settle(0,1);
        if(!blind || !bad || fault || control_mean!=0) $fatal(1,"low blind rail classified as electrical fault");
        // Cross the actual 0/1023 seam. A plausible mixed mean is still blind.
        wait(dut.window_close); @(posedge clk); #1;
        repeat(100) @(posedge adc_clk);
        #1; source=1023;
        publish;
        if(!blind || fault || !bad || raw_mean<400*16 || raw_mean>650*16)
            $fatal(1,"rail seam mixture became a trusted middle angle: raw=%0d blind=%b bad=%b fault=%b",
                raw_mean,blind,bad,fault);
        repeat(3) publish;
        if(!blind || !bad || fault || control_mean!=16368) $fatal(1,"high blind rail classified as electrical fault");
        settle(600,0);
        if(blind || bad || fault || control_mean!=9600) $fatal(1,"blind exit did not recover automatically");
        // Bounded normal motion is unchanged; there is no whole-frame delay.
        @(posedge adc_clk); #1; source=610;
        publish; publish;
        if(control_mean!=9760 || control_mean!=raw_mean || bad || blind || fault)
            $fatal(1,"small normal motion changed or acquired extra frame latency");
        $display("PASS tb_adc_spike_blind: all 1..63 conversion impulses, raw evidence, qualified OTR, blind seam, DC recovery and no frame delay");
        $finish;
    end
endmodule
