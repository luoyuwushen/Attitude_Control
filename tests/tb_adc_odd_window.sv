`timescale 1ns/1ps
module tb_adc_odd_window;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0;
    reg [9:0] source=600, data_in=600;
    wire adc_clk, valid, bad, blind, fault;
    wire [13:0] control_mean;
    wire [271:0] detail;
    // 193.6 conversions/report exercises both odd and even window lengths.
    adc_sampler #(.SAMPLE_CYCLES(121)) dut(
        .clk(clk),.rst_n(rst_n),.data_in(data_in),.otr(1'b0),.motor_observe(4'd0),
        .adc_clk(adc_clk),.valid(valid),.control_q4(control_mean),.sample_bad(bad),
        .sample_blind(blind),.sample_fault(fault),.window_detail(detail));
    always @(negedge adc_clk) if(rst_n) begin #25; data_in=source; end
    task publish;
        begin @(posedge valid); #1; end
    endtask
    integer n, length, j, odd_below=0, odd_above=0;
    initial begin #2000000; $fatal(1,"odd window timeout"); end
    initial begin
        repeat(3) @(negedge clk); rst_n=1;
        repeat(4) publish;
        for(length=102;length<=103;length=length+1) begin
            for(n=0;n<5;n=n+1) begin
                wait(dut.window_close); @(posedge clk); #1;
                repeat(20) @(posedge adc_clk);
                // The seven-point median rejects the first six alternating
                // values while baseline samples form the middle order statistic.
                // The remaining 96/97 values alternate by 800 codes, so none
                // can satisfy the coherent-excursion confirmation interval.
                for(j=0;j<length;j=j+1) begin
                    #1; source=j[0] ? 900 : 100;
                    @(posedge adc_clk);
                end
                #1; source=600;
                publish;
                if(detail[111:96]!=length-6 || control_mean!=9600 || blind || fault || bad!=(length==103))
                    $fatal(1,"half-window threshold: length=%0d count=%0d outliers=%0d control=%0d bad=%b",
                        length,detail[223:208],detail[111:96],control_mean,bad);
                if(detail[223:208]==193) begin
                    if(length==102) odd_below=odd_below+1;
                    else odd_above=odd_above+1;
                end else if(detail[223:208]!=194) $fatal(1,"fractional-window acquisition count");
            end
        end
        if(odd_below==0 || odd_above==0) $fatal(1,"both sides of odd-window half threshold were not covered");
        $display("PASS tb_adc_odd_window: true 96/193 replacements accepted, 97/193 rejected, even windows consistent and control mean preserved");
        $finish;
    end
endmodule
