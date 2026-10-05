`timescale 1ns/1ps
module tb_adc_quality;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0;
    reg [9:0] data_in=0, source_code=600;
    reg otr=0, source_otr=0;
    wire adc_clk, oe_n, valid, over_range, control_bad, sample_otr;
    // Raw diagnostics keep their previous independent oracle; control quality is tested separately.
    wire raw_quality_bad = |(quality_reason & 8'h5f);
    wire [9:0] raw_code, window_min, window_max;
    wire [13:0] sample_q4;
    wire [7:0] quality_reason;
    adc_sampler #(.SAMPLE_CYCLES(10)) dut(
        .clk(clk),.rst_n(rst_n),.data_in(data_in),.otr(otr),.motor_observe(4'd0),.adc_clk(adc_clk),.adc_oe_n(oe_n),
        .sample_q4(sample_q4),.valid(valid),.over_range(over_range),.raw_code(raw_code),
        .window_min(window_min),.window_max(window_max),.sample_bad(control_bad),.sample_otr(sample_otr),
        .quality_reason(quality_reason));
    wire production_clk, production_valid, production_bad;
    wire [13:0] production_q4;
    wire [7:0] production_reason;
    wire [271:0] production_detail;
    adc_sampler production_dut(.clk(clk),.rst_n(rst_n),.data_in(data_in),.otr(otr),.motor_observe(4'd0),
        .adc_clk(production_clk),.valid(production_valid),.sample_q4(production_q4),
        .sample_bad(production_bad),.quality_reason(production_reason),.window_detail(production_detail));
    reg [9:0] data_pipeline[0:3];
    reg otr_pipeline[0:3];
    reg [9:0] launched_code;
    reg launched_otr;
    integer j;
    time last_fall=0, last_valid=0, last_production_valid=0;
    integer production_samples=0;
    reg alternate=0, alternate_bit=0;
    reg [9:0] alternate_low=20, alternate_high=1000;
    reg monitor_ramp=0, monitor_span=0, saw_contribution_span=0, monitor_production_ramp=0;
    always @(posedge adc_clk) if(alternate) begin
        alternate_bit=~alternate_bit;
        source_code=alternate_bit ? alternate_high : alternate_low;
    end
    // Datasheet Fig.22: S1 on falling edge, Data1 after the fourth following
    // falling edge. Model tOD=25 ns typical; this is not a PVT maximum claim.
    always @(negedge adc_clk) if(rst_n) begin
        if(last_fall && $time-last_fall!=200) $fatal(1,"ADC clock is not 5MHz");
        last_fall=$time;
        launched_code=data_pipeline[3]; launched_otr=otr_pipeline[3];
        for(j=3;j>0;j=j-1) begin data_pipeline[j]=data_pipeline[j-1]; otr_pipeline[j]=otr_pipeline[j-1]; end
        data_pipeline[0]=source_code; otr_pipeline[0]=source_otr;
        #25; data_in=launched_code; otr=launched_otr;
    end
    always @(posedge valid) begin
        if(last_valid && $time-last_valid!=3200) $fatal(1,"16-point scaled cadence changed");
        last_valid=$time;
        #1;
        if(sample_otr !== quality_reason[0] || quality_reason[7]!=0)
            $fatal(1,"quality reason contract mismatch: reason=%h bad=%b",quality_reason,raw_quality_bad);
        if(monitor_ramp && (raw_quality_bad || control_bad))
            $fatal(1,"normal one-code-per-conversion ramp rejected: %h",quality_reason);
        if(monitor_span) begin
            if(quality_reason[3]) $fatal(1,"bounded block-mean ramp incorrectly hit step protection");
            if(quality_reason[2]) saw_contribution_span=1;
        end
    end
    always @(posedge production_valid) begin
        if(last_production_valid && $time-last_production_valid!=1000000)
            $fatal(1,"production cadence is not 1ms");
        last_production_valid=$time; production_samples=production_samples+1;
        #1;
        if(monitor_production_ramp && production_bad)
            $fatal(1,"bounded production-rate motion rejected: %h",production_reason);
    end
    task next_sample;
        begin @(posedge valid); #1; end
    endtask
    task next_source_window;
        begin wait(dut.window_close); @(posedge clk); #1; end
    endtask
    // Place a short conversion burst well inside one scaled output window.
    task pulse_code(input integer pulse_value, input integer conversions);
        begin
            next_source_window; repeat(2) @(posedge adc_clk);
            #1; source_code=pulse_value;
            repeat(conversions) @(posedge adc_clk);
            #1; source_code=600;
            // The publication 320 ns after this cut belongs to the previous
            // acquisition window; the burst belongs to the following one.
            next_sample;
        end
    endtask
    task settle(input integer code, input integer expect_bad);
        begin
            @(negedge clk); source_code=code; source_otr=0; alternate=0;
            repeat(10) @(posedge adc_clk);
            // A transition can survive into the preceding B8 block even
            // after the new window is constant; the cross-window block-step
            // guard correctly rejects that first constant window as well.
            next_sample; next_sample; next_sample;
            if(raw_code!=code || sample_q4!=code*16 || window_min!=code || window_max!=code ||
               sample_otr || raw_quality_bad!=expect_bad)
                $fatal(1,"static code/mean/quality: code=%0d raw=%0d sum=%0d min=%0d max=%0d bad=%0d otr=%0d reason=%h",
                    code,raw_code,sample_q4,window_min,window_max,raw_quality_bad,sample_otr,quality_reason);
        end
    endtask
    integer code, burst;
    initial begin #30000000; $fatal(1,"tb_adc_quality timeout"); end
    initial begin
        for(j=0;j<4;j=j+1) begin data_pipeline[j]=0; otr_pipeline[j]=0; end
        repeat(3) @(negedge clk); rst_n=1;
        // All ten raw bits retain their straight-binary mapping, including rails.
        for(code=0;code<1024;code=code+1) begin
            @(negedge clk); source_code=code;
            repeat(12) @(posedge adc_clk);
            #1; if(raw_code!==code) $fatal(1,"raw 10-bit mapping at %0d: %0d",code,raw_code);
        end
        settle(0,1); settle(1,0); settle(1022,0); settle(1023,1); settle(600,0);
        // A single interior +40 conversion is diagnostic evidence, not a
        // contaminated angle: every median remains exactly 600.
        pulse_code(640,1);
        if(sample_q4!=9600 || window_min!=600 || window_max!=640 || raw_quality_bad || quality_reason!=8'h20)
            $fatal(1,"isolated outlier was not rejected by median: mean=%0d reason=%h",sample_q4,quality_reason);
        next_sample;
        if(quality_reason!=0) $fatal(1,"raw warning leaked into the next window");
        pulse_code(560,1);
        if(sample_q4!=9600 || window_min!=560 || window_max!=600 || raw_quality_bad || quality_reason!=8'h20)
            $fatal(1,"isolated negative outlier was not rejected by median");
        next_sample;
        // Any one-, two-, or three-conversion interior burst is outvoted by
        // four baseline values in every seven-point window, on either side.
        pulse_code(640,2);
        if(sample_q4!=9600 || raw_quality_bad || quality_reason!=8'h20)
            $fatal(1,"two-conversion positive burst escaped median7 rejection");
        next_sample;
        pulse_code(560,2);
        if(sample_q4!=9600 || raw_quality_bad || quality_reason!=8'h20)
            $fatal(1,"two-conversion negative burst escaped median7 rejection");
        next_sample;
        pulse_code(640,3);
        if(sample_q4!=9600 || raw_quality_bad || quality_reason!=8'h20)
            $fatal(1,"three-conversion positive burst escaped median7 rejection");
        next_sample;
        pulse_code(560,3);
        if(sample_q4!=9600 || raw_quality_bad || quality_reason!=8'h20)
            $fatal(1,"three-conversion negative burst escaped median7 rejection");
        next_sample;
        // Four adjacent outliers survive median7. In a B=8 scaled block their
        // variance exceeds 256 even if divided 2+2 across neighboring blocks.
        // Block means remain within 32 codes, so the exact reason is 0x60.
        pulse_code(640,4);
        if(sample_q4!=9760 || !raw_quality_bad || quality_reason!=8'h60)
            $fatal(1,"four-conversion burst escaped scaled block variance: mean=%0d reason=%h",sample_q4,quality_reason);
        next_sample;
        pulse_code(560,4);
        if(sample_q4!=9440 || !raw_quality_bad || quality_reason!=8'h60)
            $fatal(1,"negative majority burst escaped scaled block variance: mean=%0d reason=%h",sample_q4,quality_reason);
        next_sample;
        if(quality_reason!=0) $fatal(1,"median fault leaked after a complete healthy window");
        // OTR and raw rails bypass the median, even when its output is sound.
        pulse_code(0,1);
        if(sample_q4!=9600 || !raw_quality_bad || quality_reason!=8'h22 || sample_otr)
            $fatal(1,"single low rail was hidden by median: %h",quality_reason);
        next_sample;
        pulse_code(1023,1);
        if(sample_q4!=9600 || !raw_quality_bad || quality_reason!=8'h22 || sample_otr)
            $fatal(1,"single high rail was hidden by median: %h",quality_reason);
        next_sample;
        // A one-conversion OTR pulse vanishes long before the output interval.
        next_sample; repeat(3) @(posedge adc_clk);
        #1; source_otr=1;
        @(posedge adc_clk); #1; source_otr=0;
        wait(over_range===1); wait(over_range===0);
        next_sample;
        if(!sample_otr || !raw_quality_bad || over_range || sample_q4!=9600 || quality_reason!=8'h01)
            $fatal(1,"short OTR disappeared before the window report");
        next_sample;
        if(sample_otr || raw_quality_bad) $fatal(1,"OTR leaked into an unrelated next window");
        // Put the raw OTR capture on the same edge as a source window cut.
        // Its registered quality is consumed next clock, so the next window
        // owns it; it must neither vanish nor contaminate both windows.
        wait(dut.sample_count==10);
        @(posedge adc_clk); #1; source_otr=1;
        @(posedge adc_clk); #1; source_otr=0;
        wait(over_range===1); #1;
        if(dut.interval_count!=0 || dut.sample_count!=0 || valid)
            $fatal(1,"boundary OTR test did not reach the source-cut / delayed-publication edge");
        next_sample;
        if(sample_otr) $fatal(1,"new raw capture was mixed into the closing window");
        next_sample;
        if(!sample_otr || !raw_quality_bad) $fatal(1,"boundary OTR was lost");
        next_sample;
        if(sample_otr || raw_quality_bad) $fatal(1,"boundary OTR was reported twice");
        // The same publication boundary rule applies to a raw rail conversion.
        wait(dut.sample_count==10);
        @(posedge adc_clk); #1; source_code=0;
        @(posedge adc_clk); #1; source_code=600;
        wait(raw_code===0); #1;
        if(dut.interval_count!=0 || dut.sample_count!=0 || valid)
            $fatal(1,"boundary raw rail missed the source-cut edge");
        next_sample;
        if(raw_quality_bad) $fatal(1,"boundary raw rail contaminated the previous window");
        next_sample;
        if(quality_reason!=8'h22 || sample_q4!=9600) $fatal(1,"boundary raw rail disappeared: %h",quality_reason);
        next_sample;
        if(raw_quality_bad) $fatal(1,"boundary raw rail was reported twice");
        // Equal halves on opposite ends produce a plausible mean of 510.
        // The alternating median stream preserves the discontinuity, so a
        // plausible mean cannot hide a real crossing or sustained corruption.
        @(negedge clk); alternate=1;
        repeat(24) @(posedge adc_clk); next_sample; next_sample;
        if(sample_q4!=8160 || window_min!=20 || window_max!=1000 || !raw_quality_bad || sample_otr)
            $fatal(1,"cross-boundary values were averaged into an apparently healthy code");
        @(negedge clk); alternate_low=600; alternate_high=632;
        repeat(24) @(posedge adc_clk); next_sample; next_sample;
        if(sample_q4!=9856 || window_min!=600 || window_max!=632 || raw_quality_bad)
            $fatal(1,"32-code quality boundary/fractional averaging");
        @(negedge clk); alternate_high=633;
        repeat(24) @(posedge adc_clk); next_sample; next_sample;
        if(sample_q4!=9864 || window_min!=600 || window_max!=633 || !raw_quality_bad)
            $fatal(1,"33-code excursion was not rejected");
        settle(600,0);
        monitor_ramp=1;
        for(code=601;code<=660;code=code+1) begin
            @(posedge adc_clk); #1; source_code=code;
        end
        repeat(10) @(posedge adc_clk);
        next_sample; next_sample;
        monitor_ramp=0;
        if(sample_q4!=10560) $fatal(1,"normal ramp did not converge to its true value");
        settle(600,0);
        // Three codes/conversion give successive B8 means only 24 apart;
        // within-block variance is 47.25. Both blocks are therefore valid.
        next_sample;
        monitor_span=1;
        for(code=603;code<=840;code=code+3) begin
            @(posedge adc_clk); #1; source_code=code;
        end
        next_sample;
        #1; monitor_span=0;
        if(saw_contribution_span) $fatal(1,"scaled block-mean span incorrectly used median extremes");
        settle(600,0);
        // Repeat short positive/negative bursts at the real 1 ms cadence.
        // They stay between diagnostic observations, while every filtered
        // conversion participates in control averaging and quality blocks.
        repeat(2) @(posedge production_valid);
        wait(production_dut.interval_count==20);
        for(burst=1;burst<=3;burst=burst+1) begin
            @(posedge adc_clk); #1; source_code=640;
            repeat(burst) @(posedge adc_clk); #1; source_code=600;
            repeat(12) @(posedge adc_clk);
            #1; source_code=560;
            repeat(burst) @(posedge adc_clk); #1; source_code=600;
            repeat(12) @(posedge adc_clk);
        end
        @(posedge production_valid); #1;
        if(production_q4!=9600 || production_bad || production_reason!=8'h20)
            $fatal(1,"production short-burst rejection changed the trusted mean: %0d/%h",production_q4,production_reason);
        @(posedge production_valid); #1;
        if(production_bad || production_reason!=0) $fatal(1,"production short-burst warning leaked");
        // A three-conversion burst straddles a publication in the captured
        // raw stream. Median history must persist across window boundaries.
        wait(production_dut.sample_count==15 && production_dut.interval_count==3060);
        @(posedge adc_clk); #1; source_code=640;
        repeat(3) @(posedge adc_clk); #1; source_code=600;
        @(posedge production_valid); #1;
        if(production_q4!=9600 || production_bad || production_reason!=8'h20)
            $fatal(1,"boundary short-burst first window: %0d/%h",production_q4,production_reason);
        @(posedge production_valid); #1;
        if(production_q4!=9600 || production_bad || production_reason!=8'h20)
            $fatal(1,"boundary short-burst second window: %0d/%h",production_q4,production_reason);
        @(posedge production_valid); #1;
        if(production_bad || production_reason!=0) $fatal(1,"boundary short-burst warning leaked twice");
        // Four +40 medians in a production B40 block have variance at most
        // 144; all 5000 values now contribute to the rounded control mean.
        repeat(2) @(posedge production_valid);
        wait(production_dut.interval_count==20);
        @(posedge adc_clk); #1; source_code=640;
        repeat(4) @(posedge adc_clk);
        #1; source_code=600;
        @(posedge production_valid); #1;
        if(production_q4!=9601 || production_bad || production_reason!=8'h20)
            $fatal(1,"bounded production burst mean/quality: mean=%0d reason=%h",production_q4,production_reason);
        @(posedge production_valid); #1;
        if(production_bad || production_reason!=0) $fatal(1,"production fault did not delimit its window");
        // A smooth excursion fits entirely in one 62.5 us observation gap:
        // 600 -> 20 -> 1000 -> 20 -> 600, only 20 codes per conversion. Thus
        // Every filtered value now contributes to control, and complete B40
        // block moments must invalidate the excursion between observation ticks.
        wait(production_dut.interval_count==20);
        for(code=580;code>=20;code=code-20) begin
            @(posedge adc_clk); #1; source_code=code;
        end
        for(code=40;code<=1000;code=code+20) begin
            @(posedge adc_clk); #1; source_code=code;
        end
        for(code=980;code>=20;code=code-20) begin
            @(posedge adc_clk); #1; source_code=code;
        end
        for(code=40;code<=600;code=code+20) begin
            @(posedge adc_clk); #1; source_code=code;
        end
        repeat(10) @(posedge adc_clk);
        if(production_dut.sample_count!=0)
            $fatal(1,"smooth excursion did not fit wholly before the next diagnostic observation");
        @(posedge production_valid); #1;
        if(production_q4==9600 || !production_bad || !production_reason[2] || !production_reason[6])
            $fatal(1,"smooth inter-observation excursion escaped complete block coverage: mean=%0d reason=%h",production_q4,production_reason);
        @(posedge production_valid); #1;
        if(production_bad || production_reason!=0) $fatal(1,"quality blocks leaked into the next window");
        // Real-rate motion: about 16 codes/ms, with small continuous changes.
        // A healthy trajectory below the unchanged 32-code/window threshold
        // must retain its value instead of becoming a frozen noise estimate.
        monitor_production_ramp=1;
        for(code=601;code<=616;code=code+1) begin
            @(posedge adc_clk); #1; source_code=code;
            repeat(312) @(posedge adc_clk);
        end
        repeat(2) @(posedge production_valid); #1;
        monitor_production_ramp=0;
        if(production_q4!=9856 || production_bad)
            $fatal(1,"production slow motion did not converge to its true value");
        // Fault injection drops exactly one internal median event. Independent
        // acquisition counting must veto N=4999; the unfinished B40 block must
        // be discarded at the cut so the next complete window realigns.
        wait(production_dut.sample_count==8);
        wait(production_dut.median_event);
        @(negedge clk); force production_dut.median_event=0;
        @(posedge clk); #1; release production_dut.median_event;
        @(posedge production_valid); #1;
        if(production_q4!=9856 || production_reason!=8'h10 || !production_bad ||
            production_detail[223:208]!=4999 || production_detail[271:240]!=4999*616)
            $fatal(1,"incomplete full-stream window not rejected/count-aligned: q4=%0d reason=%h count=%0d sum=%0d",
                production_q4,production_reason,production_detail[223:208],production_detail[271:240]);
        @(posedge production_valid); #1;
        if(production_q4!=9856 || production_bad || production_reason!=0 ||
            production_detail[223:208]!=5000 || production_detail[271:240]!=5000*616)
            $fatal(1,"partial quality block spilled into recovered complete window");
        if(production_samples<2 || oe_n!==0) $fatal(1,"production startup/cadence/output enable");
        $display("PASS tb_adc_quality: median7 short bursts, scaled/production block variance, raw rails/OTR delayed-publication boundaries, continuous mean, smooth inter-observation excursion, ramps, raw mapping, 1ms cadence");
        $finish;
    end
endmodule
