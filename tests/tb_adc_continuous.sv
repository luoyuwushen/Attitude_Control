`timescale 1ns/1ps
// Production-rate black-box arithmetic oracle. No force or DUT-internal probes.
module tb_adc_continuous;
    localparam integer SC=3125, N=5000, B=40;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0;
    reg [9:0] data_in=300;
    reg otr=0;
    wire adc_clk, valid, control_bad, sample_otr;
    // Independent raw quality oracle; filtered control validity has a separate regression.
    wire raw_quality_bad = |(reason & 8'h5f);
    wire [13:0] mean_q4;
    wire [9:0] raw_code, wmin, wmax;
    wire [7:0] reason;
    wire [271:0] detail;
    adc_sampler dut(.clk(clk),.rst_n(rst_n),.data_in(data_in),.otr(otr),
        .motor_observe(4'd0),.adc_clk(adc_clk),.valid(valid),.sample_bad(control_bad),
        .sample_otr(sample_otr),.sample_q4(mean_q4),.raw_code(raw_code),
        .window_min(wmin),.window_max(wmax),.quality_reason(reason),.window_detail(detail));

    // Captured code ID 57 is the first eligible median's newest input. A
    // monotonic step appears after three more inputs, hence ID 54 aligns its
    // median output to index zero. The scoreboard verifies actual ownership.
    function integer waveform(input integer id);
        integer p, window_number, position, burst;
        begin
            p=id-54; window_number=p/N; position=p%N;
            waveform=300;
            if(p>=0) begin
                case(window_number)
                    1,2,3: waveform=(id%2) ? 284 : 316;
                    4,5: waveform=(id%2) ? 283 : 317;
                    6,7,8: waveform=(id%2) ? 20 : 1000;
                    10: begin
                        // Forty separated five-output +27 bursts visit every
                        // possible offset within a production quality block.
                        for(burst=0;burst<40;burst=burst+1)
                            if(position>=200+81*burst && position<205+81*burst)
                                waveform=327;
                    end
                    11: if(position>=4998) waveform=327;
                    12: if(position<3) waveform=327;
                    13: if(position==997) waveform=0;
                    14: if(position==1997) waveform=1023;
                    17,18: waveform=333;
                    19: waveform=333+((position<2500 ? position : 4999-position)/50);
                    20,21: waveform=333;
                    default: waveform=300;
                endcase
            end
        end
    endfunction
    function integer source_otr(input integer id);
        begin source_otr=(id==54+15*N+2997); end
    endfunction
    integer conversion_id=0, pipe_code[0:3], pipe_otr[0:3], launched_code, launched_otr, j;
    time last_fall=0;
    // Datasheet four-conversion pipeline, 25 ns output delay. Only external
    // pins are driven. The same code/OTR transaction travels together.
    always @(negedge adc_clk) if(rst_n) begin
        if(last_fall && $time-last_fall!=200) $fatal(1,"ADC cadence changed");
        last_fall=$time;
        launched_code=pipe_code[3]; launched_otr=pipe_otr[3];
        for(j=3;j>0;j=j-1) begin pipe_code[j]=pipe_code[j-1]; pipe_otr[j]=pipe_otr[j-1]; end
        pipe_code[0]=waveform(conversion_id); pipe_otr[0]=source_otr(conversion_id);
        conversion_id=conversion_id+1;
        #25; data_in=launched_code; otr=launched_otr;
    end

    integer captures=0, history[0:6], selected=300, a, b, rank;
    reg selected_ready=0;
    // Rank statistic, intentionally different from the RTL sorting network.
    always @(posedge adc_clk) begin
        #81;
        if(rst_n) begin
            captures=captures+1;
            if(raw_code!==data_in) $fatal(1,"coherent external capture mismatch");
            for(a=0;a<6;a=a+1) history[a]=history[a+1];
            history[6]=data_in;
            selected_ready=captures>=7;
            if(selected_ready) begin
                for(a=0;a<7;a=a+1) begin
                    rank=0;
                    for(b=0;b<7;b=b+1)
                        if(history[b]<history[a] || (history[b]==history[a] && b<a)) rank=rank+1;
                    if(rank==3) selected=history[a];
                end
            end
        end
    end

    integer count=0, total=0, filtered_min=1023, filtered_max=0;
    integer raw_min=1023, raw_max=0, raw_count=0, raw_flags=0;
    integer block_count=0, block_sum=0, block_min_sum=0, block_max_sum=0;
    integer previous_block_sum=0, quality_flags=0, blocks=0;
    reg previous_block_seen=0;
    longint block_squares=0, variance_numerator;
    integer perturbations=0, burst_starts=0, burst_run=0, block_offsets[0:39];
    integer window_number=0;
    // Raw OTR and rails belong to their coherent capture consumed one clock
    // after capture. They remain independent of median selection.
    always @(posedge adc_clk) begin
        #101;
        if(rst_n && captures>=63) begin
            raw_count=raw_count+1;
            if(data_in<raw_min) raw_min=data_in;
            if(data_in>raw_max) raw_max=data_in;
            if(otr) raw_flags=raw_flags|1;
            if(data_in==0 || data_in==1023) raw_flags=raw_flags|2;
        end
    end
    // The previous capture's median is consumed at this next rising+40 ns.
    always @(posedge adc_clk) begin
        #41;
        if(rst_n && selected_ready && captures>=63) begin
            total=total+selected;
            if(selected<filtered_min) filtered_min=selected;
            if(selected>filtered_max) filtered_max=selected;
            if(window_number==10 && selected==327) begin
                perturbations=perturbations+1;
                if(!burst_run) begin block_offsets[count%B]=block_offsets[count%B]+1; burst_starts=burst_starts+1; end
                burst_run=burst_run+1;
            end else if(window_number==10 && burst_run) begin
                if(burst_run!=5) $fatal(1,"short burst has %0d filtered outputs instead of five",burst_run);
                burst_run=0;
            end
            count=count+1;
            block_sum=block_sum+selected;
            block_squares=block_squares+selected*selected;
            block_count=block_count+1;
            if(block_count==B) begin
                // Exact independent integer moments; no rounded block means.
                variance_numerator=B*block_squares-longint'(block_sum)*block_sum;
                if(variance_numerator>256*B*B) quality_flags=quality_flags|64;
                if(blocks==0 || block_sum<block_min_sum) block_min_sum=block_sum;
                if(blocks==0 || block_sum>block_max_sum) block_max_sum=block_sum;
                if(block_max_sum-block_min_sum>32*B) quality_flags=quality_flags|4;
                if(previous_block_seen &&
                   (block_sum-previous_block_sum>32*B || previous_block_sum-block_sum>32*B))
                    quality_flags=quality_flags|8;
                previous_block_sum=block_sum; previous_block_seen=1;
                blocks=blocks+1; block_count=0; block_sum=0; block_squares=0;
            end
        end
    end

    integer eligible_ticks=0, expected_mean=0, expected_sum=0, expected_count=0;
    integer expected_raw_min=0, expected_raw_max=0, expected_min=0, expected_max=0;
    integer expected_reason=0, expected_reference=0, previous_mean=0, expected_window=0;
    time expected_publication=0, previous_publication=0;
    always @(posedge clk) if(rst_n && captures>=63) begin
        eligible_ticks=eligible_ticks+1;
        if(eligible_ticks%(16*SC)==0) begin
            expected_publication=$time+320;
            #2; // Include any same-edge external oracle median/raw consumer.
            if(count!=N || raw_count!=N || blocks!=N/B || block_count!=0)
                $fatal(1,"production window ownership: median=%0d raw=%0d blocks=%0d tail=%0d",count,raw_count,blocks,block_count);
            expected_mean=(16*total+count/2)/count;
            expected_sum=total; expected_count=count;
            expected_raw_min=raw_min; expected_raw_max=raw_max;
            expected_min=filtered_min; expected_max=filtered_max;
            expected_reason=quality_flags|raw_flags|((raw_max-raw_min>32) ? 32 : 0);
            expected_reference=previous_mean; previous_mean=expected_mean;
            expected_window=window_number; window_number=window_number+1;
            count=0; total=0; filtered_min=1023; filtered_max=0;
            raw_min=1023; raw_max=0; raw_count=0; raw_flags=0;
            quality_flags=0; blocks=0; block_min_sum=0; block_max_sum=0;
        end
    end
    integer checked_windows=0, offset;
    always @(posedge valid) begin
        if($time!=expected_publication) $fatal(1,"valid did not occur cut+320ns");
        if(previous_publication && $time-previous_publication!=1000000) $fatal(1,"valid cadence not 1ms");
        previous_publication=$time;
        #2;
        if(mean_q4!==expected_mean || detail[271:240]!==expected_sum ||
           detail[16*13+:16]!==expected_count || detail[15:0]!==expected_reference ||
           detail[16*1+:16]!==expected_min || detail[16*2+:16]!==expected_max ||
           wmin!==expected_raw_min || wmax!==expected_raw_max ||
           reason!==expected_reason || raw_quality_bad!==((expected_reason&8'h5f)!=0) ||
           sample_otr!==((expected_reason&1)!=0))
            $fatal(1,"oracle window=%0d mean=%0d/%0d sum=%0d/%0d count=%0d/%0d raw=%0d..%0d/%0d..%0d filtered=%0d..%0d/%0d..%0d reason=%h/%h",
                expected_window,mean_q4,expected_mean,detail[271:240],expected_sum,
                detail[16*13+:16],expected_count,wmin,wmax,expected_raw_min,expected_raw_max,
                detail[16*1+:16],detail[16*2+:16],expected_min,expected_max,reason,expected_reason);
        $display("CHECK production window=%0d mean_q4=%0d count=%0d sum=%0d reason=%02h",expected_window,mean_q4,expected_count,expected_sum,reason);
        case(expected_window)
            0: if(reason!=0 || mean_q4!=4800) $fatal(1,"static baseline");
            2: if(raw_quality_bad || reason!=0 || mean_q4!=4800) $fatal(1,"RMS16 exact boundary rejected");
            4: if(!raw_quality_bad || reason!=8'h60 || mean_q4!=4800) $fatal(1,"RMS17 not rejected");
            7: if(!raw_quality_bad || reason!=8'h60 || mean_q4!=8160) $fatal(1,"20/1000 plausible mean hid corruption");
            10: begin
                if(raw_quality_bad || perturbations!=200 || burst_starts!=40 || mean_q4!=4817)
                    $fatal(1,"five-output +27 bursts rejected/lost: %0d/%0d q4=%0d reason=%h",perturbations,burst_starts,mean_q4,reason);
                for(offset=0;offset<40;offset=offset+1)
                    if(block_offsets[offset]!=1) $fatal(1,"missing short burst block offset %0d",offset);
            end
            11,12: if(raw_quality_bad || expected_sum!=1500000+27*(expected_window==11 ? 2 : 3))
                $fatal(1,"five-output boundary burst lost or double counted window=%0d sum=%0d",expected_window,expected_sum);
            13,14: if(!raw_quality_bad || !reason[1] || mean_q4!=4800) $fatal(1,"single raw rail hidden");
            15: if(!raw_quality_bad || reason!=1 || mean_q4!=4800) $fatal(1,"single raw OTR hidden");
            17: if(!raw_quality_bad || !reason[3] || reason[2] || expected_min!=333 || expected_max!=333)
                $fatal(1,"whole-window +33 DC step lost across windows: reason=%h range=%0d..%0d",reason,expected_min,expected_max);
            18: if(raw_quality_bad || mean_q4!=5328) $fatal(1,"DC step fault leaked into stable successor");
            19: if(!raw_quality_bad || !reason[2] || reason[3] || reason[6])
                $fatal(1,"slow smooth excursion span not distinguished from noise/step: %h",reason);
        endcase
        checked_windows=checked_windows+1;
        if(checked_windows==22) begin
            $display("PASS tb_adc_continuous: production SC3125/N5000/B40 external ADC + independent median/moment oracle; exact Q4/count/sum/coherent publication +320ns/1ms; RMS16/17, alternating20/1000, all40 burst offsets, cross-window ownership, raw rails/OTR, cross-window DC step, slow span");
            $finish;
        end
    end
    integer i;
    initial begin
        for(i=0;i<4;i=i+1) begin pipe_code[i]=300; pipe_otr[i]=0; end
        for(i=0;i<7;i=i+1) history[i]=300;
        for(i=0;i<40;i=i+1) block_offsets[i]=0;
        repeat(3) @(negedge clk); rst_n=1;
    end
    initial begin #23000000; $fatal(1,"tb_adc_continuous timeout"); end
endmodule
