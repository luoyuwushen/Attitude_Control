`timescale 1ns/1ps
module tb_adc_observability;
    // Fractional 20/21-conversion windows rotate through all even ADC phases.
    // QUALITY_BLOCK=1 here checks ownership; default production tests cover B40.
    localparam integer SC=13;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0;
    reg [9:0] source_code=600, data_in=600;
    reg source_otr=0, otr=0;
    reg [3:0] motor_next=0, motor_observe=0;
    always @(posedge clk) motor_observe <= rst_n ? motor_next : 4'd0;
    wire adc_clk, oe, valid, over_range, bad, sample_otr;
    wire [9:0] raw_code, wmin, wmax;
    wire [13:0] mean_q4;
    wire [7:0] reason;
    wire [271:0] detail, limited_detail;
    adc_sampler #(.SAMPLE_CYCLES(SC)) dut(
        .clk(clk),.rst_n(rst_n),.data_in(data_in),.otr(otr),.motor_observe(motor_observe),
        .adc_clk(adc_clk),.adc_oe_n(oe),.sample_q4(mean_q4),.valid(valid),.over_range(over_range),
        .raw_code(raw_code),.window_min(wmin),.window_max(wmax),.sample_bad(bad),
        .sample_otr(sample_otr),.quality_reason(reason),.window_detail(detail));
    wire limited_clk, limited_oe, limited_valid, limited_over_range, limited_bad, limited_otr;
    wire [9:0] limited_raw, limited_min, limited_max;
    wire [13:0] limited_q4;
    wire [7:0] limited_reason;
    adc_sampler #(.SAMPLE_CYCLES(SC),.DIAG_COUNT_MAX(16'd7),.DIAG_SUM_MAX(32'd4095)) limited(
        .clk(clk),.rst_n(rst_n),.data_in(data_in),.otr(otr),.motor_observe(motor_observe),
        .adc_clk(limited_clk),.adc_oe_n(limited_oe),.sample_q4(limited_q4),.valid(limited_valid),
        .over_range(limited_over_range),.raw_code(limited_raw),.window_min(limited_min),
        .window_max(limited_max),.sample_bad(limited_bad),.sample_otr(limited_otr),
        .quality_reason(limited_reason),.window_detail(limited_detail));
    wire production_valid;
    wire [271:0] production_detail;
    adc_sampler production(.clk(clk),.rst_n(rst_n),.data_in(data_in),.otr(otr),
        .motor_observe(motor_observe),.valid(production_valid),.window_detail(production_detail));
    // Diagnostic saturation must never feed back into the control path.
    // The v0.2.1 median3 fixture remains a historical artifact, not a median7
    // oracle: an independent seven-value order statistic is checked below.
    always @(posedge clk) begin
        #2;
        if({limited_clk,limited_oe,limited_q4,limited_valid,limited_over_range,limited_raw,
            limited_min,limited_max,limited_bad,limited_otr,limited_reason} !==
           {adc_clk,oe,mean_q4,valid,over_range,raw_code,wmin,wmax,bad,sample_otr,reason})
            $fatal(1,"diagnostic saturation changed a control output");
    end

    // Independent external device model. Each conversion receives a unique
    // transaction ID; data ID N is launched at falling edge N+4, after 25 ns.
    // Motor metadata is sampled just before the next system edge (fall+19 ns),
    // avoiding simulation races while representing the specified +20 ns
    // registered observation. All motor transitions here occur on clock edges.
    integer conversion_id=0, data_id=-1, launched_id, j, active_id;
    integer data_ids[0:3];
    reg [9:0] conversion_value[0:19999];
    reg conversion_otr[0:19999];
    reg [19:0] conversion_metadata[0:19999];
    time motor_change_time=0, last_rise=0;
    reg motor_edge_known=0;
    integer observed_age;
    always @(motor_observe) if(rst_n) begin
        motor_change_time=$time; motor_edge_known=1;
    end
    always @(negedge adc_clk) if(rst_n) begin
        active_id=conversion_id; conversion_id=conversion_id+1;
        conversion_value[active_id]=source_code; conversion_otr[active_id]=source_otr;
        launched_id=data_ids[3];
        for(j=3;j>0;j=j-1) data_ids[j]=data_ids[j-1];
        data_ids[0]=active_id;
        #19;
        observed_age=motor_edge_known ? ($time-motor_change_time)/20 : 65535;
        if(observed_age>65535) observed_age=65535;
        conversion_metadata[active_id]={motor_observe,observed_age[15:0]};
        #6;
        if(launched_id>=0) begin
            data_in=conversion_value[launched_id]; otr=conversion_otr[launched_id]; data_id=launched_id;
        end
    end

    integer captures=0, median_events=0, eligible_ticks=0;
    integer hvalue[0:6], hid[0:6];
    integer selected_value, selected_id, selected_slot, order_rank, a, b;
    integer slot_coverage[0:6], history_ties=0;
    reg oracle_pending_ready=0;
    integer last_selected_id=-2, repeated_sources=0, age_100_outliers=0;
    integer oracle_filtered=0, previous_filtered=0;
    integer count=0, value_sum=0, filtered_min=1023, filtered_max=0;
    integer contribute_min=1023, contribute_max=0, contributions=0, contribute_sum=0;
    integer outliers=0, current_run=0, longest_run=0, edge_outliers=0;
    integer first_index=65535, first_age=65535, first_motor=0;
    integer maximum_step=0, step_index=65535, step_age=65535, step_motor=0;
    integer baseline=0, delta, step_value;
    integer expected_mean=0;
    integer rounded_up_windows=0, rounded_down_windows=0, exact_mean_windows=0;
    reg [271:0] expected_full_window, expected_limited_window;
    time expected_publication=0;
    reg baseline_exists=0, step_exists=0, outlier_exists=0;
    reg [19:0] selected_metadata;
    // Independent order-statistic oracle, not the RTL's comparison network:
    // count smaller values, breaking equality by original chronological index.
    // Rank 3 is the seven-value median; its source ID owns the metadata.
    always @(posedge adc_clk) begin
        last_rise=$time;
        #81;
        if(rst_n) begin
            captures=captures+1;
            for(a=0;a<6;a=a+1) begin hvalue[a]=hvalue[a+1]; hid[a]=hid[a+1]; end
            hvalue[6]=data_in; hid[6]=data_id;
            if(data_id>=0 && (raw_code!==conversion_value[data_id] ||
                              dut.captured_tag!==conversion_metadata[data_id]))
                $fatal(1,"ADC four-cycle source tag mismatch id=%0d got=%h want=%h",
                    data_id,dut.captured_tag,conversion_metadata[data_id]);
            oracle_pending_ready=captures>=7;
            if(oracle_pending_ready) begin
                selected_slot=-1;
                for(a=0;a<7;a=a+1) begin
                    order_rank=0;
                    for(b=0;b<7;b=b+1)
                        if(hvalue[b]<hvalue[a] || (hvalue[b]==hvalue[a] && b<a)) order_rank=order_rank+1;
                    if(order_rank==3) selected_slot=a;
                end
                if(selected_slot<0) $fatal(1,"oracle has no rank-three value");
                selected_value=hvalue[selected_slot];
                selected_id=hid[selected_slot];
            end
        end
    end
    // Capture is rising+80 ns; seven registered rounds and consumption take
    // another 160 ns. Thus check the previous capture at next rising+41 ns.
    // This separate observer never misses a rising edge while waiting.
    always @(posedge adc_clk) begin
        #41;
        if(rst_n && oracle_pending_ready) begin
                if(selected_id>=0) begin
                    selected_metadata=conversion_metadata[selected_id];
                    if(dut.filtered!==selected_value || dut.median_tag!==selected_metadata)
                        $fatal(1,"median provenance mismatch selected_id=%0d value=%0d tag=%h want=%h",
                            selected_id,selected_value,dut.median_tag,selected_metadata);
                    slot_coverage[selected_slot]=slot_coverage[selected_slot]+1;
                    if(hvalue[0]==hvalue[1] && hvalue[1]==hvalue[2] && hvalue[2]==hvalue[3] &&
                       hvalue[3]==hvalue[4] && hvalue[4]==hvalue[5] && hvalue[5]==hvalue[6]) begin
                        if(selected_slot!=3) $fatal(1,"all-equal stable median must select the temporal center");
                        history_ties=history_ties+1;
                    end
                    if(selected_id==last_selected_id) repeated_sources=repeated_sources+1;
                    last_selected_id=selected_id;
                    if(captures>=63) begin
                        median_events=median_events+1;
                        if(selected_value<filtered_min) filtered_min=selected_value;
                        if(selected_value>filtered_max) filtered_max=selected_value;
                        value_sum=value_sum+selected_value;
                        step_value=selected_value-previous_filtered;
                        if(step_value<0) step_value=-step_value;
                        if(!step_exists || step_value>maximum_step) begin
                            maximum_step=step_value; step_index=count; step_age=selected_metadata[15:0];
                            step_motor=selected_metadata[19:16]; step_exists=1;
                        end
                        delta=selected_value*16-baseline;
                        if(delta<0) delta=-delta;
                        if(baseline_exists && delta>256) begin
                            if(!outlier_exists) begin
                                first_index=count; first_age=selected_metadata[15:0];
                                first_motor=selected_metadata[19:16]; outlier_exists=1;
                            end
                            outliers=outliers+1; current_run=current_run+1;
                            if(current_run>longest_run) longest_run=current_run;
                            if(selected_metadata[15:0]<=100) edge_outliers=edge_outliers+1;
                            if(selected_metadata[15:0]==100) age_100_outliers=age_100_outliers+1;
                        end else current_run=0;
                        count=count+1;
                    end
                end
                previous_filtered=selected_value; oracle_filtered=selected_value;
        end
    end
    // Diagnostic observation timing is derived from clocks after capture 63,
    // independent of the diagnostic implementation and its counters.
    always @(posedge clk) if(rst_n && captures>=63) begin
        eligible_ticks=eligible_ticks+1;
        if(eligible_ticks%SC==0) begin
            if(oracle_filtered<contribute_min) contribute_min=oracle_filtered;
            if(oracle_filtered>contribute_max) contribute_max=oracle_filtered;
            contributions=contributions+1;
            contribute_sum=contribute_sum+oracle_filtered;
        end
        if(eligible_ticks%(16*SC)==0) begin
            expected_publication=$time+320;
            if($time-last_rise==40) same_edge_windows=same_edge_windows+1;
            // The independent median observer updates at edge+1 ns. Include
            // that event before closing a coincident source window.
            #2;
            if(contributions!=16 || (count!=20 && count!=21))
                $fatal(1,"independent source window missed observations or median outputs");
            expected_mean=(value_sum*16+count/2)/count;
            if((value_sum*16)%count==0) exact_mean_windows=exact_mean_windows+1;
            else if(expected_mean>(value_sum*16)/count) rounded_up_windows=rounded_up_windows+1;
            else rounded_down_windows=rounded_down_windows+1;
            build_detail(0,expected_full_window); build_detail(1,expected_limited_window);
            if(outliers) outlier_windows=outlier_windows+1;
            if(longest_run>7) long_run_windows=long_run_windows+1;
            if(edge_outliers) near_windows=near_windows+1;
            if(outliers>edge_outliers) far_windows=far_windows+1;
            if(step_age==65535 && motor_edge_known) saturated_age_windows=saturated_age_windows+1;
            baseline=expected_mean; baseline_exists=1;
            count=0; value_sum=0; filtered_min=1023; filtered_max=0;
            contribute_min=1023; contribute_max=0; contributions=0; contribute_sum=0;
            outliers=0; current_run=0; longest_run=0; edge_outliers=0;
            first_index=65535; first_age=65535; first_motor=0;
            maximum_step=0; step_index=65535; step_age=65535; step_motor=0;
            step_exists=0; outlier_exists=0;
        end
    end
    function integer clamp7(input integer value);
        begin clamp7=value>7 ? 7 : value; end
    endfunction
    reg [15:0] expected_words[0:16];
    reg [271:0] expected;
    integer n, windows=0, same_edge_windows=0, outlier_windows=0, long_run_windows=0;
    integer near_windows=0, far_windows=0, saturated_age_windows=0;
    task build_detail(input bit small_limits,output reg [271:0] result);
        reg [15:0] flags;
        integer final_sum;
        begin
            flags=(baseline_exists ? 1 : 0) | (step_exists ? 4 : 0) | (outlier_exists ? 8 : 0) |
                  (first_motor<<4) | (step_motor<<8);
            if(small_limits && (count>7 || value_sum>4095 || outliers>7 || longest_run>7 || edge_outliers>7)) flags=flags|2;
            final_sum=small_limits && value_sum>4095 ? 4095 : value_sum;
            expected_words[0]=baseline; expected_words[1]=filtered_min; expected_words[2]=filtered_max;
            expected_words[3]=contribute_min; expected_words[4]=contribute_max; expected_words[5]=maximum_step;
            expected_words[6]=small_limits ? clamp7(outliers) : outliers;
            expected_words[7]=small_limits ? clamp7(longest_run) : longest_run;
            expected_words[8]=small_limits && outlier_exists ? clamp7(first_index) : first_index;
            expected_words[9]=first_age;
            expected_words[10]=small_limits && step_exists ? clamp7(step_index) : step_index;
            expected_words[11]=step_age;
            expected_words[12]=small_limits ? clamp7(edge_outliers) : edge_outliers;
            expected_words[13]=small_limits ? clamp7(count) : count;
            expected_words[14]=flags; expected_words[15]=final_sum & 65535; expected_words[16]=final_sum>>16;
            for(n=0;n<17;n=n+1) expected[16*n+:16]=expected_words[n];
            result=expected;
        end
    endtask
    always @(posedge valid) begin
        if($time!=expected_publication) $fatal(1,"public window was not delayed exactly sixteen system clocks");
        #2;
        if(mean_q4!==expected_mean) $fatal(1,"independent all-median rounded Q4 mean mismatch got%0d want%0d",mean_q4,expected_mean);
        if(detail!==expected_full_window || limited_detail!==expected_limited_window) begin
            for(n=0;n<17;n=n+1) begin
                if(detail[16*n+:16]!==expected_full_window[16*n+:16])
                    $display("full word%0d got%h want%h",n,detail[16*n+:16],expected_full_window[16*n+:16]);
                if(limited_detail[16*n+:16]!==expected_limited_window[16*n+:16])
                    $display("limited word%0d got%h want%h",n,limited_detail[16*n+:16],expected_limited_window[16*n+:16]);
            end
            $fatal(1,"independent delayed-window transaction scoreboard: window=%0d",windows);
        end
        windows=windows+1;
    end
    task source_for(input integer value,input integer periods);
        begin @(posedge adc_clk); #1; source_code=value; repeat(periods) @(posedge adc_clk); #1; end
    endtask
    reg [31:0] random_state=32'h72a50c19;
    integer i;
    initial begin #5000000; $fatal(1,"tb_adc_observability timeout"); end
    initial begin
        for(i=0;i<4;i=i+1) data_ids[i]=-1;
        for(i=0;i<7;i=i+1) begin hvalue[i]=0; hid[i]=-1; slot_coverage[i]=0; end
        repeat(3) @(negedge clk); rst_n=1;
        repeat(4) @(posedge valid); #3;
        if(detail[16*13+:16]<20 || detail[16*6+:16]!=0 || detail[16*9+:16]!=16'hffff)
            $fatal(1,"static baseline/unknown motor-edge statistics");
        // Isolated, short and long bursts; exact +/-16-code diagnostic boundary.
        motor_next=9;
        source_for(640,1); source_for(600,25);
        source_for(616,4); source_for(600,25);
        source_for(617,4); source_for(600,25);
        motor_next=8;
        source_for(640,20); source_for(600,40);
        // Align a motor transition with an ADC falling edge. A 16-conversion
        // burst contains a selected outlier exactly 100 ticks after that edge,
        // checking the inclusive 2 us diagnostic boundary and repeated sources.
        repeat(3) @(posedge valid); #3;
        @(posedge adc_clk); #89; motor_next=14; source_code=640;
        repeat(16) @(posedge adc_clk); #1; source_code=600;
        repeat(3) @(posedge valid); #3;
        // Directed median provenance and stable equal-value ties.
        motor_next=13;
        source_for(620,1); source_for(640,1); source_for(600,1);
        source_for(600,2); source_for(700,1); source_for(600,30);
        // Deterministic random values, motor changes and OTR, checked against
        // independent raw transaction IDs and the rank-statistic oracle.
        for(i=0;i<800;i=i+1) begin
            @(posedge adc_clk); #1;
            random_state={random_state[30:0],random_state[31]^random_state[21]^random_state[1]^random_state[0]};
            source_code=random_state[9:0]; source_otr=random_state[12] && random_state[19];
            if(i%7==0) motor_next=random_state[23:20];
        end
        source_otr=0; source_code=600; motor_next=9;
        // Long static dwell checks real 16-bit age saturation and production
        // 5000-value count/sum; diagnostic small limits never affect control.
        repeat(3) @(posedge production_valid); #3;
        if(production_detail[16*13+:16]!=5000 || production_detail[271:240]!=3000000 ||
           production_detail[16*1+:16]!=600 || production_detail[16*2+:16]!=600 ||
           production_detail[16*6+:16]!=0 || production_detail[16*14+1])
            $fatal(1,"production full-stream count/sum/static statistics");
        for(i=0;i<7;i=i+1)
            if(!slot_coverage[i]) $fatal(1,"median source slot %0d was never selected",i);
        if(windows<20 || !same_edge_windows || !history_ties ||
           !outlier_windows || !long_run_windows || !near_windows || !far_windows || !saturated_age_windows ||
           !repeated_sources || !age_100_outliers || !rounded_up_windows || !rounded_down_windows || !exact_mean_windows)
            $fatal(1,"observability coverage incomplete windows=%0d boundary=%0d ties=%0d outlier=%0d long=%0d near=%0d far=%0d age=%0d repeated=%0d age100=%0d",
                windows,same_edge_windows,history_ties,
                outlier_windows,long_run_windows,near_windows,far_windows,saturated_age_windows,repeated_sources,age_100_outliers);
        $display("PASS tb_adc_observability: external four-cycle ADC/tags, independent median7/all-stream rounded Q4, up/down/exact rounding %0d/%0d/%0d, all provenance slots/ties, 16-clock ownership, full detail, saturation, production sum/count", rounded_up_windows,rounded_down_windows,exact_mean_windows);
        $finish;
    end
endmodule
