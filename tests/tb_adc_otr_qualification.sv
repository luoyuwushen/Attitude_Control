`timescale 1ns/1ps
module tb_adc_otr_qualification;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0;
    reg [9:0] source=600, data_in=600, sparse_rail=0;
    reg source_otr=0, otr=0, sparse_mode=0;
    integer conversion=0;
    wire adc_clk, valid, bad, blind, fault;
    wire [13:0] raw_mean, control_mean;
    wire [7:0] reason;
    adc_sampler #(.SAMPLE_CYCLES(125)) dut(
        .clk(clk),.rst_n(rst_n),.data_in(data_in),.otr(otr),.motor_observe(4'd0),
        .adc_clk(adc_clk),.valid(valid),.sample_q4(raw_mean),.control_q4(control_mean),
        .sample_bad(bad),.sample_blind(blind),.sample_fault(fault),.quality_reason(reason));
    always @(negedge adc_clk) if(rst_n) begin
        conversion=conversion+1;
        #25; data_in=(sparse_mode && conversion%32==0) ? sparse_rail : source;
        otr=source_otr;
    end
    task publish;
        begin @(posedge valid); #2; end
    endtask
    // guard=1: no hard fault; guard=2: require a hard fault; guard=3:
    // require a confirmed blind window and never classify it as hard fault.
    integer guard=0;
    reg case_marker=0;
    reg saw_fault=0, saw_blind=0, saw_otr=0;
    wire [3:0] phase_saw_fault, phase_saw_blind;
    wire [9:0] phase_cuts [0:3];
    genvar p;
    generate for(p=0;p<4;p=p+1) begin: phase_check
        wire phase_valid, phase_bad, phase_blind, phase_fault;
        reg seen_fault=0, seen_blind=0, last_marker=0;
        reg [9:0] cuts=0;
        // The 16*SAMPLE_CYCLES window lengths are non-integral ADC periods.
        // Each instance therefore visits all reachable cuts (0/2/4/6/8),
        // including a median event simultaneous with the window closing edge.
        adc_sampler #(.SAMPLE_CYCLES(121+p)) phase_dut(
            .clk(clk),.rst_n(rst_n),.data_in(data_in),.otr(otr),.motor_observe(4'd0),
            .valid(phase_valid),.sample_bad(phase_bad),.sample_blind(phase_blind),
            .sample_fault(phase_fault));
        assign phase_saw_fault[p]=seen_fault;
        assign phase_saw_blind[p]=seen_blind;
        assign phase_cuts[p]=cuts;
        always @(posedge clk) begin
            if(rst_n && phase_dut.window_close) cuts[phase_dut.phase]<=1;
            if(last_marker!=case_marker) begin
                last_marker<=case_marker; seen_fault<=0; seen_blind<=0;
            end else if(phase_valid && guard!=0) begin
                if(phase_fault) seen_fault<=1;
                if(phase_blind) seen_blind<=1;
            end
        end
        always @(posedge phase_valid) begin
            #1;
            if((guard==1 || guard==3) && phase_fault)
                $fatal(1,"OTR/rail qualification split across window phase, SAMPLE_CYCLES=%0d",121+p);
            if(phase_fault && (!phase_bad || phase_blind))
                $fatal(1,"fractional-window hard OTR / bad / blind contract");
        end
    end endgenerate
    always @(posedge valid) begin
        #1;
        if(guard!=0) begin
            if(fault) saw_fault=1;
            if(blind) saw_blind=1;
            if(reason[0]) saw_otr=1;
            if((guard==1 || guard==3) && fault)
                $fatal(1,"isolated OTR or confirmed blind pulse became a hard fault");
            if(fault && (!bad || blind)) $fatal(1,"hard OTR / bad / blind contract");
        end
    end
    task pulse(input integer code,input integer length,input integer offset,input integer expectation);
        begin
            guard=expectation; case_marker=!case_marker; saw_fault=0; saw_blind=0; saw_otr=0;
            wait(dut.window_close); @(posedge clk); #1;
            repeat(offset) @(posedge adc_clk);
            #1; source=code; source_otr=1;
            repeat(length) @(posedge adc_clk);
            #1; source=600; source_otr=0;
            repeat(3) publish;
            guard=0;
            if(!saw_otr || (expectation==2 && !saw_fault) || (expectation==3 && !saw_blind))
                $fatal(1,"qualification/ownership missing: code=%0d length=%0d offset=%0d fault=%b blind=%b otr=%b",
                    code,length,offset,saw_fault,saw_blind,saw_otr);
            if((expectation==2 && phase_saw_fault!=4'hf) ||
                (expectation==3 && phase_saw_blind!=4'hf))
                $fatal(1,"fractional-window qualification missing code=%0d offset=%0d fault=%b blind=%b",
                    code,offset,phase_saw_fault,phase_saw_blind);
        end
    endtask
    integer rail_case, n, offset;
    initial begin #20000000; $fatal(1,"OTR qualification timeout"); end
    initial begin
        repeat(3) @(negedge clk); rst_n=1;
        repeat(4) publish;
        // Regression for a continuous hardware OTR hidden by one raw rail
        // every 32 conversions. Median7 and both means stay exactly at 600;
        // neither raw endpoint may reset the OTR qualification interval.
        for(rail_case=0;rail_case<2;rail_case=rail_case+1) begin
            @(posedge adc_clk); #1;
            sparse_rail=rail_case==0 ? 0 : 1023; sparse_mode=1; source_otr=1;
            repeat(2) publish;
            for(n=0;n<12;n=n+1) begin
                publish;
                if(!fault || !bad || blind || raw_mean!=9600 || control_mean!=9600 || (reason&3)!=3)
                    $fatal(1,"sparse raw rail hid sustained OTR: rail=%0d window=%0d fault=%b bad=%b blind=%b",
                        sparse_rail,n,fault,bad,blind);
            end
            @(posedge adc_clk); #1; sparse_mode=0; source_otr=0;
            repeat(3) publish;
            if(fault || bad || blind) $fatal(1,"OTR recovery did not clear window flags");
        end
        // One-conversion OTR and 63-conversion OTR are not qualified faults.
        // Sweep the 64th observation across the cut for midscale and both
        // endpoints: a rail's OTR and median qualification must share ownership.
        pulse(600,1,199,1);
        pulse(600,63,137,1);
        for(n=0;n<12;n=n+1) begin
            case(n)
                0: offset=0;
                1: offset=130;
                2: offset=135;
                3: offset=136;
                4: offset=137;
                5: offset=138;
                6: offset=139;
                7: offset=140;
                8: offset=196;
                9: offset=197;
                10: offset=198;
                default: offset=199;
            endcase
            pulse(600,64,offset,2);
            pulse(0,64,offset,3);
            pulse(1023,64,offset,3);
        end
        if(phase_cuts[0]!=10'h155 || phase_cuts[1]!=10'h155 ||
            phase_cuts[2]!=10'h155 || phase_cuts[3]!=10'h155)
            $fatal(1,"not all reachable window-cut phases were covered");
        $display("PASS tb_adc_otr_qualification: continuous OTR with sparse low/high rails, isolated OTR, exact 64 qualification, both blind rails and all five reachable window-cut phases");
        $finish;
    end
endmodule
