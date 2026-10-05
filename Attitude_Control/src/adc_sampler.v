`timescale 1ns/1ps
`default_nettype none
// 3PA1030 Fig. 22: conversion/data launch is referenced to the falling edge.
// At 5 MHz, capture 80 ns after rising = 180 ns after the previous falling edge.
// Capture the parallel bus together; individual bit synchronizers are unsuitable.
module adc_sampler #(
    parameter integer SAMPLE_CYCLES = 3125,
    // Engineering limits, not fitted motor/ADC measurements: 64 conversions
    // are 12.8 us at 5 MHz. Normal changes of <=32 codes have no added delay.
    parameter integer SPIKE_CONFIRM_CONVERSIONS = 64,
    parameter integer SPIKE_STEP_CODES = 32,
    // Smaller limits are used only by saturation tests; no control path uses them.
    parameter [15:0] DIAG_COUNT_MAX = 16'hffff,
    parameter [31:0] DIAG_SUM_MAX = 32'hffffffff
)(
    input wire clk, rst_n,
    input wire [9:0] data_in,
    input wire otr,
    input wire [3:0] motor_observe,
    output reg adc_clk,
    output wire adc_oe_n,
    output reg [13:0] sample_q4,
    output reg [13:0] control_q4,
    output reg valid,
    output reg over_range,
    output wire [9:0] raw_code,
    output reg [9:0] window_min,
    output reg [9:0] window_max,
    output reg sample_bad,
    output reg sample_otr,
    output reg sample_blind,
    output reg sample_fault,
    output reg spike_rejected,
    // [0] raw OTR, [1] raw rail, [2] block-mean window span >32,
    // [3] adjacent block-mean step >32, [4] incomplete acquisition,
    // [5] raw span warning only, [6] within-block variance >16^2; [7] reserved.
    output reg [7:0] quality_reason,
    output reg [271:0] window_detail
);
    reg [3:0] phase;
    reg [5:0] warmup;
    reg [9:0] captured;
    reg [31:0] interval_count;
    reg [3:0] sample_count;
    reg [9:0] pending_min, pending_max;
    reg pending_seen, pending_otr, pending_rail, pending_unready, capture_pending;
    reg [9:0] filtered;
    reg filtered_ready;
    reg [9:0] contribution_min, contribution_max;
    reg [9:0] median_min, median_max;
    reg median_seen;
    // Only captured/over_range touch the external bus. Quality logic consumes
    // that coherent pair one system clock later, within the synchronous domain.
    wire [9:0] next_min = capture_pending && (!pending_seen || captured < pending_min) ? captured : pending_min;
    wire [9:0] next_max = capture_pending && (!pending_seen || captured > pending_max) ? captured : pending_max;
    wire next_otr = pending_otr || (capture_pending && over_range);
    wire next_seen = pending_seen || capture_pending;
    wire next_rail = pending_rail || (capture_pending && (captured == 0 || captured == 1023));
    // Seven registered odd/even sorting rounds fit inside the ten system
    // clocks per conversion. A seven-point median rejects at most three
    // outliers per seven-point window; sustained changes remain observable.
    // Preserve chronological order for equal values (strict > swaps only):
    // all-equal input selects the fourth-oldest conversion and its exact tag.
    reg [29:0] filter_history [0:5]; // {raw value, conversion metadata}
    reg [2:0] history_count;
    wire [29:0] filter_input [0:6];
    reg [29:0] sort_data [0:48];
    reg [6:0] sort_valid;
    wire median_pending = sort_valid[6];
    wire [9:0] median_value = sort_data[45][29:20];
    wire [19:0] median_tag = sort_data[45][19:0];
    wire [9:0] median_step = median_value > filtered ? median_value-filtered : filtered-median_value;
    wire [9:0] contribution_next_min = sample_count == 0 || filtered < contribution_min ? filtered : contribution_min;
    wire [9:0] contribution_next_max = sample_count == 0 || filtered > contribution_max ? filtered : contribution_max;
    // Keep full-rate diagnostic extremes alongside the sixteen diagnostic
    // observation ticks. Control averaging below also includes every median.
    wire [9:0] median_next_min = median_pending && (!median_seen || median_value < median_min) ? median_value : median_min;
    wire [9:0] median_next_max = median_pending && (!median_seen || median_value > median_max) ? median_value : median_max;
    wire median_next_seen = median_seen || median_pending;
    wire window_close = warmup == 63 && interval_count == SAMPLE_CYCLES-1 && sample_count == 15;
    wire median_event = warmup == 63 && median_pending;
    assign raw_code = captured;
    assign adc_oe_n = 1'b0; // STBY is grounded on the board, no FPGA port.

    // Keep the original median stream and its diagnostics intact. Only the
    // control branch qualifies large excursions. A short excursion is filled
    // with the last trusted value; a coherent sustained change is admitted
    // after a bounded confirmation interval (there is no 1 ms frame delay).
    reg [9:0] control_value, candidate_value;
    reg control_ready;
    reg [15:0] candidate_count, rail_count, otr_count;
    reg [2:0] otr_history;
    reg [6:0] otr_median_pipe;
    wire [9:0] control_delta = median_value > control_value ?
        median_value-control_value : control_value-median_value;
    wire [9:0] candidate_delta = median_value > candidate_value ?
        median_value-candidate_value : candidate_value-median_value;
    wire control_excursion = control_ready && control_delta > SPIKE_STEP_CODES;
    wire candidate_coherent = candidate_count != 0 && candidate_delta <= SPIKE_STEP_CODES;
    wire confirm_excursion = control_excursion && candidate_coherent &&
        candidate_count >= SPIKE_CONFIRM_CONVERSIONS-1;
    wire reject_excursion = control_excursion && !confirm_excursion;
    wire [9:0] next_control_value = reject_excursion ? control_value : median_value;
    wire median_rail = median_value <= 1 || median_value >= 1022;
    wire blind_conversion = median_rail && rail_count >= SPIKE_CONFIRM_CONVERSIONS-1;
    reg control_window_blind, control_window_fault, control_window_spike;
    reg [13:0] control_rejected_count;
    reg [9:0] control_min, control_max;
    wire [9:0] next_control_min = !median_event ? control_min :
        control_count == 0 || next_control_value < control_min ? next_control_value : control_min;
    wire [9:0] next_control_max = !median_event ? control_max :
        control_count == 0 || next_control_value > control_max ? next_control_value : control_max;
    wire next_window_blind = control_window_blind || (median_event && blind_conversion);
    wire next_window_fault = control_window_fault ||
        (median_event && otr_median_pipe[6] && otr_count >= SPIKE_CONFIRM_CONVERSIONS-1);
    wire [9:0] raw_control_delta = captured > control_value ?
        captured-control_value : control_value-captured;
    wire next_window_spike = control_window_spike || (median_event && reject_excursion) ||
        (capture_pending && control_ready && raw_control_delta > SPIKE_STEP_CODES);
    wire [13:0] next_rejected_count = control_rejected_count+
        ((median_event && reject_excursion) ? 14'd1 : 14'd0);
    reg publish_control_bad, publish_control_blind, publish_control_fault, publish_control_spike;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            control_value<=0; candidate_value<=0; control_ready<=0;
            candidate_count<=0; rail_count<=0; otr_count<=0;
            otr_history<=0; otr_median_pipe<=0;
            control_window_blind<=0; control_window_fault<=0; control_window_spike<=0;
            control_rejected_count<=0; control_min<=0; control_max<=0;
            publish_control_bad<=1; publish_control_blind<=0;
            publish_control_fault<=0; publish_control_spike<=0;
        end else begin
            // OTR continuity is independent of raw code: isolated raw rails
            // must not repeatedly reset an otherwise sustained hardware OTR.
            // Align its chronological middle conversion with median7's three
            // conversion lookback and seven-clock sorter. At a rail entry the
            // two 64-conversion qualifications then belong to the same window;
            // only a confirmed median blind window can exempt the OTR fault.
            if (capture_pending) otr_history<={otr_history[1:0],over_range};
            otr_median_pipe<={otr_median_pipe[5:0],capture_pending && otr_history[2]};
            if (median_event) begin
                if (!otr_median_pipe[6]) otr_count<=0;
                else if (otr_count < SPIKE_CONFIRM_CONVERSIONS) otr_count<=otr_count+1'b1;
                control_ready<=1; control_value<=next_control_value;
                if (!control_excursion || confirm_excursion) candidate_count<=0;
                else begin
                    candidate_value<=median_value;
                    if (candidate_coherent) candidate_count<=candidate_count+1'b1;
                    else candidate_count<=1;
                end
                if (!median_rail) rail_count<=0;
                else if (rail_count < SPIKE_CONFIRM_CONVERSIONS) rail_count<=rail_count+1'b1;
                control_min<=next_control_min; control_max<=next_control_max;
            end
            control_window_blind<=next_window_blind;
            control_window_fault<=next_window_fault;
            control_window_spike<=next_window_spike;
            control_rejected_count<=next_rejected_count;
            if (window_close) begin
                // A rail/endpoint mixture is not a measurable angle. Preserve
                // its raw evidence but never publish its plausible middle mean
                // as a trusted control measurement. Half or more of a window of
                // replacement values is also not a fresh measured state.
                publish_control_blind<=next_window_blind;
                publish_control_fault<=next_window_fault && !next_window_blind;
                publish_control_spike<=next_window_spike;
                publish_control_bad<=incomplete_window || next_window_blind ||
                    next_window_fault || next_control_max-next_control_min > SPIKE_STEP_CODES ||
                    (next_rejected_count != 0 && {next_rejected_count,1'b0} >= {1'b0,control_next_count});
                control_window_blind<=0; control_window_fault<=0; control_window_spike<=0;
                control_rejected_count<=0;
            end
        end
    end

    // Diagnostic tags never affect capture, filtering, quality or control.
    // Observe the generated falling edge on the next 50 MHz clock (+20 ns),
    // including simultaneous motor-output changes. This is a digital timing
    // observation, not a zero-error measurement of the analog sampling instant.
    reg [3:0] observed_motor;
    reg motor_seen, falling_pending;
    reg [15:0] motor_edge_age;
    wire [15:0] observed_edge_age = !motor_seen ? 16'hffff :
        motor_observe != observed_motor ? 16'd0 :
        motor_edge_age == 16'hffff ? 16'hffff : motor_edge_age+16'd1;
    reg [19:0] conversion_tag [0:4];
    reg [19:0] captured_tag;
    integer tag_index;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            observed_motor <= 0; motor_seen <= 0; motor_edge_age <= 16'hffff;
            falling_pending <= 0;
            for (tag_index=0; tag_index<5; tag_index=tag_index+1)
                conversion_tag[tag_index] <= 20'h0ffff;
            captured_tag <= 20'h0ffff;
        end else begin
            observed_motor <= motor_observe; motor_seen <= 1;
            motor_edge_age <= observed_edge_age;
            falling_pending <= phase == 5;
            if (falling_pending) begin
                // Fig.22: output at falling edge N belongs to conversion N-4.
                // After inserting N, entry 4 accompanies the next bus capture.
                conversion_tag[0] <= {motor_observe,observed_edge_age};
                for (tag_index=1; tag_index<5; tag_index=tag_index+1)
                    conversion_tag[tag_index] <= conversion_tag[tag_index-1];
            end
            if (phase == 4) captured_tag <= conversion_tag[4];
        end
    end

    integer history_index;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            history_count <= 0;
            for (history_index=0; history_index<6; history_index=history_index+1)
                filter_history[history_index] <= 30'h0000ffff;
        end else if (capture_pending) begin
            filter_history[0] <= {captured,captured_tag};
            for (history_index=1; history_index<6; history_index=history_index+1)
                filter_history[history_index] <= filter_history[history_index-1];
            if (history_count < 6) history_count <= history_count+1'b1;
        end
    end
    assign filter_input[6] = {captured,captured_tag};
    genvar history_slot, round, pair;
    generate
        for (history_slot=0; history_slot<6; history_slot=history_slot+1) begin: g_history
            assign filter_input[history_slot] = filter_history[5-history_slot];
        end
        for (round=0; round<7; round=round+1) begin: g_sort
            wire load;
            wire [29:0] input_value [0:6];
            if (round == 0) begin: g_first
                assign load = capture_pending && history_count == 6;
                for (history_slot=0; history_slot<7; history_slot=history_slot+1) begin: g_input
                    assign input_value[history_slot] = filter_input[history_slot];
                end
            end else begin: g_later
                assign load = sort_valid[round-1];
                for (history_slot=0; history_slot<7; history_slot=history_slot+1) begin: g_input
                    assign input_value[history_slot] = sort_data[(round-1)*7+history_slot];
                end
            end
            always @(posedge clk or negedge rst_n)
                if (!rst_n) sort_valid[round] <= 0;
                else sort_valid[round] <= load;
            for (pair=0; pair<3; pair=pair+1) begin: g_pair
                localparam integer LEFT = pair*2+(round%2);
                always @(posedge clk or negedge rst_n) begin
                    if (!rst_n) begin
                        sort_data[round*7+LEFT] <= 30'h0000ffff;
                        sort_data[round*7+LEFT+1] <= 30'h0000ffff;
                    end else if (load) begin
                        if (input_value[LEFT][29:20] > input_value[LEFT+1][29:20]) begin
                            sort_data[round*7+LEFT] <= input_value[LEFT+1];
                            sort_data[round*7+LEFT+1] <= input_value[LEFT];
                        end else begin
                            sort_data[round*7+LEFT] <= input_value[LEFT];
                            sort_data[round*7+LEFT+1] <= input_value[LEFT+1];
                        end
                    end
                end
            end
            localparam integer UNPAIRED = round%2 == 0 ? 6 : 0;
            always @(posedge clk or negedge rst_n)
                if (!rst_n) sort_data[round*7+UNPAIRED] <= 30'h0000ffff;
                else if (load) sort_data[round*7+UNPAIRED] <= input_value[UNPAIRED];
        end
    endgenerate

    // Control uses every completed median, independently of diagnostic limits.
    // Production: 5000 conversions/ms, quality blocks of 40 (8 us). For a
    // scaled integral window use gcd(40,N); a fractional 20/21-style window
    // uses singleton blocks for phase/ownership tests, not variance testing.
    function integer greatest_common_divisor;
        input integer a, b;
        integer temporary;
        begin
            while (b != 0) begin temporary=a%b; a=b; b=temporary; end
            greatest_common_divisor=a;
        end
    endfunction
    localparam integer WINDOW_CLOCKS = 16*SAMPLE_CYCLES;
    localparam integer MIN_CONVERSIONS = WINDOW_CLOCKS/10;
    localparam integer MAX_CONVERSIONS = (WINDOW_CLOCKS+9)/10;
    localparam integer QUALITY_BLOCK = WINDOW_CLOCKS%10 == 0 ?
        greatest_common_divisor(40,MIN_CONVERSIONS) : 1;
    localparam [31:0] VARIANCE_LIMIT = 256*QUALITY_BLOCK*QUALITY_BLOCK;
    localparam [15:0] BLOCK_STEP_LIMIT = 32*QUALITY_BLOCK;
    // synthesis translate_off
    initial begin
        if (SAMPLE_CYCLES < 5 || MAX_CONVERSIONS > 8200)
            $fatal(1,"adc_sampler SAMPLE_CYCLES must be 5..5125");
        if (SPIKE_CONFIRM_CONVERSIONS < 2 || SPIKE_CONFIRM_CONVERSIONS > 32767 ||
            SPIKE_STEP_CODES < 1 || SPIKE_STEP_CODES > 511)
            $fatal(1,"adc_sampler spike qualification parameters out of range");
    end
    // synthesis translate_on

    reg window_epoch;
    reg [22:0] control_sum;
    reg [22:0] qualified_sum;
    reg [13:0] control_count;
    wire [23:0] control_next_sum = {1'b0,control_sum}+
        (median_event ? {14'd0,median_value} : 24'd0);
    wire [13:0] control_next_count = control_count+(median_event ? 14'd1 : 14'd0);
    wire [23:0] qualified_next_sum = {1'b0,qualified_sum}+
        (median_event ? {14'd0,next_control_value} : 24'd0);
    // Integral windows are divisible by the gcd block; fractional windows
    // use B=1. The count bounds therefore also prove there is no block tail.
    wire incomplete_window = control_next_sum[23] || control_next_count < MIN_CONVERSIONS ||
        control_next_count > MAX_CONVERSIONS;
    reg [31:0] divide_remainder, divide_denominator;
    reg [13:0] divide_quotient, completed_mean;
    reg [31:0] qualified_remainder;
    reg [13:0] qualified_quotient, completed_control_mean;
    reg [3:0] divide_remaining;
    reg divide_busy;
    wire divide_bit = divide_remainder >= divide_denominator;
    wire [13:0] divide_next_quotient = {divide_quotient[12:0],divide_bit};
    wire qualified_bit = qualified_remainder >= divide_denominator;
    wire [13:0] qualified_next_quotient = {qualified_quotient[12:0],qualified_bit};
    reg pending_epoch;
    reg [9:0] publish_raw_min, publish_raw_max;
    reg [9:0] publish_median_min, publish_median_max;
    reg [9:0] publish_observation_min, publish_observation_max;
    reg [7:0] publish_base_reason;

    // Tags retain conversion provenance. Delay only diagnostic processing and
    // the window marker, so the preceding mean is ready before the first
    // event of the next diagnostic window. Public outputs share this marker.
    reg [15:0] diagnostic_valid_pipe, window_close_pipe;
    reg [40:0] diagnostic_data_pipe [0:15];
    wire publication_tick = window_close_pipe[15];
    wire diagnostic_event = diagnostic_valid_pipe[15];
    wire [9:0] diagnostic_value = diagnostic_data_pipe[15][9:0];
    wire [19:0] diagnostic_tag = diagnostic_data_pipe[15][29:10];
    wire [9:0] diagnostic_step = diagnostic_data_pipe[15][39:30];
    wire diagnostic_ready = diagnostic_data_pipe[15][40];
    integer delay_index;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            diagnostic_valid_pipe <= 0; window_close_pipe <= 0;
            for (delay_index=0; delay_index<16; delay_index=delay_index+1)
                diagnostic_data_pipe[delay_index] <= 0;
        end else begin
            diagnostic_valid_pipe <= {diagnostic_valid_pipe[14:0],median_event};
            window_close_pipe <= {window_close_pipe[14:0],window_close};
            diagnostic_data_pipe[0] <= {filtered_ready,median_step,median_tag,median_value};
            for (delay_index=1; delay_index<16; delay_index=delay_index+1)
                diagnostic_data_pipe[delay_index] <= diagnostic_data_pipe[delay_index-1];
        end
    end

    // Block moments and comparisons are registered separately. No truncation
    // to an integer mean occurs before span/step or variance comparisons:
    // Var = (B*sum(x*x)-sum(x)^2)/B^2; reject only Var > 256.
    reg quality_v1, quality_v2, quality_v3, quality_v4, quality_close1;
    reg quality_epoch1, quality_epoch2, quality_epoch3, quality_epoch4;
    reg [9:0] quality_value1;
    reg [19:0] quality_square1;
    reg [5:0] block_count;
    reg [15:0] block_sum, block_sum2, block_sum3;
    reg [25:0] block_square_sum, block_square_sum2;
    wire [15:0] block_next_sum = block_sum+{6'd0,quality_value1};
    wire [25:0] block_next_square_sum = block_square_sum+{6'd0,quality_square1};
    reg [31:0] block_sum_squared3, block_scaled_squares3, variance_numerator4;
    reg block_span4, block_step4;
    reg [15:0] block_min [0:1];
    reg [15:0] block_max [0:1];
    reg [1:0] block_seen;
    reg [2:0] block_faults [0:1]; // {variance,step,span}, owned by window epoch
    reg previous_block_seen;
    reg [15:0] previous_block_sum;
    wire [15:0] block_next_min = !block_seen[quality_epoch3] || block_sum3 < block_min[quality_epoch3] ?
        block_sum3 : block_min[quality_epoch3];
    wire [15:0] block_next_max = !block_seen[quality_epoch3] || block_sum3 > block_max[quality_epoch3] ?
        block_sum3 : block_max[quality_epoch3];
    wire [15:0] block_difference = block_sum3 > previous_block_sum ?
        block_sum3-previous_block_sum : previous_block_sum-block_sum3;
    integer quality_bank;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            quality_v1<=0; quality_v2<=0; quality_v3<=0; quality_v4<=0; quality_close1<=0;
            quality_epoch1<=0; quality_epoch2<=0; quality_epoch3<=0; quality_epoch4<=0;
            quality_value1<=0; quality_square1<=0; block_count<=0;
            block_sum<=0; block_sum2<=0; block_sum3<=0;
            block_square_sum<=0; block_square_sum2<=0;
            block_sum_squared3<=0; block_scaled_squares3<=0; variance_numerator4<=0;
            block_span4<=0; block_step4<=0; block_seen<=0;
            previous_block_seen<=0; previous_block_sum<=0;
            for (quality_bank=0; quality_bank<2; quality_bank=quality_bank+1) begin
                block_min[quality_bank]<=0; block_max[quality_bank]<=0; block_faults[quality_bank]<=0;
            end
        end else begin
            quality_v1<=median_event; quality_v2<=0; quality_v3<=quality_v2; quality_v4<=quality_v3;
            quality_close1<=window_close;
            if (median_event) begin
                quality_value1<=median_value; quality_square1<=median_value*median_value;
                quality_epoch1<=window_epoch;
            end
            if (quality_v1) begin
                if (block_count == QUALITY_BLOCK-1) begin
                    block_sum2<=block_next_sum; block_square_sum2<=block_next_square_sum;
                    quality_epoch2<=quality_epoch1; quality_v2<=1;
                    block_sum<=0; block_square_sum<=0; block_count<=0;
                end else begin
                    block_sum<=block_next_sum; block_square_sum<=block_next_square_sum;
                    block_count<=block_count+1'b1;
                end
            end
            // Retire a possible same-edge last conversion before realigning.
            // A missing-event window is rejected by its independent count;
            // its partial block must not spill into the following window.
            if (quality_close1) begin block_sum<=0; block_square_sum<=0; block_count<=0; end
            if (quality_v2) begin
                block_sum_squared3<={16'd0,block_sum2}*{16'd0,block_sum2};
                block_scaled_squares3<={6'd0,block_square_sum2}*QUALITY_BLOCK;
                block_sum3<=block_sum2; quality_epoch3<=quality_epoch2;
            end
            if (quality_v3) begin
                variance_numerator4<=block_scaled_squares3-block_sum_squared3;
                block_span4<=block_next_max-block_next_min > BLOCK_STEP_LIMIT;
                block_step4<=previous_block_seen && block_difference > BLOCK_STEP_LIMIT;
                block_min[quality_epoch3]<=block_next_min; block_max[quality_epoch3]<=block_next_max;
                block_seen[quality_epoch3]<=1;
                // Preserve across windows: a whole-window DC step is real.
                previous_block_sum<=block_sum3; previous_block_seen<=1;
                quality_epoch4<=quality_epoch3;
            end
            if (quality_v4)
                block_faults[quality_epoch4]<=block_faults[quality_epoch4] |
                    {variance_numerator4 > VARIANCE_LIMIT,block_step4,block_span4};
            if (window_close) begin
                // Reuse only the bank from two windows ago. In-flight results
                // retain the closing epoch, even on a same-clock last median.
                block_seen[!window_epoch]<=0; block_faults[!window_epoch]<=0;
            end
        end
    end
    wire [7:0] completed_reason = publish_base_reason |
        {1'b0,block_faults[pending_epoch][2],2'b00,block_faults[pending_epoch][1:0],2'b00} |
        (divide_busy ? 8'h10 : 8'h00);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            window_epoch<=0; pending_epoch<=0; control_sum<=0; control_count<=0; qualified_sum<=0;
            divide_remainder<=0; divide_denominator<=0; divide_quotient<=0;
            completed_mean<=0; divide_remaining<=0; divide_busy<=0;
            qualified_remainder<=0; qualified_quotient<=0; completed_control_mean<=0;
            publish_raw_min<=0; publish_raw_max<=0; publish_median_min<=0; publish_median_max<=0;
            publish_observation_min<=0; publish_observation_max<=0; publish_base_reason<=8'h10;
            sample_q4<=0; control_q4<=0; valid<=0; window_min<=0; window_max<=0;
            sample_bad<=1; sample_otr<=0; quality_reason<=8'h10;
            sample_blind<=0; sample_fault<=0; spike_rejected<=0;
        end else begin
            valid<=0;
            if (median_event) begin
                control_sum<=control_next_sum[22:0]; control_count<=control_next_count;
                qualified_sum<=qualified_next_sum[22:0];
            end
            if (divide_busy) begin
                if (divide_bit) divide_remainder<=divide_remainder-divide_denominator;
                if (qualified_bit) qualified_remainder<=qualified_remainder-divide_denominator;
                divide_denominator<=divide_denominator>>1;
                divide_quotient<=divide_next_quotient;
                qualified_quotient<=qualified_next_quotient;
                divide_remaining<=divide_remaining-1'b1;
                if (divide_remaining == 1) begin
                    completed_mean<=divide_next_quotient; completed_control_mean<=qualified_next_quotient;
                    divide_busy<=0;
                end
            end
            if (window_close) begin
                // Exact nearest Q4, with a 14-bit non-negative quotient.
                divide_remainder<=({8'd0,control_next_sum}<<4)+({18'd0,control_next_count}>>1);
                qualified_remainder<=({8'd0,qualified_next_sum}<<4)+({18'd0,control_next_count}>>1);
                divide_denominator<={18'd0,control_next_count}<<13;
                divide_quotient<=0; qualified_quotient<=0; divide_remaining<=14; divide_busy<=control_next_count != 0;
                if (control_next_count == 0) begin completed_mean<=0; completed_control_mean<=0; end
                control_sum<=0; qualified_sum<=0; control_count<=0;
                pending_epoch<=window_epoch; window_epoch<=!window_epoch;
                publish_raw_min<=next_min; publish_raw_max<=next_max;
                publish_median_min<=median_next_min; publish_median_max<=median_next_max;
                publish_observation_min<=contribution_next_min; publish_observation_max<=contribution_next_max;
                publish_base_reason<={2'b00,(next_max-next_min > 32),
                    (pending_unready || !filtered_ready || !next_seen || incomplete_window),2'b00,next_rail,next_otr};
            end
            if (publication_tick) begin
                sample_q4<=completed_mean; valid<=1; window_min<=publish_raw_min; window_max<=publish_raw_max;
                control_q4<=completed_control_mean;
                quality_reason<=completed_reason; sample_bad<=publish_control_bad || divide_busy;
                sample_otr<=completed_reason[0];
                sample_blind<=publish_control_blind; sample_fault<=publish_control_fault;
                spike_rejected<=publish_control_spike;
            end
        end
    end

    // The baseline is the preceding completed full-stream mean (Q4), even if that
    // window was bad. "Baseline valid" means it exists, not that it is trusted.
    reg diag_baseline_valid;
    reg [13:0] diag_reference_q4;
    reg [15:0] diag_count, diag_outliers, diag_run, diag_longest, diag_edge_outliers;
    reg [15:0] diag_first_index, diag_first_age, diag_step_index, diag_step_age;
    reg [3:0] diag_first_motor, diag_step_motor;
    reg [9:0] diag_max_step;
    reg [31:0] diag_sum;
    reg diag_outlier_seen, diag_step_seen, diag_overflow;
    reg [15:0] diag_next_count, diag_next_outliers, diag_next_run, diag_next_longest, diag_next_edge_outliers;
    reg [15:0] diag_next_first_index, diag_next_first_age, diag_next_step_index, diag_next_step_age;
    reg [3:0] diag_next_first_motor, diag_next_step_motor;
    reg [9:0] diag_next_max_step;
    reg [31:0] diag_next_sum;
    reg diag_next_outlier_seen, diag_next_step_seen, diag_next_overflow;
    wire [13:0] diagnostic_delta = {diagnostic_value,4'b0} > diag_reference_q4 ?
        {diagnostic_value,4'b0}-diag_reference_q4 : diag_reference_q4-{diagnostic_value,4'b0};
    wire diagnostic_outlier = diag_baseline_valid && diagnostic_delta > 256;
    wire [32:0] diagnostic_sum = {1'b0,diag_sum}+{23'd0,diagnostic_value};
    function [15:0] diag_increment;
        input [15:0] value;
        begin diag_increment = value >= DIAG_COUNT_MAX ? DIAG_COUNT_MAX : value+16'd1; end
    endfunction
    always @* begin
        diag_next_count=diag_count; diag_next_outliers=diag_outliers; diag_next_run=diag_run;
        diag_next_longest=diag_longest; diag_next_edge_outliers=diag_edge_outliers;
        diag_next_first_index=diag_first_index; diag_next_first_age=diag_first_age;
        diag_next_step_index=diag_step_index; diag_next_step_age=diag_step_age;
        diag_next_first_motor=diag_first_motor; diag_next_step_motor=diag_step_motor;
        diag_next_max_step=diag_max_step; diag_next_sum=diag_sum;
        diag_next_outlier_seen=diag_outlier_seen; diag_next_step_seen=diag_step_seen;
        diag_next_overflow=diag_overflow;
        if (diagnostic_event) begin
            diag_next_count=diag_increment(diag_count);
            if (diag_count >= DIAG_COUNT_MAX) diag_next_overflow=1;
            if (diagnostic_sum > {1'b0,DIAG_SUM_MAX}) begin
                diag_next_sum=DIAG_SUM_MAX; diag_next_overflow=1;
            end else diag_next_sum=diagnostic_sum[31:0];
            if (diagnostic_ready && (!diag_step_seen || diagnostic_step > diag_max_step)) begin
                diag_next_step_seen=1; diag_next_max_step=diagnostic_step;
                diag_next_step_index=diag_count; diag_next_step_age=diagnostic_tag[15:0];
                diag_next_step_motor=diagnostic_tag[19:16];
            end
            if (diagnostic_outlier) begin
                diag_next_outliers=diag_increment(diag_outliers);
                diag_next_run=diag_increment(diag_run);
                if (diag_outliers >= DIAG_COUNT_MAX || diag_run >= DIAG_COUNT_MAX) diag_next_overflow=1;
                if (diag_next_run > diag_longest) diag_next_longest=diag_next_run;
                if (!diag_outlier_seen) begin
                    diag_next_outlier_seen=1;
                    diag_next_first_index=diag_count; diag_next_first_age=diagnostic_tag[15:0];
                    diag_next_first_motor=diagnostic_tag[19:16];
                end
                if (diagnostic_tag[15:0] <= 100) begin
                    diag_next_edge_outliers=diag_increment(diag_edge_outliers);
                    if (diag_edge_outliers >= DIAG_COUNT_MAX) diag_next_overflow=1;
                end
            end else diag_next_run=0;
        end
    end
    wire [15:0] detail_flags = {4'd0,diag_next_step_motor,diag_next_first_motor,
        diag_next_outlier_seen,diag_next_step_seen,diag_next_overflow,diag_baseline_valid};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            window_detail <= 0; diag_baseline_valid <= 0; diag_reference_q4 <= 0;
            diag_count <= 0; diag_outliers <= 0; diag_run <= 0; diag_longest <= 0; diag_edge_outliers <= 0;
            diag_first_index <= 16'hffff; diag_first_age <= 16'hffff;
            diag_step_index <= 16'hffff; diag_step_age <= 16'hffff;
            diag_first_motor <= 0; diag_step_motor <= 0; diag_max_step <= 0; diag_sum <= 0;
            diag_outlier_seen <= 0; diag_step_seen <= 0; diag_overflow <= 0;
        end else if (publication_tick) begin
            // Word 0 occupies bits 15:0. All words and the final u32 sum are
            // little-endian on the wire; publication shares the control valid.
            window_detail <= {diag_next_sum,detail_flags,diag_next_count,diag_next_edge_outliers,
                diag_next_step_age,diag_next_step_index,diag_next_first_age,diag_next_first_index,
                diag_next_longest,diag_next_outliers,{6'd0,diag_next_max_step},
                {6'd0,publish_observation_max},{6'd0,publish_observation_min},
                {6'd0,publish_median_max},{6'd0,publish_median_min},{2'd0,diag_reference_q4}};
            diag_baseline_valid <= 1; diag_reference_q4 <= completed_mean;
            diag_count <= 0; diag_outliers <= 0; diag_run <= 0; diag_longest <= 0; diag_edge_outliers <= 0;
            diag_first_index <= 16'hffff; diag_first_age <= 16'hffff;
            diag_step_index <= 16'hffff; diag_step_age <= 16'hffff;
            diag_first_motor <= 0; diag_step_motor <= 0; diag_max_step <= 0; diag_sum <= 0;
            diag_outlier_seen <= 0; diag_step_seen <= 0; diag_overflow <= 0;
        end else begin
            diag_count <= diag_next_count; diag_outliers <= diag_next_outliers; diag_run <= diag_next_run;
            diag_longest <= diag_next_longest; diag_edge_outliers <= diag_next_edge_outliers;
            diag_first_index <= diag_next_first_index; diag_first_age <= diag_next_first_age;
            diag_step_index <= diag_next_step_index; diag_step_age <= diag_next_step_age;
            diag_first_motor <= diag_next_first_motor; diag_step_motor <= diag_next_step_motor;
            diag_max_step <= diag_next_max_step; diag_sum <= diag_next_sum;
            diag_outlier_seen <= diag_next_outlier_seen; diag_step_seen <= diag_next_step_seen;
            diag_overflow <= diag_next_overflow;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase <= 0; adc_clk <= 0; warmup <= 0; captured <= 0;
            interval_count <= 0; sample_count <= 0; over_range <= 0;
            pending_min <= 0; pending_max <= 0; pending_seen <= 0; pending_otr <= 0;
            pending_rail <= 0; pending_unready <= 0;
            capture_pending <= 0; filtered <= 0; filtered_ready <= 0;
            contribution_min <= 0; contribution_max <= 0;
            median_min <= 0; median_max <= 0; median_seen <= 0;
        end else begin
            capture_pending <= phase == 4;
            phase <= (phase == 4'd9) ? 4'd0 : phase + 4'd1;
            if (phase == 0) adc_clk <= 1;
            if (phase == 5) adc_clk <= 0;
            if (phase == 4) begin
                captured <= data_in;
                over_range <= otr;
                if (warmup != 63) warmup <= warmup + 1'b1;
            end
            if (median_pending) begin
                filtered <= median_value; filtered_ready <= 1;
            end
            // Startup: discard 63 ADC periods, exceeding 4-cycle pipeline latency.
            if (warmup == 63) begin
                // Raw OTR/rails bypass the median and cover every conversion.
                // Raw extremes remain diagnostics, not a veto on a sound mean.
                if (capture_pending) begin
                    pending_min <= next_min; pending_max <= next_max;
                    pending_seen <= 1; pending_otr <= next_otr; pending_rail <= next_rail;
                end
                if (median_pending) begin
                    median_min <= median_next_min; median_max <= median_next_max; median_seen <= 1;
                end
                if (interval_count == SAMPLE_CYCLES-1) begin
                    interval_count <= 0;
                    contribution_min <= contribution_next_min;
                    contribution_max <= contribution_next_max;
                    pending_unready <= pending_unready || !filtered_ready;
                    if (sample_count == 15) begin
                        sample_count <= 0;
                        pending_seen <= 0; pending_otr <= 0;
                        pending_rail <= 0; pending_unready <= 0;
                        median_seen <= 0;
                    end else begin
                        sample_count <= sample_count + 1'b1;
                    end
                end else interval_count <= interval_count + 1'b1;
            end
        end
    end
endmodule
`default_nettype wire
