// Frozen project-v0.2.1 behavior for diagnostic-only equivalence regression.
`timescale 1ns/1ps
`default_nettype none
// 3PA1030 Fig. 22: conversion/data launch is referenced to the falling edge.
// At 5 MHz, capture 80 ns after rising = 180 ns after the previous falling edge.
// Capture the parallel bus together; individual bit synchronizers are unsuitable.
module adc_sampler_v021_reference #(
    parameter integer SAMPLE_CYCLES = 3125
)(
    input wire clk, rst_n,
    input wire [9:0] data_in,
    input wire otr,
    output reg adc_clk,
    output wire adc_oe_n,
    output reg [13:0] sample_q4,
    output reg valid,
    output reg over_range,
    output wire [9:0] raw_code,
    output reg [9:0] window_min,
    output reg [9:0] window_max,
    output reg sample_bad,
    output reg sample_otr,
    // [0] OTR, [1] raw rail, [2] full-window/contributed median span,
    // [3] median step,
    // [4] incomplete filter, [5] raw span warning only; [7:6] reserved.
    output reg [7:0] quality_reason
);
    reg [3:0] phase;
    reg [5:0] warmup;
    reg [9:0] captured;
    reg [31:0] interval_count;
    reg [3:0] sample_count;
    reg [13:0] sum;
    reg [9:0] pending_min, pending_max;
    reg pending_seen, pending_otr, pending_rail, pending_step, pending_unready, capture_pending;
    reg [9:0] history_1, history_2;
    reg [1:0] history_count;
    reg [9:0] pair_low, pair_high, median_third, filtered;
    reg median_pending, filtered_ready;
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
    // Two registered comparison stages fit between consecutive 5 MHz captures.
    // This rejects one isolated conversion; two consecutive outliers remain
    // observable in the median stream and must still invalidate the window.
    wire [9:0] median_value = median_third < pair_low ? pair_low :
                              median_third > pair_high ? pair_high : median_third;
    wire [9:0] median_step = median_value > filtered ? median_value-filtered : filtered-median_value;
    wire next_step = pending_step || (median_pending && filtered_ready && median_step > 32);
    wire [9:0] contribution_next_min = sample_count == 0 || filtered < contribution_min ? filtered : contribution_min;
    wire [9:0] contribution_next_max = sample_count == 0 || filtered > contribution_max ? filtered : contribution_max;
    // Include every median output, not only the sparse mean contributors:
    // a smooth out-and-back excursion can fit between two contribution ticks.
    wire [9:0] median_next_min = median_pending && (!median_seen || median_value < median_min) ? median_value : median_min;
    wire [9:0] median_next_max = median_pending && (!median_seen || median_value > median_max) ? median_value : median_max;
    wire median_next_seen = median_seen || median_pending;
    wire [9:0] trusted_min = median_next_seen && median_next_min < contribution_next_min ? median_next_min : contribution_next_min;
    wire [9:0] trusted_max = median_next_seen && median_next_max > contribution_next_max ? median_next_max : contribution_next_max;
    wire [7:0] next_reason = {2'b00, (next_max-next_min > 32),
                             (pending_unready || !filtered_ready || !next_seen),
                             next_step, (trusted_max-trusted_min > 32),
                             next_rail, next_otr};
    assign raw_code = captured;
    assign adc_oe_n = 1'b0; // STBY is grounded on the board, no FPGA port.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase <= 0; adc_clk <= 0; warmup <= 0; captured <= 0;
            interval_count <= 0; sample_count <= 0; sum <= 0;
            sample_q4 <= 0; valid <= 0; over_range <= 0;
            window_min <= 0; window_max <= 0; sample_bad <= 1; sample_otr <= 0;
            pending_min <= 0; pending_max <= 0; pending_seen <= 0; pending_otr <= 0;
            pending_rail <= 0; pending_step <= 0; pending_unready <= 0;
            capture_pending <= 0; history_1 <= 0; history_2 <= 0; history_count <= 0;
            pair_low <= 0; pair_high <= 0; median_third <= 0; filtered <= 0;
            median_pending <= 0; filtered_ready <= 0;
            contribution_min <= 0; contribution_max <= 0; quality_reason <= 8'h10;
            median_min <= 0; median_max <= 0; median_seen <= 0;
        end else begin
            valid <= 0;
            capture_pending <= phase == 4;
            median_pending <= 0;
            phase <= (phase == 4'd9) ? 4'd0 : phase + 4'd1;
            if (phase == 0) adc_clk <= 1;
            if (phase == 5) adc_clk <= 0;
            if (phase == 4) begin
                captured <= data_in;
                over_range <= otr;
                if (warmup != 63) warmup <= warmup + 1'b1;
            end
            if (capture_pending) begin
                history_2 <= history_1; history_1 <= captured;
                if (history_count < 2) history_count <= history_count+1'b1;
                pair_low <= history_1 < history_2 ? history_1 : history_2;
                pair_high <= history_1 > history_2 ? history_1 : history_2;
                median_third <= captured;
                median_pending <= history_count == 2;
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
                    pending_step <= next_step;
                    median_min <= median_next_min; median_max <= median_next_max; median_seen <= 1;
                end
                if (interval_count == SAMPLE_CYCLES-1) begin
                    interval_count <= 0;
                    contribution_min <= contribution_next_min;
                    contribution_max <= contribution_next_max;
                    pending_unready <= pending_unready || !filtered_ready;
                    if (sample_count == 15) begin
                        sample_q4 <= sum + {4'b0,filtered};
                        sum <= 0; sample_count <= 0; valid <= 1;
                        window_min <= next_min; window_max <= next_max;
                        sample_otr <= next_otr;
                        quality_reason <= next_reason;
                        sample_bad <= |next_reason[4:0];
                        pending_seen <= 0; pending_otr <= 0;
                        pending_rail <= 0; pending_step <= 0; pending_unready <= 0;
                        median_seen <= 0;
                    end else begin
                        sum <= sum + {4'b0,filtered};
                        sample_count <= sample_count + 1'b1;
                    end
                end else interval_count <= interval_count + 1'b1;
            end
        end
    end
endmodule
`default_nettype wire
