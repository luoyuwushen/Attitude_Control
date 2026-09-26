`timescale 1ns/1ps
`default_nettype none
// 3PA1030: 5 MHz clock, bus captured 80 ns after its rising edge.
// Keep the bus coherent; this is a source-synchronous interface, not ten CDC bits.
module adc_sampler #(
    parameter integer SAMPLE_CYCLES = 3125
)(
    input wire clk, rst_n,
    input wire [9:0] data_in,
    input wire otr,
    output reg adc_clk,
    output wire adc_oe_n,
    output reg [13:0] sample_q4,
    output reg valid,
    output reg over_range
);
    reg [3:0] phase;
    reg [5:0] warmup;
    reg [9:0] captured;
    reg [31:0] interval_count;
    reg [3:0] sample_count;
    reg [13:0] sum;
    assign adc_oe_n = 1'b0; // STBY is grounded on the board, no FPGA port.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase <= 0; adc_clk <= 0; warmup <= 0; captured <= 0;
            interval_count <= 0; sample_count <= 0; sum <= 0;
            sample_q4 <= 0; valid <= 0; over_range <= 0;
        end else begin
            valid <= 0;
            phase <= (phase == 4'd9) ? 4'd0 : phase + 4'd1;
            if (phase == 0) adc_clk <= 1;
            if (phase == 5) adc_clk <= 0;
            if (phase == 4) begin
                captured <= data_in;
                over_range <= otr;
                if (warmup != 63) warmup <= warmup + 1'b1;
            end
            // Startup: discard 63 ADC periods, exceeding 4-cycle pipeline latency.
            if (warmup == 63) begin
                if (interval_count == SAMPLE_CYCLES-1) begin
                    interval_count <= 0;
                    if (sample_count == 15) begin
                        sample_q4 <= sum + {4'b0,captured};
                        sum <= 0; sample_count <= 0; valid <= 1;
                    end else begin
                        sum <= sum + {4'b0,captured};
                        sample_count <= sample_count + 1'b1;
                    end
                end else interval_count <= interval_count + 1'b1;
            end
        end
    end
endmodule
`default_nettype wire
