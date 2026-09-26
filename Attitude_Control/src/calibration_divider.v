`timescale 1ns/1ps
`default_nettype none
// Restoring division runs only at calibration, avoiding a combinational divider.
module calibration_divider(
    input wire clk, rst_n, start,
    input wire [13:0] denominator,
    output reg [31:0] quotient,
    output reg busy, done
);
    reg [31:0] dividend, result;
    reg [14:0] remainder;
    reg [13:0] divisor;
    reg [5:0] bit_count;
    wire [15:0] trial = {remainder,dividend[31]};
    wire [15:0] difference = trial - {2'b0,divisor};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            quotient <= 0; busy <= 0; done <= 0; dividend <= 0;
            result <= 0; remainder <= 0; divisor <= 0; bit_count <= 0;
        end else begin
            done <= 0;
            if (start && !busy && denominator != 0) begin
                // round(pi * 2^20), angle coefficient is radians / ADC Q4 unit.
                dividend <= 32'd3294199; divisor <= denominator;
                remainder <= 0; result <= 0; bit_count <= 0; busy <= 1;
            end else if (busy) begin
                dividend <= {dividend[30:0],1'b0};
                if (trial >= {2'b0,divisor}) begin
                    remainder <= difference[14:0];
                    result <= {result[30:0],1'b1};
                end else begin
                    remainder <= trial[14:0]; result <= {result[30:0],1'b0};
                end
                if (bit_count == 31) begin
                    quotient <= {result[30:0],(trial >= {2'b0,divisor})};
                    busy <= 0; done <= 1;
                end else bit_count <= bit_count + 1'b1;
            end
        end
    end
endmodule
`default_nettype wire
