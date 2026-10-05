// Bounded open-loop PWM test; not a speed regulator. Command units: permille.
// permit is sampled only on admission. Dynamic safety conditions use stop/fault.
`timescale 1ns/1ps
`default_nettype none
module motor_test #(
    parameter integer RUN_CYCLES = 25000000,
    parameter integer NO_MOTION_CYCLES = 15000000,
    parameter integer MAX_TRAVEL = 128
) (
    input wire clk,
    input wire rst_n,
    input wire request,
    input wire signed [15:0] requested_command,
    input wire permit,
    input wire stop,
    input wire fault,
    input wire signed [31:0] position,
    output reg active,
    output reg signed [15:0] command,
    output reg [7:0] status,
    output wire signed [31:0] delta
);
    localparam integer RUN_LIMIT = RUN_CYCLES < 1 ? 1 : RUN_CYCLES;
    localparam integer NO_MOTION_LIMIT = NO_MOTION_CYCLES < 1 ? 1 : NO_MOTION_CYCLES;
    localparam signed [32:0] TRAVEL_LIMIT = MAX_TRAVEL < 1 ? 33'sd1 : MAX_TRAVEL;
    reg [31:0] elapsed_cycles;
    reg signed [31:0] origin;
    reg has_movement, has_origin;
    // Extend both operands before subtraction: a counter crossing 0x7fffffff
    // must never wrap into a small apparently safe travel distance.
    wire signed [32:0] difference = $signed({position[31],position}) -
                                    $signed({origin[31],origin});
    wire travel_limit = difference >= TRAVEL_LIMIT || difference <= -TRAVEL_LIMIT;
    wire movement_seen = has_movement || position != origin;
    wire command_allowed = requested_command == 16'sd150 || requested_command == -16'sd150 ||
                           requested_command == 16'sd220 || requested_command == -16'sd220;
    // No displacement is attributed to a test before its first admission.
    // Retain the origin after stopping, so subsequent coasting remains visible.
    assign delta = has_origin ? difference[31:0] : 32'sd0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active <= 0; command <= 0; status <= 0;
            origin <= 0; elapsed_cycles <= 0; has_movement <= 0; has_origin <= 0;
        end else if (active && stop) begin
            active <= 0; command <= 0; status <= 8'd4;
        end else if (active && fault) begin
            active <= 0; command <= 0; status <= 8'd5;
        end else if (active) begin
            // Requests during a test cannot restart its clock or reverse it.
            has_movement <= movement_seen;
            if (travel_limit) begin
                active <= 0; command <= 0; status <= 8'd3;
            end else if (elapsed_cycles >= RUN_LIMIT-1) begin
                active <= 0; command <= 0; status <= 8'd2;
            end else if (!movement_seen && elapsed_cycles >= NO_MOTION_LIMIT-1) begin
                active <= 0; command <= 0; status <= 8'd6;
            end else elapsed_cycles <= elapsed_cycles+1'b1;
        end else if (request) begin
            if (permit && !stop && !fault && command_allowed) begin
                active <= 1; command <= requested_command; status <= 8'd1;
                origin <= position; elapsed_cycles <= 0; has_movement <= 0; has_origin <= 1;
            end else begin
                active <= 0; command <= 0; status <= 8'd7;
            end
        end
    end
endmodule
`default_nettype wire
