`timescale 1ns/1ps
`default_nettype none
// Passive H journal. A row is an already aligned, completed control update.
// Never infer a controller commit from a fixed pipeline delay.
// RAM is synchronous and is deliberately NOT cleared on reset or capture start.
module control_trace_buffer #(
    parameter integer ADDRESS_BITS = 12,
    parameter integer ROW_BITS = 256
)(
    input wire clk, rst_n,
    input wire start_capture,
    input wire freeze_capture,
    input wire [7:0] freeze_reason_in,
    input wire [31:0] device_time_ms,
    input wire [13:0] down_q4, up_q4,
    input wire signed [15:0] capture_arm_q10,
    input wire row_valid,
    input wire [ROW_BITS-1:0] row_data,
    output reg capturing, frozen,
    output reg [15:0] capture_id,
    output reg [ADDRESS_BITS:0] row_count,
    output reg [31:0] total_committed,
    output reg overwritten,
    output reg [31:0] freeze_time_ms,
    output reg [7:0] freeze_reason,
    output reg [13:0] saved_down_q4, saved_up_q4,
    output reg signed [15:0] saved_capture_arm_q10,
    // Ordinal zero is the oldest retained row, independent of ring wrap.
    // Exactly one synchronous-read cycle; invalid requests expose no RAM data.
    input wire read_request,
    input wire [ADDRESS_BITS:0] read_index,
    output reg read_valid,
    output reg [ROW_BITS-1:0] read_data
);
    localparam integer DEPTH = 1 << ADDRESS_BITS;
    localparam [ADDRESS_BITS:0] DEPTH_VALUE = DEPTH;
    reg [ROW_BITS-1:0] memory [0:DEPTH-1];
    reg [ADDRESS_BITS-1:0] write_address;
    wire [ADDRESS_BITS-1:0] oldest = row_count == DEPTH_VALUE ? write_address : {ADDRESS_BITS{1'b0}};
    wire [ADDRESS_BITS-1:0] read_address = oldest + read_index[ADDRESS_BITS-1:0];
    wire write_enable = rst_n && !start_capture && capturing && row_valid;
    wire read_enable = rst_n && !start_capture && frozen && read_request && read_index < row_count;
    reg [ROW_BITS-1:0] ram_read_data;

    // Separate RAM process, with no asynchronous reset, allows BSRAM inference.
    always @(posedge clk) begin
        if (write_enable) memory[write_address] <= row_data;
        if (read_enable) ram_read_data <= memory[read_address];
    end
    // The RAM output is gated by registered validity. Reset/start/read rejection
    // immediately hide a previous capture without resetting the RAM itself.
    always @* begin
        read_data = read_valid ? ram_read_data : {ROW_BITS{1'b0}};
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            capturing <= 0; frozen <= 0; capture_id <= 0;
            row_count <= 0; total_committed <= 0; write_address <= 0;
            overwritten <= 0; freeze_time_ms <= 0; freeze_reason <= 0;
            saved_down_q4 <= 0; saved_up_q4 <= 0; saved_capture_arm_q10 <= 0;
            read_valid <= 0;
        end else begin
            read_valid <= read_enable;
            if (start_capture) begin
                // A new accepted H owns a new generation even if an old dump
                // is still draining. The exporter must abort on this event.
                capture_id <= capture_id + 1'b1;
                row_count <= 0; total_committed <= 0; write_address <= 0;
                overwritten <= 0; frozen <= 0;
                capturing <= !freeze_capture;
                freeze_time_ms <= freeze_capture ? device_time_ms : 32'd0;
                freeze_reason <= freeze_capture ? freeze_reason_in : 8'd0;
                saved_down_q4 <= down_q4; saved_up_q4 <= up_q4;
                saved_capture_arm_q10 <= capture_arm_q10;
                // Same-cycle start/freeze represents an empty, cancelled H.
                if (freeze_capture) frozen <= 1;
            end else if (capturing) begin
                if (row_valid) begin
                    write_address <= write_address + 1'b1;
                    if (row_count < DEPTH_VALUE) row_count <= row_count + 1'b1;
                    else overwritten <= 1;
                    // Saturate the lifetime count: do not silently wrap its
                    // coverage claim on extremely long runs.
                    if (total_committed != 32'hffffffff)
                        total_committed <= total_committed + 1'b1;
                end
                // A row already committed before stop may be supplied with
                // freeze; retain it. No row may be supplied for cancelled work.
                if (freeze_capture) begin
                    capturing <= 0; frozen <= 1;
                    freeze_time_ms <= device_time_ms;
                    freeze_reason <= freeze_reason_in;
                end
            end
        end
    end
endmodule
`default_nettype wire
