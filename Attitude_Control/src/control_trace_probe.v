`timescale 1ns/1ps
`default_nettype none
// Binds ADC-epoch inputs to a true controller commit, then observes the next
// registered motor mapping. Observation never authorizes motor output.
module control_trace_probe #(
    parameter integer ADDRESS_BITS = 12
)(
    input wire clk, rst_n,
    input wire adc_valid,
    input wire [13:0] adc_mean_q4,
    input wire signed [31:0] encoder_count,
    input wire [31:0] sample_counter, device_time_ms,
    input wire [7:0] adc_quality, sensor_flags, fault,
    input wire [13:0] down_q4, up_q4,
    input wire h_active, sample_accept, command_commit, exit_fault,
    input wire [63:0] control_states,
    input wire signed [31:0] integral_q24,
    input wire signed [15:0] capture_arm, command, gated_motor_command,
    input wire [55:0] handover_detail,
    input wire read_request,
    input wire [ADDRESS_BITS:0] read_index,
    output wire read_valid,
    output wire [255:0] read_data,
    output wire capturing, frozen,
    output wire [15:0] capture_id,
    output wire [ADDRESS_BITS:0] row_count,
    output wire [31:0] total_committed,
    output wire overwritten,
    output wire [31:0] freeze_time_ms,
    output wire [7:0] freeze_reason,
    output wire [13:0] saved_down_q4, saved_up_q4,
    output wire signed [15:0] saved_capture_arm_q10
);
    reg previous_h, freeze_pending, pending, row_valid;
    reg [7:0] reason;
    reg [255:0] pending_row, row;
    reg [13:0] adc_epoch_mean, accepted_mean;
    reg signed [31:0] adc_epoch_encoder, accepted_encoder;
    reg [31:0] adc_epoch_counter, accepted_counter;
    reg [7:0] adc_epoch_quality, accepted_quality, accepted_sensor_flags;
    wire start_capture = h_active && !previous_h;
    // A new H generation supersedes a freeze left by the previous H, including
    // an immediate board-key stop / UART restart on successive clocks.
    wire freeze_capture = freeze_pending && !pending && !start_capture;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            previous_h <= 0; freeze_pending <= 0; pending <= 0; row_valid <= 0;
            reason <= 0; pending_row <= 0; row <= 0;
            adc_epoch_mean <= 0; accepted_mean <= 0;
            adc_epoch_encoder <= 0; accepted_encoder <= 0;
            adc_epoch_counter <= 0; accepted_counter <= 0;
            adc_epoch_quality <= 0; accepted_quality <= 0; accepted_sensor_flags <= 0;
        end else begin
            previous_h <= h_active;
            row_valid <= 0;
            // Same ADC-valid edge as the production estimator receives.
            // The top-level sample counter is incremented on this edge.
            if (adc_valid) begin
                adc_epoch_mean <= adc_mean_q4;
                adc_epoch_encoder <= encoder_count;
                adc_epoch_counter <= sample_counter + 32'd1;
                adc_epoch_quality <= adc_quality;
            end
            if (sample_accept) begin
                accepted_mean <= adc_epoch_mean;
                accepted_encoder <= adc_epoch_encoder;
                accepted_counter <= adc_epoch_counter;
                accepted_quality <= adc_epoch_quality;
                accepted_sensor_flags <= sensor_flags;
            end
            if (start_capture) begin
                freeze_pending <= 0; pending <= 0; row_valid <= 0;
            end else begin
                if (command_commit) begin
                    pending_row <= {fault,accepted_sensor_flags,accepted_quality,
                        handover_detail[55:48],handover_detail[47:32],16'd0,command,
                        integral_q24,control_states,{2'd0,accepted_mean},
                        accepted_encoder,accepted_counter};
                    pending <= 1;
                end else if (pending) begin
                    // The direction/permission register has now consumed the
                    // submitted command. A coincident stop correctly records 0.
                    row <= pending_row;
                    row[207:192] <= gated_motor_command;
                    row[255:248] <= fault;
                    row_valid <= 1;
                    pending <= 0;
                end
                if (previous_h && !h_active) begin
                    freeze_pending <= 1;
                    // Controller exit state, not a later aggregate sensor flag:
                    // a coincident S takes priority and actually exits to IDLE.
                    reason <= exit_fault ? 8'd2 : 8'd1;
                end else if (freeze_capture) freeze_pending <= 0;
            end
        end
    end
    control_trace_buffer #(.ADDRESS_BITS(ADDRESS_BITS)) journal(
        .clk(clk),.rst_n(rst_n),.start_capture(start_capture),
        .freeze_capture(freeze_capture),.freeze_reason_in(reason),
        .device_time_ms(device_time_ms),.down_q4(down_q4),.up_q4(up_q4),
        .capture_arm_q10(capture_arm),.row_valid(row_valid),.row_data(row),
        .capturing(capturing),.frozen(frozen),.capture_id(capture_id),
        .row_count(row_count),.total_committed(total_committed),.overwritten(overwritten),
        .freeze_time_ms(freeze_time_ms),.freeze_reason(freeze_reason),
        .saved_down_q4(saved_down_q4),.saved_up_q4(saved_up_q4),
        .saved_capture_arm_q10(saved_capture_arm_q10),
        .read_request(read_request),.read_index(read_index),.read_valid(read_valid),.read_data(read_data));
endmodule
`default_nettype wire
