// v1 为 24 字节，v2 为 208 字节小端遥测；UART 8N1，角度使用有符号 Q10。
// 两版共用 0..21 基础字段；v2 在其后追加 TLV，不移动已有字段。
// CRC16/CCITT-FALSE: poly=1021, init=FFFF，无反射/异或，位于帧尾两字节；
// v1 覆盖 0..21，v2 覆盖 0..205。
`timescale 1ns/1ps
`default_nettype none
module telemetry #(
    parameter integer CLK_HZ = 50000000,
    parameter integer BAUD = 115200,
    parameter integer EXTENDED = 0,
    parameter integer TRACE_STATUS = 0,
    parameter integer CONTROL_ADC = 0,
    parameter integer HOLD_ENABLE = 0
) (
    input wire clk,
    input wire rst_n,
    input wire sample_valid,
    input wire [9:0] adc,
    input wire signed [15:0] theta,
    input wire signed [15:0] omega,
    input wire signed [15:0] arm,
    input wire signed [15:0] arm_speed,
    input wire signed [15:0] command,
    input wire [2:0] state,
    input wire [7:0] fault,
    input wire calibrated,
    input wire [7:0] diagnostic_status,
    output wire tx,
    input wire [31:0] device_time_ms,
    input wire signed [31:0] encoder_count,
    input wire [15:0] sensor_flags,
    input wire [9:0] adc_down, adc_up, adc_raw, adc_window_min, adc_window_max,
    input wire [13:0] adc_mean_q4,
    input wire [13:0] adc_control_q4,
    input wire signed [15:0] motor_command,
    input wire [7:0] first_fault,
    input wire [31:0] sample_counter,
    input wire [7:0] adc_quality_reason,
    input wire [15:0] sensor_fault_reason,
    input wire [9:0] adc_fault_window_min, adc_fault_window_max,
    input wire [13:0] adc_fault_mean_q4,
    input wire [271:0] adc_window_detail, adc_fault_detail,
    input wire [31:0] adc_fault_time_ms, adc_fault_sample_counter,
    input wire [7:0] motor_test_status,
    input wire signed [31:0] motor_test_delta,
    input wire [55:0] handover_detail,
    input wire hold_frame,
    input wire [7:0] trace_flags,
    input wire [15:0] trace_row_count, trace_capture_id,
    output wire busy
);
    function [15:0] crc16_byte;
        input [15:0] crc_in;
        input [7:0] byte_in;
        reg [15:0] value;
        integer k;
        begin
            value = crc_in ^ {byte_in, 8'h00};
            for (k = 0; k < 8; k = k + 1)
                value = value[15] ? ((value << 1) ^ 16'h1021) : (value << 1);
            crc16_byte = value;
        end
    endfunction
    localparam integer CONTROL_OFFSET = TRACE_STATUS ? 213 : 206;
    localparam integer FRAME_BYTES = EXTENDED ? (208 + (TRACE_STATUS ? 7 : 0) + (CONTROL_ADC ? 4 : 0)) : 24;
    localparam integer PAYLOAD_BYTES = FRAME_BYTES-2;
    reg [7:0] frame [0:216];
    reg [15:0] sequence_number, crc;
    reg [7:0] byte_index;
    reg sending, draining;
    assign busy = sending || draining;
    wire byte_ready;
    wire [7:0] byte_data = byte_index < PAYLOAD_BYTES ? frame[byte_index] :
                          (byte_index == PAYLOAD_BYTES ? crc[7:0] : crc[15:8]);
    uart_tx_byte #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) transmitter (
        .clk(clk), .rst_n(rst_n), .valid(sending), .data(byte_data),
        .ready(byte_ready), .tx(tx)
    );
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sequence_number <= 16'd0;
            crc <= 16'hFFFF;
            byte_index <= 8'd0;
            sending <= 1'b0;
            draining <= 1'b0;
            for (i = 0; i < PAYLOAD_BYTES; i = i + 1) frame[i] <= 8'd0;
        end else begin
            // 等最后一个停止位发完才接受新快照，忙时样本直接丢弃。
            if (draining && byte_ready) draining <= 1'b0;
            if (!sending && !draining && sample_valid && (!HOLD_ENABLE || !hold_frame)) begin
                frame[0] <= 8'hAA; frame[1] <= 8'h55;
                frame[2] <= EXTENDED ? 8'h02 : 8'h01; frame[3] <= FRAME_BYTES;
                frame[4] <= sequence_number[7:0]; frame[5] <= sequence_number[15:8];
                frame[6] <= adc[7:0]; frame[7] <= {6'd0, adc[9:8]};
                frame[8] <= theta[7:0]; frame[9] <= theta[15:8];
                frame[10] <= omega[7:0]; frame[11] <= omega[15:8];
                frame[12] <= arm[7:0]; frame[13] <= arm[15:8];
                frame[14] <= arm_speed[7:0]; frame[15] <= arm_speed[15:8];
                frame[16] <= command[7:0]; frame[17] <= command[15:8];
                frame[18] <= {5'd0, state}; frame[19] <= {7'd0, calibrated};
                frame[20] <= fault; frame[21] <= diagnostic_status;
                if (EXTENDED) begin
                frame[22] <= 8'd1; frame[23] <= 8'd4;
                frame[24] <= device_time_ms[7:0];
                frame[25] <= device_time_ms[15:8];
                frame[26] <= device_time_ms[23:16];
                frame[27] <= device_time_ms[31:24];
                frame[28] <= 8'd2; frame[29] <= 8'd4;
                frame[30] <= encoder_count[7:0];
                frame[31] <= encoder_count[15:8];
                frame[32] <= encoder_count[23:16];
                frame[33] <= encoder_count[31:24];
                frame[34] <= 8'd9; frame[35] <= 8'd2;
                frame[36] <= 8'he8;
                frame[37] <= 8'h03;
                frame[38] <= 8'd10; frame[39] <= 8'd2;
                frame[40] <= sensor_flags[7:0];
                frame[41] <= sensor_flags[15:8];
                frame[42] <= 8'd11; frame[43] <= 8'd2;
                frame[44] <= adc_down[7:0];
                frame[45] <= {6'd0,adc_down[9:8]};
                frame[46] <= 8'd12; frame[47] <= 8'd2;
                frame[48] <= adc_up[7:0];
                frame[49] <= {6'd0,adc_up[9:8]};
                frame[50] <= 8'd13; frame[51] <= 8'd2;
                frame[52] <= adc_raw[7:0];
                frame[53] <= {6'd0,adc_raw[9:8]};
                frame[54] <= 8'd14; frame[55] <= 8'd2;
                frame[56] <= adc_window_min[7:0];
                frame[57] <= {6'd0,adc_window_min[9:8]};
                frame[58] <= 8'd15; frame[59] <= 8'd2;
                frame[60] <= adc_window_max[7:0];
                frame[61] <= {6'd0,adc_window_max[9:8]};
                frame[62] <= 8'd16; frame[63] <= 8'd2;
                frame[64] <= adc_mean_q4[7:0];
                frame[65] <= {2'd0,adc_mean_q4[13:8]};
                frame[66] <= 8'd17; frame[67] <= 8'd2;
                frame[68] <= motor_command[7:0];
                frame[69] <= motor_command[15:8];
                frame[70] <= 8'd18; frame[71] <= 8'd1;
                frame[72] <= first_fault;
                frame[73] <= 8'd19; frame[74] <= 8'd4;
                frame[75] <= 8'h00;
                frame[76] <= 8'h00;
                frame[77] <= 8'h03;
                frame[78] <= 8'h00;
                frame[79] <= 8'd20; frame[80] <= 8'd4;
                frame[81] <= sample_counter[7:0];
                frame[82] <= sample_counter[15:8];
                frame[83] <= sample_counter[23:16];
                frame[84] <= sample_counter[31:24];
                frame[85] <= 8'd21; frame[86] <= 8'd1;
                frame[87] <= adc_quality_reason;
                frame[88] <= 8'd22; frame[89] <= 8'd2;
                frame[90] <= sensor_fault_reason[7:0];
                frame[91] <= sensor_fault_reason[15:8];
                frame[92] <= 8'd23; frame[93] <= 8'd2;
                frame[94] <= adc_fault_window_min[7:0];
                frame[95] <= {6'd0,adc_fault_window_min[9:8]};
                frame[96] <= 8'd24; frame[97] <= 8'd2;
                frame[98] <= adc_fault_window_max[7:0];
                frame[99] <= {6'd0,adc_fault_window_max[9:8]};
                frame[100] <= 8'd25; frame[101] <= 8'd2;
                frame[102] <= adc_fault_mean_q4[7:0];
                frame[103] <= {2'd0,adc_fault_mean_q4[13:8]};
                frame[104] <= 8'd26; frame[105] <= 8'd34;
                frame[140] <= 8'd27; frame[141] <= 8'd34;
                // The complete packed diagnostic bundles are snapshotted with
                // the legacy fields; byte zero is each bundle's least significant.
                for (i = 0; i < 34; i = i + 1) begin
                    frame[106+i] <= adc_window_detail[i*8 +: 8];
                    frame[142+i] <= adc_fault_detail[i*8 +: 8];
                end
                frame[176] <= 8'd28; frame[177] <= 8'd4;
                frame[178] <= adc_fault_time_ms[7:0];
                frame[179] <= adc_fault_time_ms[15:8];
                frame[180] <= adc_fault_time_ms[23:16];
                frame[181] <= adc_fault_time_ms[31:24];
                frame[182] <= 8'd29; frame[183] <= 8'd4;
                frame[184] <= adc_fault_sample_counter[7:0];
                frame[185] <= adc_fault_sample_counter[15:8];
                frame[186] <= adc_fault_sample_counter[23:16];
                frame[187] <= adc_fault_sample_counter[31:24];
                frame[188] <= 8'd30; frame[189] <= 8'd1;
                frame[190] <= motor_test_status;
                frame[191] <= 8'd31; frame[192] <= 8'd4;
                frame[193] <= motor_test_delta[7:0];
                frame[194] <= motor_test_delta[15:8];
                frame[195] <= motor_test_delta[23:16];
                frame[196] <= motor_test_delta[31:24];
                frame[197] <= 8'd32; frame[198] <= 8'd7;
                for (i = 0; i < 7; i = i + 1)
                    frame[199+i] <= handover_detail[i*8 +: 8];
                if (TRACE_STATUS) begin
                    frame[206] <= 8'd33; frame[207] <= 8'd5;
                    frame[208] <= trace_flags;
                    frame[209] <= trace_row_count[7:0]; frame[210] <= trace_row_count[15:8];
                    frame[211] <= trace_capture_id[7:0]; frame[212] <= trace_capture_id[15:8];
                end
                if (CONTROL_ADC) begin
                    frame[CONTROL_OFFSET] <= 8'd34; frame[CONTROL_OFFSET+1] <= 8'd2;
                    frame[CONTROL_OFFSET+2] <= adc_control_q4[7:0];
                    frame[CONTROL_OFFSET+3] <= {2'd0,adc_control_q4[13:8]};
                end
                end
                sequence_number <= sequence_number + 1'b1;
                crc <= 16'hFFFF;
                byte_index <= 8'd0;
                sending <= 1'b1;
            end else if (sending && byte_ready) begin
                if (byte_index < PAYLOAD_BYTES) crc <= crc16_byte(crc, byte_data);
                if (byte_index == FRAME_BYTES-1) begin
                    sending <= 1'b0;
                    draining <= 1'b1;
                end else byte_index <= byte_index + 1'b1;
            end
        end
    end
endmodule
`default_nettype wire
