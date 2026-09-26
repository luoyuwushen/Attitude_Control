// 24 字节小端遥测，UART 8N1；角度及角速度使用有符号 Q10 rad(/s)。
// 0..3 AA 55 01 18；4..5序号；6..7 ADC；8..17五个 signed16；
// 18状态、19校准标志、20故障、21保留0；22..23 CRC16 小端。
// CRC16/CCITT-FALSE: poly=1021, init=FFFF，覆盖 0..21，无反射/异或。
`timescale 1ns/1ps
`default_nettype none
module telemetry #(
    parameter integer CLK_HZ = 50000000,
    parameter integer BAUD = 115200
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
    output wire tx
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
    reg [7:0] frame [0:21];
    reg [15:0] sequence_number, crc;
    reg [4:0] byte_index;
    reg sending, draining;
    wire byte_ready;
    wire [7:0] byte_data = byte_index < 22 ? frame[byte_index] :
                          (byte_index == 22 ? crc[7:0] : crc[15:8]);
    uart_tx_byte #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) transmitter (
        .clk(clk), .rst_n(rst_n), .valid(sending), .data(byte_data),
        .ready(byte_ready), .tx(tx)
    );
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sequence_number <= 16'd0;
            crc <= 16'hFFFF;
            byte_index <= 5'd0;
            sending <= 1'b0;
            draining <= 1'b0;
            for (i = 0; i < 22; i = i + 1) frame[i] <= 8'd0;
        end else begin
            // 等最后一个停止位发完才接受新快照，忙时样本直接丢弃。
            if (draining && byte_ready) draining <= 1'b0;
            if (!sending && !draining && sample_valid) begin
                frame[0] <= 8'hAA; frame[1] <= 8'h55;
                frame[2] <= 8'h01; frame[3] <= 8'd24;
                frame[4] <= sequence_number[7:0]; frame[5] <= sequence_number[15:8];
                frame[6] <= adc[7:0]; frame[7] <= {6'd0, adc[9:8]};
                frame[8] <= theta[7:0]; frame[9] <= theta[15:8];
                frame[10] <= omega[7:0]; frame[11] <= omega[15:8];
                frame[12] <= arm[7:0]; frame[13] <= arm[15:8];
                frame[14] <= arm_speed[7:0]; frame[15] <= arm_speed[15:8];
                frame[16] <= command[7:0]; frame[17] <= command[15:8];
                frame[18] <= {5'd0, state}; frame[19] <= {7'd0, calibrated};
                frame[20] <= fault; frame[21] <= 8'd0;
                sequence_number <= sequence_number + 1'b1;
                crc <= 16'hFFFF;
                byte_index <= 5'd0;
                sending <= 1'b1;
            end else if (sending && byte_ready) begin
                if (byte_index < 22) crc <= crc16_byte(crc, byte_data);
                if (byte_index == 23) begin
                    sending <= 1'b0;
                    draining <= 1'b1;
                end else byte_index <= byte_index + 1'b1;
            end
        end
    end
endmodule
`default_nettype wire
