`timescale 1ns/1ps
`default_nettype none
// Read-only download of a frozen H journal. The arbiter grants complete packets;
// once a start bit has been sent, even a motion request finishes that packet.
// No part of this module is in the controller/motor permission path.
module control_trace_export #(
    parameter integer CLK_HZ = 50000000,
    parameter integer BAUD = 115200,
    parameter integer ADDRESS_BITS = 12,
    parameter [31:0] FIRMWARE_ID = 32'h00030000,
    parameter [31:0] SAMPLE_PERIOD_CYCLES = 50000
)(
    input wire clk, rst_n,
    input wire request, permit, frozen, packet_grant,
    input wire [15:0] capture_id,
    input wire [ADDRESS_BITS:0] row_count,
    input wire [31:0] total_committed, freeze_time_ms,
    input wire overwritten,
    input wire [7:0] freeze_reason,
    input wire [13:0] down_q4, up_q4,
    input wire signed [15:0] capture_arm_q10,
    output wire read_request,
    output wire [ADDRESS_BITS:0] read_index,
    input wire read_valid,
    input wire [255:0] read_data,
    output wire active, packet_busy, tx
);
    localparam [3:0] IDLE=0, PREPARE=1, READ_REQUEST=2, READ_WAIT=3,
                     WAIT_GRANT=4, SEND=5, DRAIN=6;
    reg [3:0] state;
    reg [15:0] saved_id, total_rows, ordinal, packet_crc, rows_crc;
    reg [7:0] kind, length, byte_index, read_timeout;
    reg [3:0] packet_rows, loaded_rows;
    reg abort_pending;
    // Seven rows are prefetched before arbitration. Shifting the payload avoids
    // a large variable byte-select mux in the UART data timing path.
    reg [1791:0] payload;
    wire byte_ready;
    reg [7:0] byte_data;
    wire cancelled = !permit || !frozen || capture_id != saved_id;
    assign active = state != IDLE;
    assign packet_busy = state == SEND || state == DRAIN;
    assign read_request = state == READ_REQUEST && !cancelled && !abort_pending;
    assign read_index = ordinal[ADDRESS_BITS:0] + loaded_rows;

    function [15:0] crc16_byte;
        input [15:0] previous;
        input [7:0] value;
        reg [15:0] c;
        integer bit_number;
        begin
            c = previous ^ {value,8'd0};
            for (bit_number=0;bit_number<8;bit_number=bit_number+1)
                c = c[15] ? (c << 1) ^ 16'h1021 : c << 1;
            crc16_byte = c;
        end
    endfunction
    always @* begin
        case (byte_index)
            0: byte_data=8'haa;
            1: byte_data=8'h55;
            2: byte_data=8'd3;
            3: byte_data=length;
            4: byte_data=kind;
            5: byte_data=8'd1;
            6: byte_data=saved_id[7:0];
            7: byte_data=saved_id[15:8];
            8: byte_data=ordinal[7:0];
            9: byte_data=ordinal[15:8];
            10: byte_data=total_rows[7:0];
            11: byte_data=total_rows[15:8];
            12: byte_data={4'd0,packet_rows};
            13: byte_data=8'd0;
            default: byte_data=byte_index==length-2 ? packet_crc[7:0] :
                               byte_index==length-1 ? packet_crc[15:8] : payload[7:0];
        endcase
    end
    uart_tx_byte #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) transmitter(
        .clk(clk),.rst_n(rst_n),.valid(state==SEND),.data(byte_data),
        .ready(byte_ready),.tx(tx));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=IDLE; saved_id<=0; total_rows<=0; ordinal<=0;
            packet_crc<=16'hffff; rows_crc<=16'hffff;
            kind<=0; length<=0; byte_index<=0; read_timeout<=0;
            packet_rows<=0; loaded_rows<=0; abort_pending<=0; payload<=0;
        end else begin
            if (active && cancelled) abort_pending<=1;
            // Outside an on-wire packet, cancellation needs no further RAM access.
            if (active && !packet_busy && (cancelled || abort_pending)) begin
                state<=IDLE;
            end else case (state)
                IDLE: if (request && permit && frozen) begin
                    saved_id<=capture_id; total_rows<=row_count; ordinal<=0;
                    packet_crc<=16'hffff; rows_crc<=16'hffff;
                    abort_pending<=0; kind<=1; length<=40; byte_index<=0;
                    packet_rows<=0; loaded_rows<=0; payload<=0;
                    payload[191:0]<={7'd0,overwritten,freeze_reason,total_committed,
                        freeze_time_ms,SAMPLE_PERIOD_CYCLES,capture_arm_q10,
                        2'd0,up_q4,2'd0,down_q4,FIRMWARE_ID};
                    state<=WAIT_GRANT;
                end
                PREPARE: begin
                    payload<=0; byte_index<=0; packet_crc<=16'hffff;
                    loaded_rows<=0; read_timeout<=0;
                    if (ordinal==total_rows) begin
                        kind<=3; length<=18; packet_rows<=0;
                        payload[15:0]<=rows_crc; state<=WAIT_GRANT;
                    end else begin
                        kind<=2;
                        packet_rows<=total_rows-ordinal>=7 ? 4'd7 : total_rows-ordinal;
                        length<=total_rows-ordinal>=7 ? 8'd240 : 16+(total_rows-ordinal)*32;
                        state<=READ_REQUEST;
                    end
                end
                READ_REQUEST: begin read_timeout<=0; state<=READ_WAIT; end
                READ_WAIT: begin
                    if (read_valid) begin
                        payload[loaded_rows*256 +: 256]<=read_data;
                        loaded_rows<=loaded_rows+1'b1;
                        state<=loaded_rows+1==packet_rows ? WAIT_GRANT : READ_REQUEST;
                    end else if (read_timeout==8'hff) state<=IDLE;
                    else read_timeout<=read_timeout+1'b1;
                end
                WAIT_GRANT: if (packet_grant) state<=SEND;
                SEND: if (byte_ready) begin
                    if (byte_index<length-2) packet_crc<=crc16_byte(packet_crc,byte_data);
                    if (byte_index>=14 && byte_index<length-2) begin
                        payload<=payload>>8;
                        if (kind==2) rows_crc<=crc16_byte(rows_crc,byte_data);
                    end
                    if (byte_index==length-1) state<=DRAIN;
                    else byte_index<=byte_index+1'b1;
                end
                DRAIN: if (byte_ready) begin
                    if (cancelled || abort_pending || kind==3) state<=IDLE;
                    else begin
                        if (kind==2) ordinal<=ordinal+packet_rows;
                        state<=PREPARE;
                    end
                end
                default: state<=IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
