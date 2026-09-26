// 四倍频计数，{A,B}: 00->10->11->01->00 为正向。
// 同步后对整个 AB 状态滤波；两位同时改变只报 illegal，不猜测方向。
`timescale 1ns/1ps
`default_nettype none
module quadrature_encoder #(
    parameter integer FILTER_CYCLES = 8
) (
    input wire clk,
    input wire rst_n,
    input wire a,
    input wire b,
    input wire zero,
    output reg signed [31:0] position,
    output reg illegal
);
    function integer counter_width;
        input integer value;
        integer v;
        begin
            counter_width = 1;
            for (v = value - 1; v > 1; v = v >> 1)
                counter_width = counter_width + 1;
        end
    endfunction
    localparam integer CW = counter_width(FILTER_CYCLES + 1);
    (* syn_preserve = 1, async_reg = "true" *) reg [1:0] a_sync, b_sync;
    reg [2:0] sync_valid;
    reg [1:0] candidate, previous;
    reg [CW-1:0] stable_count;
    reg initialized;
    wire [1:0] sampled = {a_sync[1], b_sync[1]};
    wire accept_state = sync_valid[2] &&
        ((FILTER_CYCLES <= 1) ||
         ((sampled == candidate) && (stable_count >= FILTER_CYCLES - 1)));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_sync <= 2'b00;
            b_sync <= 2'b00;
            sync_valid <= 3'b000;
            candidate <= 2'b00;
            previous <= 2'b00;
            stable_count <= {CW{1'b0}};
            initialized <= 1'b0;
            position <= 32'sd0;
            illegal <= 1'b0;
        end else begin
            a_sync <= {a_sync[0], a};
            b_sync <= {b_sync[0], b};
            sync_valid <= {sync_valid[1:0], 1'b1};
            illegal <= 1'b0;
            if (sync_valid[2]) begin
                if (sampled != candidate) begin
                    candidate <= sampled;
                    stable_count <= {{(CW-1){1'b0}}, 1'b1};
                end else if (stable_count < FILTER_CYCLES)
                    stable_count <= stable_count + 1'b1;
            end
            // 首次可靠状态作为起点，吞掉上电状态，避免虚假位置变化。
            if (accept_state) begin
                previous <= sampled;
                initialized <= 1'b1;
                if (initialized && (sampled != previous)) begin
                    case ({previous, sampled})
                        4'b0010, 4'b1011, 4'b1101, 4'b0100:
                            if (!zero) position <= position + 32'sd1;
                        4'b0001, 4'b0111, 4'b1110, 4'b1000:
                            if (!zero) position <= position - 32'sd1;
                        default: illegal <= 1'b1;
                    endcase
                end
            end
            if (zero) position <= 32'sd0;
        end
    end
endmodule
`default_nettype wire
