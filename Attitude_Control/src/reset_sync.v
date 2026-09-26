// 异步复位立即生效，解除复位经过两个时钟沿，避免异步释放亚稳态。
`timescale 1ns/1ps
`default_nettype none
module reset_sync (
    input wire clk,
    input wire arst_n,
    output wire rst_n
);
    (* syn_preserve = 1, async_reg = "true" *) reg [1:0] release_pipe;
    always @(posedge clk or negedge arst_n) begin
        if (!arst_n) release_pipe <= 2'b00;
        else release_pipe <= {release_pipe[0], 1'b1};
    end
    assign rst_n = release_pipe[1];
endmodule
`default_nettype wire
