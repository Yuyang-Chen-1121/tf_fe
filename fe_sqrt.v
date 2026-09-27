`timescale 1ns/1ps
// Non-restoring-style digit-by-digit integer square root, two radicand bits
// per cycle. Final remainder gives exact nearest-integer rounding.
module fe_sqrt (
    input wire clk, input wire rst_n, input wire clear,
    input wire in_valid, output wire in_ready,
    input wire [63:0] radicand,
    output wire out_valid, input wire out_ready,
    output wire [32:0] result
);
    wire running, valid;
    wire [4:0] cycle;
    wire [63:0] number;
    wire [31:0] root;
    wire [65:0] remainder;
    wire accept = in_valid & in_ready;
    wire step = running & ~clear;
    wire finish = step & (cycle == 5'd31);
    wire [65:0] shifted = {remainder[63:0], number[63:62]};
    wire [65:0] trial = {32'd0, root, 2'b01};
    wire subtract = shifted >= trial;
    wire [65:0] remainder_next = subtract ? shifted-trial : shifted;
    wire [31:0] root_next = {root[30:0], subtract};
    wire [32:0] rounded = {1'b0, root_next} +
                           {32'd0, (remainder_next > {34'd0, root_next})};
    fe_dfflr #(1) u_running(clk, rst_n, clear | accept | finish,
                            clear ? 1'b0 : accept, running);
    fe_dfflr #(1) u_valid(clk, rst_n, clear | finish | (out_valid & out_ready),
                          clear ? 1'b0 : finish, valid);
    fe_dfflr #(5) u_cycle(clk, rst_n, clear | accept | step,
                          (clear | accept) ? 5'd0 : cycle+5'd1, cycle);
    fe_dffl #(64) u_number(clk, accept | step, accept ? radicand : {number[61:0], 2'b00}, number);
    fe_dffl #(32) u_root(clk, accept | step, accept ? 32'd0 : root_next, root);
    fe_dffl #(66) u_remainder(clk, accept | step, accept ? 66'd0 : remainder_next, remainder);
    fe_dffl #(33) u_result(clk, finish, rounded, result);
    assign in_ready = ~running & ~valid & ~clear;
    assign out_valid = valid & ~clear;
endmodule
