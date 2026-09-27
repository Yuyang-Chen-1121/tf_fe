`timescale 1ns/1ps
// Serial complex dot product with two real multipliers and resident sums.
// The caller streams causal taps in ANY fixed order, explicitly supplies zero
// for absent history, and marks the first/last tap of each dot product.
module fe_cwt_mac (
    input wire clk, input wire rst_n, input wire clear,
    input wire in_valid, output wire in_ready,
    input wire first, input wire last,
    input wire signed [31:0] sample,
    input wire signed [23:0] coefficient_re, coefficient_im,
    output wire out_valid, input wire out_ready,
    output wire signed [31:0] result_re, result_im,
    output wire overflow
);
    wire valid_q;
    wire accept = in_valid & in_ready;
    wire finish = accept & last;
    wire signed [63:0] acc_re, acc_im;
    wire signed [63:0] product_re = sample*coefficient_re;
    wire signed [63:0] product_im = sample*coefficient_im;
    wire signed [63:0] sum_re = (first ? 64'sd0 : acc_re) + product_re;
    wire signed [63:0] sum_im = (first ? 64'sd0 : acc_im) + product_im;
    wire signed [63:0] rounded_re, rounded_im;
    wire signed [31:0] next_re, next_im;
    wire overflow_re, overflow_im;
    fe_dffl #(64) u_acc_re(clk, accept, sum_re, acc_re);
    fe_dffl #(64) u_acc_im(clk, accept, sum_im, acc_im);
    fe_rne #(22) u_round_re(sum_re, rounded_re);
    fe_rne #(22) u_round_im(sum_im, rounded_im);
    fe_sat32 u_sat_re(rounded_re, next_re, overflow_re);
    fe_sat32 u_sat_im(rounded_im, next_im, overflow_im);
    fe_dffl #(32) u_result_re(clk, finish, next_re, result_re);
    fe_dffl #(32) u_result_im(clk, finish, next_im, result_im);
    fe_dffl #(1) u_overflow(clk, finish, overflow_re | overflow_im, overflow);
    wire valid_next = clear ? 1'b0 : finish;
    fe_dfflr #(1) u_valid(clk, rst_n, clear | finish | (valid_q & out_ready),
                        valid_next, valid_q);
    assign in_ready = ~valid_q & ~clear;
    assign out_valid = valid_q & ~clear;
endmodule
