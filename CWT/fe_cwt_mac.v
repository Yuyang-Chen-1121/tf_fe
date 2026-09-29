// ================================================================================================
// File          : fe_cwt_mac.v
// Project       : Feature Extraction Hardware Accelerator
// Organization  : EPFL INL
// Author        : Yuyang Chen
// Last modified : 2026-09-27
// Language      : Verilog HDL (IEEE 1364-2005)
// ------------------------------------------------------------------------------------------------
// Function
// Serial complex FIR dot product with full-precision accumulation.
//
// Algorithm / implementation
// For each accepted tap, accumulate sample*coefficient_re and sample*coefficient_im.
// first selects zero instead of the previous sums; last also captures the result.
// Samples are signed32 Q20; coefficients are signed12 with scale 2^-12.
// Products and accumulated sums use signed64 containers. Round once, after
// the complete dot product, by shifting 12 bits with ties to even; saturate to Q20.
// The caller provides causal sample order and explicit zeros for missing history.
//
// Interface / integration
// Clock: rising-edge clk; reset: asynchronous active-low rst_n for output validity.
// clear invalidates the transaction. A new dot product must assert first.
// Results and overflow remain stable while out_valid is held by backpressure.
// Only one completed result can be pending; no new tap is accepted in that state.
//
// Revision note : Equivalent declaration style and documentation; behavior preserved.
// ================================================================================================

`timescale 1ns/1ps
// Serial complex dot product with two real multipliers and resident sums.
// Coefficients are signed 12-bit integers with scale 2^-12. The 64-bit
// accumulator retains all products; one RNE shift returns a Q20 result.
// The caller streams causal taps in ANY fixed order, explicitly supplies zero
// for absent history, and marks the first/last tap of each dot product.
module fe_cwt_mac (
    input clk,
    input rst_n,
    input clear,
    input in_valid,
    output wire in_ready,
    input first,
    input last,
    input signed [31:0] sample,
    input signed [11:0] coefficient_re,
    input signed [11:0] coefficient_im,
    output wire out_valid,
    input out_ready,
    output wire signed [31:0] result_re,
    output wire signed [31:0] result_im,
    output wire overflow
);
    // Tap handshake and transaction boundary detection. last is meaningful
    // only on an accepted tap; stalled input must retain its transaction fields.
    wire valid_q;
    wire accept;
    assign accept = in_valid & in_ready;
    wire finish;
    assign finish = accept & last;
    // Accumulate exact signed products independently for real and imaginary
    // components. first overrides any stale partial sum after clear.
    wire signed [63:0] acc_re, acc_im;
    wire signed [63:0] product_re;
    assign product_re = sample * coefficient_re;
    wire signed [63:0] product_im;
    assign product_im = sample * coefficient_im;
    wire signed [63:0] sum_re;
    assign sum_re = (first ? 64'sd0 : acc_re) + product_re;
    wire signed [63:0] sum_im;
    assign sum_im = (first ? 64'sd0 : acc_im) + product_im;
    wire signed [63:0] rounded_re, rounded_im;
    wire signed [31:0] next_re, next_im;
    wire overflow_re, overflow_im;
    // Update both resident sums once per accepted tap. Keep all guard bits
    // until the entire convolution is complete.
    fe_dffl #(64) u_acc_re (
        clk,
        accept, // enable
        sum_re, // next
        acc_re  // value
    );

    fe_dffl #(64) u_acc_im (
        clk,
        accept, // enable
        sum_im, // next
        acc_im  // value
    );

    // Restore Q20 with one nearest-even rounding per component, then check
    // signed32 saturation. Intermediate taps do not undergo rounding.
    fe_rne #(12) u_round_re (
        sum_re,     // value
        rounded_re  // rounded
    );

    fe_rne #(12) u_round_im (
        sum_im,     // value
        rounded_im  // rounded
    );

    fe_sat32 u_sat_re (
        rounded_re,  // value
        next_re,     // result
        overflow_re  // overflow
    );

    fe_sat32 u_sat_im (
        rounded_im,  // value
        next_im,     // result
        overflow_im  // overflow
    );

    // Capture result and overflow together on the last tap. The registered
    // values remain stable until the consumer retires this output.
    fe_dffl #(32) u_result_re (
        clk,
        finish,    // enable
        next_re,   // next
        result_re  // value
    );

    fe_dffl #(32) u_result_im (
        clk,
        finish,    // enable
        next_im,   // next
        result_im  // value
    );

    fe_dffl #(1) u_overflow (
        clk,
        finish,                    // enable
        overflow_re | overflow_im, // next
        overflow                   // value
    );

    // Control validity rather than resetting wide sums. No new dot product
    // can begin while the completed result is waiting for the consumer.
    wire valid_next;
    assign valid_next = clear ? 1'b0 : finish;
    fe_dfflr #(1) u_valid (
        clk,
        rst_n,
        clear | finish | (valid_q & out_ready), // enable
        valid_next,                             // next
        valid_q                                 // value
    );

    assign in_ready = ~valid_q & ~clear;
    assign out_valid = valid_q & ~clear;
endmodule
