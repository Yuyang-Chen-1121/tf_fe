// ================================================================================================
// File          : fe_sqrt.v
// Project       : Feature Extraction Hardware Accelerator
// Organization  : EPFL INL
// Author        : Yuyang Chen
// Last modified : 2026-09-27
// Language      : Verilog HDL (IEEE 1364-2005)
// ------------------------------------------------------------------------------------------------
// Function
// Digit-by-digit unsigned integer square root with nearest-integer rounding.
//
// Algorithm / implementation
// Consume two radicand bits and produce one root bit on each of 32 steps.
// Shift the partial remainder and compare it with 4*root+1. Append a one
// and subtract the trial value when the trial fits; otherwise append a zero.
// At completion N=root^2+remainder. Round upward exactly when remainder>root.
// An integer radicand cannot lie at the half-integer square-root tie.
// The extra result bit permits rounding a value near 2^64 up to 2^32.
//
// Interface / integration
// Clock: rising-edge clk; reset: asynchronous active-low rst_n for control.
// One transaction at a time. Hold result/out_valid until out_ready.
// clear cancels the operation; acceptance initializes all iterative datapath state.
// CWT supplies a Q40 sum of squares, so the returned magnitude is Q20.
//
// Revision note : Equivalent declaration style and documentation; behavior preserved.
// ================================================================================================

`timescale 1ns/1ps
// Digit-by-digit integer square root, two radicand bits
// per cycle. Final remainder gives exact nearest-integer rounding.
module fe_sqrt (
    input clk,
    input rst_n,
    input clear,
    input in_valid,
    output wire in_ready,
    input [63:0] radicand,
    output wire out_valid,
    input out_ready,
    output wire [32:0] result
);
    // Separate busy and pending-output flags prevent overwriting a stalled
    // result. cycle counts accepted iterative steps, from zero through 31.
    wire running, valid;
    wire [4:0] cycle;
    wire [63:0] number;
    wire [31:0] root;
    wire [65:0] remainder;
    wire accept;
    assign accept = in_valid & in_ready;
    wire step;
    assign step = running & ~clear;
    wire finish;
    assign finish = step & (cycle == 5'd31);
    // One binary square-root digit: bring down the next radicand bit pair
    // and test whether setting the next root bit keeps the remainder nonnegative.
    wire [65:0] shifted;
    assign shifted = {remainder[63:0], number[63:62]};
    wire [65:0] trial;
    assign trial = {32'd0, root, 2'b01};
    wire subtract;
    assign subtract = shifted >= trial;
    wire [65:0] remainder_next;
    assign remainder_next = subtract ? shifted - trial : shifted;
    wire [31:0] root_next;
    assign root_next = {root[30:0], subtract};
    // Nearest-integer rounding follows directly from the final remainder.
    // The 33-bit addition preserves a possible carry above a 32-bit root.
    wire [32:0] rounded;
    assign rounded = {1'b0, root_next} + {32'd0,(remainder_next > {34'd0, root_next})};
    // Acceptance starts processing; the last step transfers ownership to
    // the output-valid flag. clear cancels either phase of the transaction.
    fe_dfflr #(1) u_running (
        clk,
        rst_n,
        clear | accept | finish, // enable
        clear ? 1'b0 : accept,   // next
        running                  // value
    );

    fe_dfflr #(1) u_valid (
        clk,
        rst_n,
        clear | finish | (out_valid & out_ready), // enable
        clear ? 1'b0 : finish,                    // next
        valid                                     // value
    );

    fe_dfflr #(5) u_cycle (
        clk,
        rst_n,
        clear | accept | step,                  // enable
        (clear | accept) ? 5'd0 : cycle + 5'd1, // next
        cycle                                   // value
    );

    // Initialize all iterative state on acceptance, then shift/update it
    // for each active step. Wide datapath registers intentionally have no reset.
    fe_dffl #(64) u_number (
        clk,
        accept | step,                             // enable
        accept ? radicand : {number[61:0], 2'b00}, // next
        number                                     // value
    );

    fe_dffl #(32) u_root (
        clk,
        accept | step,              // enable
        accept ? 32'd0 : root_next, // next
        root                        // value
    );

    fe_dffl #(66) u_remainder (
        clk,
        accept | step,                   // enable
        accept ? 66'd0 : remainder_next, // next
        remainder                        // value
    );

    fe_dffl #(33) u_result (
        clk,
        finish,  // enable
        rounded, // next
        result   // value
    );

    // A new radicand requires both the iterative engine and output slot to
    // be free. The last result retires only through the ready/valid handshake.
    assign in_ready = ~running & ~valid & ~clear;
    assign out_valid = valid & ~clear;
endmodule
