// ================================================================================================
// File          : fe_stream_control.v
// Project       : Feature Extraction Hardware Accelerator
// Organization  : EPFL INL
// Author        : Yuyang Chen
// Last modified : 2026-09-27
// Language      : Verilog HDL (IEEE 1364-2005)
// ------------------------------------------------------------------------------------------------
// Function
// Frame-based lifetime counters for initialization, pooling, and OSSM gain age.
//
// Algorithm / implementation
// Advance only when the surrounding engine reports frame_done.
// The first completed frame marks the persistent algorithm state as initialized.
// Pool position wraps at average_count; gain_age independently saturates at 749.
// Pool boundaries do not reset the oscillator states or wavelet history.
//
// Interface / integration
// Clock: rising-edge clk; reset: asynchronous active-low rst_n.
// clear synchronously starts a fresh lifetime for all three counters.
// The parent validates average_count and asserts clear when the count changes.
// first_sample/pool_first/pool_last describe the frame currently being processed.
//
// Revision note : Equivalent declaration style and documentation; behavior preserved.
// ================================================================================================

`timescale 1ns/1ps
// Continuous stream lifetime: only clear resets history/state/gain age.
// Pool boundaries reset the block position, never the algorithm state.
module fe_stream_control (
    input clk,
    input rst_n,
    input clear,
    input frame_done,
    input [15:0] average_count,
    output wire first_sample,
    output wire pool_first,
    output wire pool_last,
    output wire [9:0] gain_age
);
    // started tracks lifetime initialization, while position tracks only
    // the current averaging block. These are deliberately separate concepts.
    wire started;
    wire [15:0] position;
    assign first_sample = ~started;
    assign pool_first = position == 16'd0;
    assign pool_last = (average_count != 16'd0) & (position == average_count - 16'd1);
    // The first FINISH event retires the initial frame. Subsequent pool
    // wraps leave started asserted and retain the algorithm history/state.
    fe_dfflr #(1) u_started (
        clk,
        rst_n,
        clear | frame_done, // enable
        ~clear,             // next
        started             // value
    );

    // Reset position on a completed block or runtime clear; otherwise count
    // one completed frame. A partial averaging block never emits a token.
    fe_dfflr #(16) u_position (
        clk,
        rst_n,
        clear | frame_done,                             // enable
        (clear | pool_last) ? 16'd0 : position + 16'd1, // next
        position                                        // value
    );

    // Hold the terminal gain; do not wrap it onto a live state.
    fe_dfflr #(10) u_age (
        clk,
        rst_n,
        clear | frame_done,                                                 // enable
        clear ? 10'd0 : (gain_age == 10'd749) ? 10'd749 : gain_age + 10'd1, // next
        gain_age                                                            // value
    );

endmodule
