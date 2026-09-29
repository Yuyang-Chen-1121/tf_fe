// ================================================================================================
// File          : fe_output_quant.v
// Project       : Feature Extraction Hardware Accelerator
// Organization  : EPFL INL
// Author        : Yuyang Chen
// Last modified : 2026-09-27
// Language      : Verilog HDL (IEEE 1364-2005)
// ------------------------------------------------------------------------------------------------
// Function
// Iterative unsigned division, nearest-even rounding, and six-bit feature clipping.
//
// Algorithm / implementation
// The restoring divider processes four numerator bits per clock.
// Each bit shifts the remainder, compares it with the divisor, and conditionally
// subtracts the divisor while appending one quotient bit.
// After all bits, compare twice the remainder with the divisor for ties-to-even
// rounding, then clip the result to 0..63 in an 8-bit output word.
// The denominator already combines averaging count and feature calibration.
//
// Interface / integration
// Clock: rising-edge clk; reset: asynchronous active-low rst_n for control.
// Supported NUM_WIDTH values are 64 and 80, taking 16 and 20 divide steps.
// One transaction at a time; the completed byte holds while out_ready is low.
// A zero denominator sets error; its computed output must not be consumed.
// clear cancels validity and processing without resetting wide datapath registers.
//
// Revision note : Equivalent declaration style and documentation; behavior preserved.
// ================================================================================================

`timescale 1ns/1ps
// Unsigned NUM_WIDTH-by-64 restoring division, four quotient bits per cycle.
// Supported numerator widths are 64 (16 cycles) and 80 (20 cycles).
// Denominator includes the average count and output calibration scale.
// One nearest-even rounding is followed by saturation to unsigned 0..63.
module fe_output_quant #(
    parameter integer NUM_WIDTH = 64
) (
    input clk,
    input rst_n,
    input clear,
    input in_valid,
    output wire in_ready,
    input [NUM_WIDTH - 1:0] numerator,
    input [63:0] denominator,
    output wire out_valid,
    input out_ready,
    output wire [7:0] out_data,
    output wire error
);
    // The quotient register initially holds the numerator. Each four-bit
    // iteration replaces its oldest numerator bits with new quotient bits.
    wire running, valid;
    localparam integer LAST_STEP = NUM_WIDTH / 4 - 1;
    wire [4:0] cycle;
    wire [NUM_WIDTH - 1:0] quotient;
    wire [63:0] divisor;
    wire [64:0] remainder;
    wire accept;
    assign accept = in_valid & in_ready;
    wire step;
    assign step = running & ~clear;
    wire finish;
    assign finish = step & (cycle == LAST_STEP[4:0]);
    // Unroll four restoring steps combinationally between successive clocks.
    // The first step reads registers; later steps consume the preceding step.
    genvar digit;
    generate
        for (digit = 0; digit < 4; digit = digit + 1) begin : g_divide
            wire [NUM_WIDTH - 1:0] q_in, q_out;
            wire [64:0] r_in, r_out;
            if (digit == 0) begin : g_first
                assign q_in = quotient;
                assign r_in = remainder;
            end else begin : g_later
                assign q_in = g_divide[digit - 1].q_out;
                assign r_in = g_divide[digit - 1].r_out;
            end
            // Shift one numerator bit into the remainder, then conditionally subtract
            // the divisor. The comparison result is the next quotient bit.
            wire [64:0] trial;
            assign trial = {r_in[63:0], q_in[NUM_WIDTH - 1]};
            wire subtract;
            assign subtract = trial >= {1'b0, divisor};
            assign r_out = subtract ? trial - {1'b0, divisor} : trial;
            assign q_out = {q_in[NUM_WIDTH - 2:0], subtract};
        end
    endgenerate
    // Only the fourth unrolled step feeds the registers or final result;
    // no intermediate quotient is rounded or saturated.
    wire [NUM_WIDTH - 1:0] final_quotient;
    assign final_quotient = g_divide[3].q_out;
    wire [64:0] final_remainder;
    assign final_remainder = g_divide[3].r_out;
    // Busy and valid flags serialize transactions and retain a completed
    // output under backpressure. clear cancels work and pending output.
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

    // Latch the divisor at acceptance. Initialize quotient/remainder once,
    // then store the four-step result on each divide clock.
    fe_dffl #(64) u_divisor (
        clk,
        accept,      // enable
        denominator, // next
        divisor      // value
    );

    fe_dffl #(NUM_WIDTH) u_quotient (
        clk,
        accept | step,                       // enable
        accept ? numerator : final_quotient, // next
        quotient                             // value
    );

    fe_dffl #(65) u_remainder (
        clk,
        accept | step,                    // enable
        accept ? 65'd0 : final_remainder, // next
        remainder                         // value
    );

    // Use a wide doubled remainder for an exact half comparison. At a tie,
    // increment only an odd quotient to select the even output code.
    wire [65:0] twice_remainder;
    assign twice_remainder = {final_remainder, 1'b0};
    wire round_up;
    assign round_up =
        (twice_remainder > {2'd0, divisor}) |
        ((twice_remainder == {2'd0, divisor}) & final_quotient[0]);
    // Clip large quotients before narrowing. A quotient of 62 may round to
    // 63; all valid results still have out_data[7:6] equal to zero.
    wire [7:0] code;
    assign code =
        (final_quotient >= {{(NUM_WIDTH - 6) {1'b0}}, 6'd63}) ? 8'd63 : {2'd0, final_quotient[5:0]}
            + {7'd0, round_up};
    fe_dffl #(8) u_code (
        clk,
        finish,   // enable
        code,     // next
        out_data  // value
    );

    // Record division by zero at transaction acceptance. The parent treats
    // this as a runtime error and must discard the resulting byte.
    fe_dfflr #(1) u_error (
        clk,
        rst_n,
        clear | accept,                        // enable
        clear ? 1'b0 : (denominator == 64'd0), // next
        error                                  // value
    );

    assign in_ready = ~running & ~valid & ~clear;
    assign out_valid = valid & ~clear;
endmodule
