// ================================================================================================
// File          : fe_round.v
// Project       : Feature Extraction Hardware Accelerator
// Organization  : EPFL INL
// Author        : Yuyang Chen
// Last modified : 2026-09-27
// Language      : Verilog HDL (IEEE 1364-2005)
// ------------------------------------------------------------------------------------------------
// Function
// Combinational signed nearest-even rounding and signed32 saturation primitives.
//
// Algorithm / implementation
// fe_rne computes floor(value/2^SHIFT) using an arithmetic shift, then obtains
// a nonnegative remainder. Increment for a remainder above half, or exactly
// half with an odd quotient. This also handles negative ties correctly.
// fe_sat32 clamps a signed64 input to [-2^31, 2^31-1] and reports overflow.
//
// Interface / integration
// Both modules are combinational; neither contains a clock or reset.
// fe_rne is used with SHIFT=12 or SHIFT=22 in this design; SHIFT must be positive.
// Rounding changes the fractional position; saturation only limits the range.
//
// Revision note : Equivalent declaration style and documentation; behavior preserved.
// ================================================================================================

`timescale 1ns/1ps
// Signed nearest-even rounding, including negative exact half cases.
module fe_rne #(
    parameter integer SHIFT = 22
) (
    input signed [63:0] value,
    output wire signed [63:0] rounded
);
    // Arithmetic right shift gives a floor quotient even for negative inputs.
    // Subtracting its reconstructed multiple gives remainder in [0, 2^SHIFT).
    wire signed [63:0] quotient;
    assign quotient = value >>> SHIFT;
    wire [63:0] remainder;
    assign remainder = $unsigned(value - (quotient <<< SHIFT));
    localparam [63:0] HALF = 64'd1 << (SHIFT - 1);
    // At an exact half, quotient[0] selects the even neighboring integer.
    // The one-bit increment is explicitly widened as a nonnegative signed value.
    wire increment;
    assign increment = (remainder > HALF) | ((remainder == HALF) & quotient[0]);
    assign rounded = quotient + $signed({63'd0, increment});
endmodule

module fe_sat32 (
    input signed [63:0] value,
    output wire signed [31:0] result,
    output wire overflow
);
    // Compare in signed64 before truncation, preserving both signed32 endpoints.
    // Overflow is asserted for either clamp direction.
    wire high;
    assign high = value > 64'sd2147483647;
    wire low;
    assign low = value < -64'sd2147483648;
    assign overflow = high | low;
    assign result = high ? 32'h7fffffff : low ? 32'h80000000 : value[31:0];
endmodule
