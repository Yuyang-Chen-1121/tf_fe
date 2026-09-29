// ================================================================================================
// File          : fe_input_affine.v
// Project       : Feature Extraction Hardware Accelerator
// Organization  : EPFL INL
// Author        : Yuyang Chen
// Last modified : 2026-09-27
// Language      : Verilog HDL (IEEE 1364-2005)
// ------------------------------------------------------------------------------------------------
// Function
// Combinational signed ADC normalization using frozen channel calibration.
//
// Algorithm / implementation
// normalized = saturate_signed32(sample * inverse_std + bias).
// sample is a signed 12-bit code. inverse_std and bias are signed24 Q20.
// The raw-units-per-code scale is already folded into inverse_std.
// The product therefore already has 20 fractional bits; no right shift is needed.
// The signed64 temporary preserves the full product and sign-extended bias.
//
// Interface / integration
// Combinational module: no clock, reset, or transaction storage.
// overflow reports that the unbounded result is outside the signed32 range.
// The controller selects the calibration for the current channel.
//
// Revision note : Equivalent declaration style and documentation; behavior preserved.
// ================================================================================================

`timescale 1ns/1ps
// Signed ADC decode and frozen channel z-score. Scaling raw units per ADC
// code is already folded into inverse_std (signed 24-bit Q20).
module fe_input_affine (
    input signed [11:0] sample,
    input signed [23:0] inverse_std,
    input signed [23:0] bias,
    output wire signed [31:0] normalized,
    output wire overflow
);
    // The ADC is a signed integer, while inverse_std has 20 fractional bits.
    // Sign-extend bias explicitly before adding it to the full-width product.
    wire signed [63:0] product;
    assign product = sample * inverse_std;
    wire signed [63:0] total;
    assign total = product + $signed({{40{bias[23]}}, bias});
    // Return the Q20 sample used by both algorithms and report lost range
    // through overflow; fe_engine latches that condition into its runtime error.
    fe_sat32 u_sat (
        total,      // value
        normalized, // result
        overflow
    );

endmodule
