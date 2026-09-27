`timescale 1ns/1ps
// Signed ADC decode and frozen channel z-score. Scaling raw units per ADC
// code is already folded into inverse_std (signed 24-bit Q20).
module fe_input_affine (
    input wire signed [11:0] sample,
    input wire signed [23:0] inverse_std, bias,
    output wire signed [31:0] normalized,
    output wire overflow
);
    wire signed [63:0] product = sample*inverse_std;
    wire signed [63:0] total = product + $signed({{40{bias[23]}}, bias});
    fe_sat32 u_sat(total, normalized, overflow);
endmodule
