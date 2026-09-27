`timescale 1ns/1ps
// Signed nearest-even rounding, including negative exact half cases.
module fe_rne #(parameter integer SHIFT = 22) (
    input wire signed [63:0] value,
    output wire signed [63:0] rounded
);
    wire signed [63:0] quotient = value >>> SHIFT;
    wire [63:0] remainder = $unsigned(value - (quotient <<< SHIFT));
    localparam [63:0] HALF = 64'd1 << (SHIFT-1);
    wire increment = (remainder > HALF) |
                     ((remainder == HALF) & quotient[0]);
    assign rounded = quotient + $signed({63'd0, increment});
endmodule

module fe_sat32 (
    input wire signed [63:0] value,
    output wire signed [31:0] result,
    output wire overflow
);
    wire high = value > 64'sd2147483647;
    wire low = value < -64'sd2147483648;
    assign overflow = high | low;
    assign result = high ? 32'h7fffffff : low ? 32'h80000000 : value[31:0];
endmodule
