`timescale 1ns/1ps
// Unsigned NUM_WIDTH-by-64 restoring division, four quotient bits per cycle.
// Supported numerator widths are 64 (16 cycles) and 80 (20 cycles).
// Denominator includes the average count and output calibration scale.
// One nearest-even rounding is followed by saturation to unsigned 0..63.
module fe_output_quant #(parameter integer NUM_WIDTH=64) (
    input wire clk, input wire rst_n, input wire clear,
    input wire in_valid, output wire in_ready,
    input wire [NUM_WIDTH-1:0] numerator,
    input wire [63:0] denominator,
    output wire out_valid, input wire out_ready,
    output wire [7:0] out_data,
    output wire error
);
    wire running, valid;
    localparam integer LAST_STEP=NUM_WIDTH/4-1;
    wire [4:0] cycle;
    wire [NUM_WIDTH-1:0] quotient;
    wire [63:0] divisor;
    wire [64:0] remainder;
    wire accept = in_valid & in_ready;
    wire step = running & ~clear;
    wire finish = step & (cycle == LAST_STEP[4:0]);
    genvar digit;
    generate for (digit=0; digit<4; digit=digit+1) begin: g_divide
        wire [NUM_WIDTH-1:0] q_in, q_out;
        wire [64:0] r_in, r_out;
        if (digit==0) begin: g_first
            assign q_in=quotient;
            assign r_in=remainder;
        end else begin: g_later
            assign q_in=g_divide[digit-1].q_out;
            assign r_in=g_divide[digit-1].r_out;
        end
        wire [64:0] trial = {r_in[63:0], q_in[NUM_WIDTH-1]};
        wire subtract = trial >= {1'b0, divisor};
        assign r_out = subtract ? trial-{1'b0, divisor} : trial;
        assign q_out = {q_in[NUM_WIDTH-2:0], subtract};
    end endgenerate
    wire [NUM_WIDTH-1:0] final_quotient=g_divide[3].q_out;
    wire [64:0] final_remainder=g_divide[3].r_out;
    fe_dfflr #(1) u_running(clk, rst_n, clear | accept | finish,
                            clear ? 1'b0 : accept, running);
    fe_dfflr #(1) u_valid(clk, rst_n, clear | finish | (out_valid & out_ready),
                          clear ? 1'b0 : finish, valid);
    fe_dfflr #(5) u_cycle(clk, rst_n, clear | accept | step,
                          (clear | accept) ? 5'd0 : cycle+5'd1, cycle);
    fe_dffl #(64) u_divisor(clk, accept, denominator, divisor);
    fe_dffl #(NUM_WIDTH) u_quotient(clk, accept | step, accept ? numerator : final_quotient, quotient);
    fe_dffl #(65) u_remainder(clk, accept | step, accept ? 65'd0 : final_remainder, remainder);
    wire [65:0] twice_remainder = {final_remainder, 1'b0};
    wire round_up = (twice_remainder > {2'd0, divisor}) |
                   ((twice_remainder == {2'd0, divisor}) & final_quotient[0]);
    wire [7:0] code = (final_quotient >= {{(NUM_WIDTH-6){1'b0}},6'd63}) ? 8'd63 :
                      {2'd0, final_quotient[5:0]} + {7'd0, round_up};
    fe_dffl #(8) u_code(clk, finish, code, out_data);
    fe_dfflr #(1) u_error(clk, rst_n, clear | accept,
                          clear ? 1'b0 : (denominator == 64'd0), error);
    assign in_ready = ~running & ~valid & ~clear;
    assign out_valid = valid & ~clear;
endmodule
