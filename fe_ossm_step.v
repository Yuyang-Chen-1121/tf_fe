`timescale 1ns/1ps
// One channel OSSM update. The caller supplies persistent state and the gain
// at the current stream age, then stores the result in its channel RAM.
// States and baseline: signed Q20. Rotation/gain/alpha: signed 24-bit Q22.
// Stage order: updated high-pass baseline -> joint prediction/innovation ->
// gain correction -> target power. Oscillators 0..2 are nuisance states.
module fe_ossm_step (
    input wire clk, input wire rst_n, input wire clear,
    input wire in_valid, output wire in_ready,
    input wire signed [31:0] sample,
    input wire signed [31:0] baseline_in,
    input wire [511:0] state_in,
    input wire [383:0] rotation,
    input wire [383:0] gain,
    input wire signed [23:0] alpha,
    output wire out_valid, input wire out_ready,
    output wire signed [31:0] baseline_out,
    output wire [511:0] state_out,
    output wire [319:0] power_out,
    output wire overflow
);
    wire [1:0] phase;
    wire accept = in_valid & in_ready;
    wire predict = phase == 2'd1;
    wire correct = phase == 2'd2;
    wire retire = out_valid & out_ready;
    wire [1:0] phase_next = clear ? 2'd0 : accept ? 2'd1 :
                           predict ? 2'd2 : correct ? 2'd3 : 2'd0;
    fe_dfflr #(2) u_phase(clk, rst_n, clear | accept | predict | correct | retire,
                         phase_next, phase);
    assign in_ready = (phase == 2'd0) & ~clear;
    assign out_valid = (phase == 2'd3) & ~clear;

    wire signed [31:0] sample_q;
    wire [511:0] previous;
    wire [383:0] rotation_q, gain_q;
    fe_dffl #(32) u_sample(clk, accept, sample, sample_q);
    fe_dffl #(512) u_previous(clk, accept, state_in, previous);
    fe_dffl #(384) u_rotation(clk, accept, rotation, rotation_q);
    fe_dffl #(384) u_gain(clk, accept, gain, gain_q);

    // Keep the subtraction wide before multiplying by the small HP alpha.
    wire signed [63:0] sample_wide = {{32{sample[31]}}, sample};
    wire signed [63:0] baseline_wide = {{32{baseline_in[31]}}, baseline_in};
    wire signed [63:0] hp_product = (sample_wide-baseline_wide) * alpha;
    wire signed [63:0] hp_increment, baseline_sum;
    wire signed [31:0] baseline_next;
    wire hp_overflow;
    fe_rne #(22) u_hp_round(hp_product, hp_increment);
    assign baseline_sum = baseline_wide + hp_increment;
    fe_sat32 u_hp_sat(baseline_sum, baseline_next, hp_overflow);
    fe_dffl #(32) u_baseline(clk, accept, baseline_next, baseline_out);

    wire [511:0] predicted, predicted_q;
    wire [15:0] pred_overflow, state_overflow;
    genvar oscillator;
    generate for (oscillator=0; oscillator<8; oscillator=oscillator+1) begin: g_predict
        wire signed [31:0] re = previous[oscillator*64 +: 32];
        wire signed [31:0] im = previous[oscillator*64+32 +: 32];
        wire signed [23:0] a = rotation_q[oscillator*48 +: 24];
        wire signed [23:0] b = rotation_q[oscillator*48+24 +: 24];
        wire signed [63:0] rr = re*a;
        wire signed [63:0] ib = im*b;
        wire signed [63:0] rb = re*b;
        wire signed [63:0] ia = im*a;
        wire signed [63:0] re_product = rr-ib;
        wire signed [63:0] im_product = rb+ia;
        wire signed [63:0] re_rounded, im_rounded;
        wire signed [31:0] re_next, im_next;
        fe_rne #(22) u_re_round(re_product, re_rounded);
        fe_rne #(22) u_im_round(im_product, im_rounded);
        fe_sat32 u_re_sat(re_rounded, re_next, pred_overflow[oscillator*2]);
        fe_sat32 u_im_sat(im_rounded, im_next, pred_overflow[oscillator*2+1]);
        assign predicted[oscillator*64 +: 32] = re_next;
        assign predicted[oscillator*64+32 +: 32] = im_next;
        wire signed [63:0] re_wide = {{32{re_next[31]}},re_next};
    end endgenerate
    // Balanced reduction shortens the joint-observation adder path.
    wire signed [63:0] pair01=g_predict[0].re_wide+g_predict[1].re_wide;
    wire signed [63:0] pair23=g_predict[2].re_wide+g_predict[3].re_wide;
    wire signed [63:0] pair45=g_predict[4].re_wide+g_predict[5].re_wide;
    wire signed [63:0] pair67=g_predict[6].re_wide+g_predict[7].re_wide;
    wire signed [63:0] sum_real=(pair01+pair23)+(pair45+pair67);
    wire signed [63:0] innovation_wide =
        $signed({{32{sample_q[31]}}, sample_q}) -
        $signed({{32{baseline_out[31]}}, baseline_out}) - sum_real;
    wire signed [31:0] innovation_next, innovation_q;
    wire innovation_overflow;
    fe_sat32 u_innovation_sat(innovation_wide, innovation_next, innovation_overflow);
    fe_dffl #(32) u_innovation(clk, predict, innovation_next, innovation_q);
    fe_dffl #(512) u_prediction(clk, predict, predicted, predicted_q);

    wire [511:0] updated;
    genvar component;
    generate for (component=0; component<16; component=component+1) begin: g_correct
        wire signed [23:0] k = gain_q[component*24 +: 24];
        wire signed [31:0] pred = predicted_q[component*32 +: 32];
        wire signed [63:0] product = innovation_q*k;
        wire signed [63:0] rounded, total;
        fe_rne #(22) u_round(product, rounded);
        assign total = $signed({{32{pred[31]}}, pred}) + rounded;
        fe_sat32 u_sat(total, updated[component*32 +: 32], state_overflow[component]);
    end endgenerate
    fe_dffl #(512) u_state(clk, correct, updated, state_out);

    genvar target;
    generate for (target=0; target<5; target=target+1) begin: g_power
        wire signed [31:0] re = state_out[(target+3)*64 +: 32];
        wire signed [31:0] im = state_out[(target+3)*64+32 +: 32];
        wire [63:0] rr = re*re;
        wire [63:0] ii = im*im;
        assign power_out[target*64 +: 64] = rr+ii;
    end endgenerate

    // Overflow belongs to the current transaction and remains stable on stall.
    wire overflow_q;
    wire overflow_next = clear ? 1'b0 : accept ? hp_overflow :
                         predict ? (overflow_q | (|pred_overflow) | innovation_overflow) :
                         (overflow_q | (|state_overflow));
    fe_dfflr #(1) u_overflow(clk, rst_n, clear | accept | predict | correct,
                           overflow_next, overflow_q);
    assign overflow = overflow_q;
endmodule
