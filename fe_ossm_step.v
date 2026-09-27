`timescale 1ns/1ps
// One channel OSSM update with one shared, registered signed multiplier.
// States/baseline use signed Q20; rotation/gain/alpha use signed24 Q22.
// The operation order and rounding match the parallel arithmetic reference:
// HP baseline -> all predictions -> joint innovation -> correction -> power.
// Oscillators 0..2 are nuisance states; only oscillators 3..7 produce features.
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
    localparam [3:0] IDLE=0, HP_LOAD=1, PRED_LOAD=2, CORR_LOAD=3,
        POWER_LOAD=4, MULTIPLY=5, HP_STORE=6, PRED_STORE=7,
        INNOVATION=8, CORR_STORE=9, POWER_STORE=10, DONE=11;
    wire [3:0] phase, return_phase;
    wire accept = in_valid & in_ready;
    wire retire = out_valid & out_ready;
    wire hp_load = (phase==HP_LOAD) & ~clear;
    wire pred_load = (phase==PRED_LOAD) & ~clear;
    wire corr_load = (phase==CORR_LOAD) & ~clear;
    wire power_load = (phase==POWER_LOAD) & ~clear;
    wire hp_store = (phase==HP_STORE) & ~clear;
    wire pred_store = (phase==PRED_STORE) & ~clear;
    wire corr_store = (phase==CORR_STORE) & ~clear;
    wire power_store = (phase==POWER_STORE) & ~clear;
    wire innovation_store = (phase==INNOVATION) & ~clear;
    wire issue = hp_load | pred_load | corr_load | power_load;
    wire multiply = (phase==MULTIPLY) & ~clear;
    assign in_ready = (phase==IDLE) & ~clear;
    assign out_valid = (phase==DONE) & ~clear;

    // Capture every transaction input. The source may change them while busy.
    wire signed [31:0] sample_q, baseline_q;
    wire signed [23:0] alpha_q;
    wire [511:0] previous, predicted_q;
    wire [383:0] rotation_q, gain_q;
    fe_dffl #(32) u_sample(clk,accept,sample,sample_q);
    fe_dffl #(32) u_baseline_input(clk,accept,baseline_in,baseline_q);
    fe_dffl #(24) u_alpha(clk,accept,alpha,alpha_q);
    fe_dffl #(512) u_previous(clk,accept,state_in,previous);
    fe_dffl #(384) u_rotation(clk,accept,rotation,rotation_q);
    fe_dffl #(384) u_gain(clk,accept,gain,gain_q);

    // Four products per oscillator: re*a, im*b, re*b, im*a.
    wire [2:0] oscillator;
    wire [1:0] term;
    wire [3:0] component;
    wire last_prediction = (oscillator==3'd7) & (term==2'd3);
    wire last_component = component==4'd15;
    fe_dfflr #(2) u_term(clk,rst_n,clear | accept | pred_store,
        (clear | accept) ? 2'd0 : term+2'd1,term);
    fe_dfflr #(3) u_oscillator(clk,rst_n,clear | accept | (pred_store & (term==2'd3)),
        (clear | accept) ? 3'd0 : oscillator+3'd1,oscillator);
    // Corrections visit components 0..15. Power then visits components 6..15.
    wire [3:0] component_next = (clear | accept) ? 4'd0 :
        (corr_store & last_component) ? 4'd6 : component+4'd1;
    fe_dfflr #(4) u_component(clk,rst_n,clear | accept | corr_store | power_store,
        component_next,component);

    wire signed [31:0] old_re = previous[oscillator*64 +: 32];
    wire signed [31:0] old_im = previous[oscillator*64+32 +: 32];
    wire signed [23:0] rotation_a = rotation_q[oscillator*48 +: 24];
    wire signed [23:0] rotation_b = rotation_q[oscillator*48+24 +: 24];
    wire signed [31:0] prediction_source = term[0] ? old_im : old_re;
    wire signed [23:0] prediction_factor = (term==2'd0 || term==2'd3) ? rotation_a : rotation_b;
    wire signed [23:0] correction_gain = gain_q[component*24 +: 24];
    wire signed [31:0] prediction_value = predicted_q[component*32 +: 32];
    wire signed [31:0] power_value = state_out[component*32 +: 32];
    wire signed [31:0] innovation_q;
    wire signed [32:0] hp_difference =
        {sample_q[31],sample_q} - {baseline_q[31],baseline_q};

    // The HP subtraction needs 33 bits. All other left operands are signed32.
    // Register operands before the DSP, then register the product separately.
    wire signed [32:0] operand_a =
        ({33{hp_load}} & hp_difference) |
        ({33{pred_load}} & {prediction_source[31],prediction_source}) |
        ({33{corr_load}} & {innovation_q[31],innovation_q}) |
        ({33{power_load}} & {power_value[31],power_value});
    wire signed [31:0] operand_b =
        ({32{hp_load}} & {{8{alpha_q[23]}},alpha_q}) |
        ({32{pred_load}} & {{8{prediction_factor[23]}},prediction_factor}) |
        ({32{corr_load}} & {{8{correction_gain[23]}},correction_gain}) |
        ({32{power_load}} & power_value);
    wire signed [32:0] multiplier_a;
    wire signed [31:0] multiplier_b;
    wire signed [63:0] product_q;
    fe_dffl #(33) u_operand_a(clk,issue,operand_a,multiplier_a);
    fe_dffl #(32) u_operand_b(clk,issue,operand_b,multiplier_b);
    (* syn_use_dsp = "yes" *) wire signed [64:0] full_product = multiplier_a*multiplier_b;
    // HP is 33x24, prediction/correction are 32x24, and power is 32x32.
    // Each selected operation fits signed64 exactly; bit 64 is sign extension.
    fe_dffl #(64) u_product(clk,multiply,full_product[63:0],product_q);
    wire [3:0] return_next = ({4{hp_load}} & HP_STORE) |
        ({4{pred_load}} & PRED_STORE) | ({4{corr_load}} & CORR_STORE) |
        ({4{power_load}} & POWER_STORE);
    fe_dffl #(4) u_return(clk,issue,return_next,return_phase);

    wire signed [63:0] product_rounded;
    fe_rne #(22) u_product_round(product_q,product_rounded);
    wire signed [63:0] baseline_sum =
        $signed({{32{baseline_q[31]}},baseline_q}) + product_rounded;
    wire signed [31:0] baseline_next;
    wire hp_overflow;
    fe_sat32 u_hp_sat(baseline_sum,baseline_next,hp_overflow);
    fe_dffl #(32) u_baseline(clk,hp_store,baseline_next,baseline_out);

    // Hold the first term of a complex component until the second is ready.
    // Round only after adding/subtracting the complete product pair.
    wire signed [63:0] first_product;
    fe_dffl #(64) u_first_product(clk,pred_store & ~term[0],product_q,first_product);
    wire signed [63:0] prediction_pair = term[1] ?
        first_product+product_q : first_product-product_q;
    wire signed [63:0] prediction_rounded;
    wire signed [31:0] prediction_next;
    wire prediction_overflow;
    fe_rne #(22) u_prediction_round(prediction_pair,prediction_rounded);
    fe_sat32 u_prediction_sat(prediction_rounded,prediction_next,prediction_overflow);
    wire prediction_write = pred_store & term[0];
    wire [3:0] prediction_index = {oscillator,term[1]};

    // The sum of eight signed32 predictions fits signed35. A signed64
    // accumulator preserves the old balanced tree's exact integer result.
    wire signed [63:0] prediction_sum;
    wire add_prediction = pred_store & (term==2'd1);
    wire signed [63:0] prediction_sum_next = accept ? 64'sd0 :
        prediction_sum + $signed({{32{prediction_next[31]}},prediction_next});
    fe_dffl #(64) u_prediction_sum(clk,accept | add_prediction,prediction_sum_next,prediction_sum);
    wire signed [63:0] innovation_wide =
        $signed({{32{sample_q[31]}},sample_q}) -
        $signed({{32{baseline_out[31]}},baseline_out}) - prediction_sum;
    wire signed [31:0] innovation_next;
    wire innovation_overflow;
    fe_sat32 u_innovation_sat(innovation_wide,innovation_next,innovation_overflow);
    fe_dffl #(32) u_innovation(clk,innovation_store,innovation_next,innovation_q);

    wire signed [63:0] correction_sum =
        $signed({{32{prediction_value[31]}},prediction_value}) + product_rounded;
    wire signed [31:0] correction_next;
    wire correction_overflow;
    fe_sat32 u_correction_sat(correction_sum,correction_next,correction_overflow);
    genvar element;
    generate for (element=0; element<16; element=element+1) begin: g_components
        fe_dffl #(32) u_prediction(clk,prediction_write & (prediction_index==element),
            prediction_next,predicted_q[element*32 +: 32]);
        fe_dffl #(32) u_correction(clk,corr_store & (component==element),
            correction_next,state_out[element*32 +: 32]);
    end endgenerate

    // Power uses corrected states and keeps both unsigned square terms whole.
    wire [63:0] first_square;
    wire [63:0] power_sum = first_square + product_q;
    fe_dffl #(64) u_first_square(clk,power_store & ~component[0],product_q,first_square);
    genvar target;
    generate for (target=0; target<5; target=target+1) begin: g_power
        fe_dffl #(64) u_power(clk,power_store & (component==2*(target+3)+1),
            power_sum,power_out[target*64 +: 64]);
    end endgenerate

    // Overflow is transaction-local and remains stable until output retires.
    wire arithmetic_overflow = (hp_store & hp_overflow) |
        (prediction_write & prediction_overflow) |
        (innovation_store & innovation_overflow) | (corr_store & correction_overflow);
    fe_dfflr #(1) u_overflow(clk,rst_n,clear | accept | arithmetic_overflow,
        (clear | accept) ? 1'b0 : 1'b1,overflow);

    wire [3:0] selected_phase =
        ({4{phase==IDLE}} & (accept ? HP_LOAD : IDLE)) |
        ({4{phase==HP_LOAD}} & MULTIPLY) |
        ({4{phase==PRED_LOAD}} & MULTIPLY) |
        ({4{phase==CORR_LOAD}} & MULTIPLY) |
        ({4{phase==POWER_LOAD}} & MULTIPLY) |
        ({4{phase==MULTIPLY}} & return_phase) |
        ({4{phase==HP_STORE}} & PRED_LOAD) |
        ({4{phase==PRED_STORE}} & (last_prediction ? INNOVATION : PRED_LOAD)) |
        ({4{phase==INNOVATION}} & CORR_LOAD) |
        ({4{phase==CORR_STORE}} & (last_component ? POWER_LOAD : CORR_LOAD)) |
        ({4{phase==POWER_STORE}} & (last_component ? DONE : POWER_LOAD)) |
        ({4{phase==DONE}} & (retire ? IDLE : DONE));
    fe_dfflr #(4) u_phase(clk,rst_n,1'b1,clear ? IDLE : selected_phase,phase);
endmodule
