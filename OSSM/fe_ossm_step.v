// ================================================================================================
// File          : fe_ossm_step.v
// Project       : Feature Extraction Hardware Accelerator
// Organization  : EPFL INL
// Author        : Yuyang Chen
// Last modified : 2026-09-28
// Language      : Verilog HDL (IEEE 1364-2005)
// ------------------------------------------------------------------------------------------------
// Function
// One-channel oscillator state-space update using one shared signed multiplier.
//
// Algorithm / implementation
// Algorithm order (RNE denotes nearest-even fixed-point rounding):
// 1. b_new = b + RNE(alpha * (x-b)): estimate the baseline and remove DC drift.
// 2. p_re = RNE(s_re*a - s_im*b_rot), p_im = RNE(s_re*b_rot + s_im*a).
// 3. innovation = x - b_new - sum(p_re over all eight oscillators).
// 4. s_new = p + RNE(gain * innovation), for all 16 real/imaginary components.
// 5. feature_f = s_re^2 + s_im^2, for target oscillators 3..7.
// Prediction pairs are rounded only after both products are combined.
// LOAD -> MULTIPLY -> STORE schedules HP, prediction, correction, and power
// onto one registered 33x32 signed multiplier without changing operation order.
//
// Interface / integration
// Clock: rising-edge clk; reset: asynchronous active-low rst_n for control.
// Capture every input on in_valid && in_ready; later input changes are harmless.
// States/baseline are signed32 Q20; rotation/alpha/expanded gains are signed24 Q22.
// Packed INT12 gain decoding belongs to fe_engine, not to this arithmetic unit.
// power_out contains five unsigned64 Q40 values, target 0 in bits [63:0].
// clear cancels an operation; out_valid and output data otherwise hold until ready.
// overflow is sticky within a transaction and clears on acceptance or clear.
//
// Revision note : Presentation three-process FSM; arithmetic and cycle schedule preserved.
// ================================================================================================

`timescale 1ns/1ps
// One channel OSSM update with one shared, registered signed multiplier.
// States/baseline use signed Q20; rotation/gain/alpha use signed24 Q22.
// The operation order and rounding match the parallel arithmetic reference:
// HP baseline -> all predictions -> joint innovation -> correction -> power.
// Oscillators 0..2 are nuisance states; only oscillators 3..7 produce features.
module fe_ossm_step (
    input clk,
    input rst_n,
    input clear,
    input in_valid,
    output reg in_ready,
    input signed [31:0] sample,
    input signed [31:0] baseline_in,
    input [511:0] state_in,
    input [383:0] rotation,
    input [383:0] gain,
    input signed [23:0] alpha,
    output reg out_valid,
    input out_ready,
    output wire signed [31:0] baseline_out,
    output wire [511:0] state_out,
    output wire [319:0] power_out,
    output wire overflow
);
    // The arithmetic schedule is serialized into operand load, registered
    // multiply, and result-store states. return_phase selects the matching consumer.
    localparam [3:0] IDLE = 0;
    localparam [3:0] HP_LOAD = 1;
    localparam [3:0] PRED_LOAD = 2;
    localparam [3:0] CORR_LOAD = 3;
    localparam [3:0] POWER_LOAD = 4;
    localparam [3:0] MULTIPLY = 5;
    localparam [3:0] HP_STORE = 6;
    localparam [3:0] PRED_STORE = 7;
    localparam [3:0] INNOVATION = 8;
    localparam [3:0] CORR_STORE = 9;
    localparam [3:0] POWER_STORE = 10;
    localparam [3:0] DONE = 11;
    reg [3:0] phase;
    reg [3:0] phase_next;
    wire [3:0] return_phase;
    // Each store strobe is suppressed by clear. Acceptance and retirement
    // are independent handshakes; no second channel update can overlap this one.
    reg in_ready_comb;
    reg out_valid_comb;
    reg accept;
    reg retire;
    reg hp_load;
    reg pred_load;
    reg corr_load;
    reg power_load;
    reg hp_store;
    reg pred_store;
    reg corr_store;
    reg power_store;
    reg innovation_store;
    reg issue;
    reg multiply;

    // Capture every transaction input. The source may change them while busy.
    wire signed [31:0] sample_q, baseline_q;
    wire signed [23:0] alpha_q;
    wire [511:0] previous, predicted_q;
    wire [383:0] rotation_q, gain_q;
    fe_dffl #(32) u_sample (
        clk,
        accept,   // enable
        sample,   // next
        sample_q  // value
    );

    fe_dffl #(32) u_baseline_input (
        clk,
        accept,      // enable
        baseline_in, // next
        baseline_q   // value
    );

    fe_dffl #(24) u_alpha (
        clk,
        accept,  // enable
        alpha,   // next
        alpha_q  // value
    );

    fe_dffl #(512) u_previous (
        clk,
        accept,   // enable
        state_in, // next
        previous  // value
    );

    fe_dffl #(384) u_rotation (
        clk,
        accept,     // enable
        rotation,   // next
        rotation_q  // value
    );

    fe_dffl #(384) u_gain (
        clk,
        accept, // enable
        gain,   // next
        gain_q  // value
    );

    // Four products per oscillator: re*a, im*b, re*b, im*a.
    // Prediction visits four products per oscillator, in order: re*a, im*b,
    // re*b, im*a. term selects the product; oscillator advances after term three.
    wire [2:0] oscillator;
    wire [1:0] term;
    wire [3:0] component;
    wire last_prediction;
    assign last_prediction = (oscillator == 3'd7) & (term == 2'd3);
    wire last_component;
    assign last_component = component == 4'd15;
    fe_dfflr #(2) u_term (
        clk,
        rst_n,
        clear | accept | pred_store,           // enable
        (clear | accept) ? 2'd0 : term + 2'd1, // next
        term                                   // value
    );

    fe_dfflr #(3) u_oscillator (
        clk,
        rst_n,
        clear | accept | (pred_store & (term == 2'd3)), // enable
        (clear | accept) ? 3'd0 : oscillator + 3'd1,    // next
        oscillator                                      // value
    );

    // Corrections visit components 0..15. Power then visits components 6..15.
    wire [3:0] component_next;
    assign component_next =
        (clear | accept) ? 4'd0 : (corr_store & last_component) ? 4'd6 : component + 4'd1;
    fe_dfflr #(4) u_component (
        clk,
        rst_n,
        clear | accept | corr_store | power_store, // enable
        component_next,                            // next
        component                                  // value
    );

    // Extract one old state pair and its complex rotation. Prediction uses
    // only captured old states, so partially corrected states cannot feed back early.
    wire signed [31:0] old_re;
    assign old_re = previous[oscillator * 64 +: 32];
    wire signed [31:0] old_im;
    assign old_im = previous[oscillator * 64 + 32 +: 32];
    wire signed [23:0] rotation_a;
    assign rotation_a = rotation_q[oscillator * 48 +: 24];
    wire signed [23:0] rotation_b;
    assign rotation_b = rotation_q[oscillator * 48 + 24 +: 24];
    wire signed [31:0] prediction_source;
    assign prediction_source = term[0] ? old_im : old_re;
    wire signed [23:0] prediction_factor;
    assign prediction_factor = (term == 2'd0 || term == 2'd3) ? rotation_a : rotation_b;
    // Correction addresses all 16 predicted components. Power later
    // addresses corrected components 6..15, corresponding to target oscillators 3..7.
    wire signed [23:0] correction_gain;
    assign correction_gain = gain_q[component * 24 +: 24];
    wire signed [31:0] prediction_value;
    assign prediction_value = predicted_q[component * 32 +: 32];
    wire signed [31:0] power_value;
    assign power_value = state_out[component * 32 +: 32];
    wire signed [31:0] innovation_q;
    // The difference of two signed32 numbers can require 33 bits; retain
    // that guard bit before multiplying by the high-pass update coefficient.
    wire signed [32:0] hp_difference;
    assign hp_difference = {sample_q[31], sample_q} - {baseline_q[31], baseline_q};

    // The HP subtraction needs 33 bits. All other left operands are signed32.
    // Register operands before the DSP, then register the product separately.
    reg signed [32:0] operand_a;
    reg signed [31:0] operand_b;
    wire signed [32:0] multiplier_a;
    wire signed [31:0] multiplier_b;
    wire signed [63:0] product_q;
    fe_dffl #(33) u_operand_a (
        clk,
        issue,        // enable
        operand_a,    // next
        multiplier_a  // value
    );

    fe_dffl #(32) u_operand_b (
        clk,
        issue,        // enable
        operand_b,    // next
        multiplier_b  // value
    );

    (* syn_use_dsp = "yes" *)
    wire signed [64:0] full_product;
    assign full_product = multiplier_a * multiplier_b;
    // HP is 33x24, prediction/correction are 32x24, and power is 32x32.
    // Each selected operation fits signed64 exactly; bit 64 is sign extension.
    fe_dffl #(64) u_product (
        clk,
        multiply,           // enable
        full_product[63:0], // next
        product_q           // value
    );

    // Remember which store phase owns the forthcoming registered product.
    // FSM part 3 selects this return state together with the operands.
    reg [3:0] return_next;
    fe_dffl #(4) u_return (
        clk,
        issue,        // enable
        return_next,  // next
        return_phase  // value
    );

    // HP and correction products are Q42. RNE by 22 restores a Q20
    // increment, which is then added to the corresponding Q20 baseline/prediction.
    wire signed [63:0] product_rounded;
    fe_rne #(22) u_product_round (
        product_q,       // value
        product_rounded  // rounded
    );

    wire signed [63:0] baseline_sum;
    assign baseline_sum = $signed({{32{baseline_q[31]}}, baseline_q}) + product_rounded;
    wire signed [31:0] baseline_next;
    wire hp_overflow;
    fe_sat32 u_hp_sat (
        baseline_sum,  // value
        baseline_next, // result
        hp_overflow    // overflow
    );

    fe_dffl #(32) u_baseline (
        clk,
        hp_store,      // enable
        baseline_next, // next
        baseline_out   // value
    );

    // Hold the first term of a complex component until the second is ready.
    // Round only after adding/subtracting the complete product pair.
    // A complex prediction component combines two complete products.
    // term[1] selects subtraction for real or addition for imaginary.
    wire signed [63:0] first_product;
    fe_dffl #(64) u_first_product (
        clk,
        pred_store & ~term[0], // enable
        product_q,             // next
        first_product          // value
    );

    wire signed [63:0] prediction_pair;
    assign prediction_pair =
        term[1] ? first_product + product_q : first_product - product_q;
    wire signed [63:0] prediction_rounded;
    wire signed [31:0] prediction_next;
    wire prediction_overflow;
    fe_rne #(22) u_prediction_round (
        prediction_pair,    // value
        prediction_rounded  // rounded
    );

    fe_sat32 u_prediction_sat (
        prediction_rounded,  // value
        prediction_next,     // result
        prediction_overflow  // overflow
    );

    // Store a predicted component after the second product of its pair.
    // The component index interleaves real and imaginary entries per oscillator.
    wire prediction_write;
    assign prediction_write = pred_store & term[0];
    wire [3:0] prediction_index;
    assign prediction_index = {oscillator, term[1]};

    // The sum of eight signed32 predictions fits signed35. A signed64
    // accumulator preserves the old balanced tree's exact integer result.
    wire signed [63:0] prediction_sum;
    wire add_prediction;
    assign add_prediction = pred_store & (term == 2'd1);
    wire signed [63:0] prediction_sum_next;
    assign prediction_sum_next =
        accept ? 64'sd0 : prediction_sum + $signed({{32{prediction_next[31]}}, prediction_next});
    fe_dffl #(64) u_prediction_sum (
        clk,
        accept | add_prediction, // enable
        prediction_sum_next,     // next
        prediction_sum           // value
    );

    // All eight real predictions must be accumulated before innovation is
    // formed. The same saturated innovation is reused by every correction.
    wire signed [63:0] innovation_wide;
    assign innovation_wide =
        $signed({{32{sample_q[31]}}, sample_q}) - $signed({{32{baseline_out[31]}}, baseline_out}) -
            prediction_sum;
    wire signed [31:0] innovation_next;
    wire innovation_overflow;
    fe_sat32 u_innovation_sat (
        innovation_wide,     // value
        innovation_next,     // result
        innovation_overflow  // overflow
    );

    fe_dffl #(32) u_innovation (
        clk,
        innovation_store, // enable
        innovation_next,  // next
        innovation_q      // value
    );

    // Add the rounded gain correction to its predicted component and
    // saturate to signed32. No correction consumes another corrected component.
    wire signed [63:0] correction_sum;
    assign correction_sum =
        $signed({{32{prediction_value[31]}}, prediction_value}) + product_rounded;
    wire signed [31:0] correction_next;
    wire correction_overflow;
    fe_sat32 u_correction_sat (
        correction_sum,      // value
        correction_next,     // result
        correction_overflow  // overflow
    );

    // Independent register enables write exactly one predicted or corrected
    // component at a time; all other state components retain their values.
    genvar element;
    generate
        for (element = 0; element < 16; element = element + 1) begin : g_components
            fe_dffl #(32) u_prediction (
                clk,
                prediction_write & (prediction_index == element), // enable
                prediction_next,                                  // next
                predicted_q[element * 32 +: 32]                   // value
            );

            fe_dffl #(32) u_correction (
                clk,
                corr_store & (component == element), // enable
                correction_next,                     // next
                state_out[element * 32 +: 32]        // value
            );

        end
    endgenerate

    // Power uses corrected states and keeps both unsigned square terms whole.
    // Power is a sum of two nonnegative full-width squares. No Q22
    // coefficient rounding is applied to these Q40 power products.
    wire [63:0] first_square;
    wire [63:0] power_sum;
    assign power_sum = first_square + product_q;
    fe_dffl #(64) u_first_square (
        clk,
        power_store & ~component[0], // enable
        product_q,                   // next
        first_square                 // value
    );

    genvar target;
    generate
        for (target = 0; target < 5; target = target + 1) begin : g_power
            fe_dffl #(64) u_power (
                clk,
                power_store & (component == 2 * (target + 3) + 1), // enable
                power_sum,                                         // next
                power_out[target * 64 +: 64]                       // value
            );

        end
    endgenerate

    // Overflow is transaction-local and remains stable until output retires.
    reg arithmetic_overflow;
    fe_dfflr #(1) u_overflow (
        clk,
        rst_n,
        clear | accept | arithmetic_overflow, // enable
        (clear | accept) ? 1'b0 : 1'b1,       // next
        overflow                              // value
    );

    // Advance through HP, all predictions, innovation, all corrections, and
    // all five powers. DONE holds the complete transaction for its consumer.
    // FSM part 1: current arithmetic stage, reset/cancel, and per-clock advance.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            phase <= IDLE;
        else if (clear)
            phase <= IDLE;
        else
            phase <= phase_next;
    end

    // FSM part 2: schedule one operation through LOAD -> MULTIPLY -> STORE.
    // return_phase identifies the consumer of the registered shared product.
    always @* begin
        phase_next = phase;
        case (phase)
            IDLE: begin
                if (accept)
                    phase_next = HP_LOAD;
            end
            HP_LOAD, PRED_LOAD, CORR_LOAD, POWER_LOAD: phase_next = MULTIPLY;
            MULTIPLY: phase_next = return_phase;
            HP_STORE: phase_next = PRED_LOAD;
            PRED_STORE: begin
                if (last_prediction)
                    phase_next = INNOVATION;
                else
                    phase_next = PRED_LOAD;
            end
            INNOVATION: phase_next = CORR_LOAD;
            CORR_STORE: begin
                if (last_component)
                    phase_next = POWER_LOAD;
                else
                    phase_next = CORR_LOAD;
            end
            POWER_STORE: begin
                if (last_component)
                    phase_next = DONE;
                else
                    phase_next = POWER_LOAD;
            end
            DONE: begin
                if (retire)
                    phase_next = IDLE;
            end
            default: phase_next = IDLE;
        endcase
    end
    // FSM part 3: ready/valid, operation strobes, multiplier routing and faults.
    // Every selected operation is explicit in the state case; no assign-based
    // phase decoder or masked OR network implements the control sequence.
    always @* begin
        in_ready_comb = 1'b0;
        out_valid_comb = 1'b0;
        accept = 1'b0;
        retire = 1'b0;
        hp_load = 1'b0;
        pred_load = 1'b0;
        corr_load = 1'b0;
        power_load = 1'b0;
        hp_store = 1'b0;
        pred_store = 1'b0;
        corr_store = 1'b0;
        power_store = 1'b0;
        innovation_store = 1'b0;
        issue = 1'b0;
        multiply = 1'b0;
        operand_a = 33'sd0;
        operand_b = 32'sd0;
        return_next = IDLE;
        arithmetic_overflow = 1'b0;

        if (!clear) begin
            case (phase)
                IDLE: begin
                    in_ready_comb = 1'b1;
                    accept = in_valid;
                end
                HP_LOAD: begin
                    hp_load = 1'b1;
                    issue = 1'b1;
                    operand_a = hp_difference;
                    operand_b = {{8{alpha_q[23]}}, alpha_q};
                    return_next = HP_STORE;
                end
                PRED_LOAD: begin
                    pred_load = 1'b1;
                    issue = 1'b1;
                    operand_a = {prediction_source[31], prediction_source};
                    operand_b = {{8{prediction_factor[23]}}, prediction_factor};
                    return_next = PRED_STORE;
                end
                CORR_LOAD: begin
                    corr_load = 1'b1;
                    issue = 1'b1;
                    operand_a = {innovation_q[31], innovation_q};
                    operand_b = {{8{correction_gain[23]}}, correction_gain};
                    return_next = CORR_STORE;
                end
                POWER_LOAD: begin
                    power_load = 1'b1;
                    issue = 1'b1;
                    operand_a = {power_value[31], power_value};
                    operand_b = power_value;
                    return_next = POWER_STORE;
                end
                MULTIPLY: multiply = 1'b1;
                HP_STORE: begin
                    hp_store = 1'b1;
                    arithmetic_overflow = hp_overflow;
                end
                PRED_STORE: begin
                    pred_store = 1'b1;
                    arithmetic_overflow = term[0] && prediction_overflow;
                end
                INNOVATION: begin
                    innovation_store = 1'b1;
                    arithmetic_overflow = innovation_overflow;
                end
                CORR_STORE: begin
                    corr_store = 1'b1;
                    arithmetic_overflow = correction_overflow;
                end
                POWER_STORE: power_store = 1'b1;
                DONE: begin
                    out_valid_comb = 1'b1;
                    retire = out_ready;
                end
                default: begin
                    // Invalid encodings issue no operation and return to IDLE.
                end
            endcase
        end

        // Commit each handshake output once per evaluation. Temporary defaults
        // must not propagate through the parent ready/valid combinational path
        // and repeatedly wake both blocks in an event-driven simulator.
        in_ready = in_ready_comb;
        out_valid = out_valid_comb;
    end
endmodule
