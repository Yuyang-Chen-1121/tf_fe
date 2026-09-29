// ================================================================================================
// File          : ossm_feature_engine.v
// Project       : Feature Extraction Hardware Accelerator - Presentation Edition
// Organization  : EPFL INL
// Author        : Yuyang Chen
// Last modified : 2026-09-28
// Language      : Verilog HDL (IEEE 1364-2005)
// ------------------------------------------------------------------------------------------------
// Function
// Independent OSSM streaming engine, with no algorithm-selection parameter.
//
// Algorithm / implementation
// OSSM: baseline removal, eight complex predictions, one joint innovation,
// gain correction, and five target powers. State persists between tokens.
// Packed INT12 gains expand exactly to the existing Q22 arithmetic interface.
// Each completed averaging block is quantized into one 320-byte token.
// A conventional three-process FSM controls the datapath. Other storage retains
// the original DFF/RAM wrappers and fixed-point operation order.
//
// Interface / integration
// Rising-edge clk; asynchronous active-low control reset rst_n.
// Non-IDLE operations retain the original address/execute two-clock schedule.
// clear/count changes cancel runtime work; valid active calibration is retained.
// Parameter image: 6785 words of 24 bits. Frame: 64 signed12 samples.
// Output: 320 unsigned bytes per token, each in 0..63, with ready/valid flow control.
// Default average count 50 gives one token per 100 ms at 500 Hz input.
// Revision note : Split for presentation; numerical and cycle behavior preserved.
// ================================================================================================

`timescale 1ns/1ps
// Task-spec continuous FE: one state/history, one non-overlapping block average.
// Every frame updates the algorithm and pool. Each full block emits one token.
module ossm_feature_engine #(
    parameter integer CH_NUM = 64,
    parameter integer SAMPLE_WIDTH = 12,
    parameter integer FEATURE_BYTES = 320,
    parameter integer FREQUENCY_MAJOR = 0
) (
    input clk,
    input rst_n,
    input clear,
    input [15:0] average_sample_count,
    input in_valid,
    output reg in_ready,
    input [CH_NUM * SAMPLE_WIDTH - 1:0] in_frame,
    input [23:0] param_word,
    input param_valid,
    input param_first,
    input param_last,
    output wire params_valid,
    output wire out_valid,
    input out_ready,
    output wire [8:0] out_index,
    output wire [7:0] out_data,
    output wire out_last,
    output reg busy,
    output wire error
);
    // Fixed parameter map for this independent OSSM engine.
    localparam integer WORDS = 6785;
    localparam [13:0] DEN_BASE = 14'd6145;
    // Algorithm-specific states. Encodings retain the verified timing reference.
    // Each RAM operation has an address phase followed by an execute phase.
    localparam [4:0] IDLE = 5'd0; // Wait for a configured input-frame handshake.
    localparam [4:0] NORM_INV = 5'd1; // Read the current channel inverse scale.
    localparam [4:0] NORM_BIAS = 5'd2; // Apply bias; store the normalized sample.
    localparam [4:0] ROTATE_LOAD = 5'd7; // Read 16 rotation components and alpha.
    localparam [4:0] PROCESS_START = 5'd8; // Initialize the feature-processing loop.
    localparam [4:0] GAIN_LOAD = 5'd10; // Read eight packed OSSM gain words.
    localparam [4:0] STEP_START = 5'd11; // Launch one channel state update.
    localparam [4:0] STEP_WAIT = 5'd12; // Wait for corrected states and target powers.
    localparam [4:0] POOL = 5'd13; // Accumulate one channel/frequency feature.
    localparam [4:0] DEN_LOW = 5'd14; // Fetch denominator bits 23:0.
    localparam [4:0] DEN_HIGH = 5'd15; // Fetch bits 47:24 and multiply by count.
    localparam [4:0] DIV_START = 5'd16; // Launch calibrated averaging/quantization.
    localparam [4:0] DIV_WAIT = 5'd17; // Store the next quantized feature byte.
    localparam [4:0] COPY = 5'd19; // Publish the complete token into a free bank.
    localparam [4:0] FINISH = 5'd20; // Advance frame lifetime and return to idle.
    reg [4:0] state;
    reg [4:0] state_next;

    // Named control outputs; all are assigned in FSM part 3.
    reg inverse_load;
    reg normalized_write;
    reg process_start;
    reg pool_write;
    reg den_low_load;
    reg den_high_load;
    reg divide_start;
    reg divide_result_ready;
    reg output_write_valid;
    reg frame_done;
    reg rotation_load;
    reg gain_load;
    reg step_start;
    reg step_result_ready;
    reg accept;
    reg quantized;
    reg copy_accept;
    reg pool_advance;
    reg channel_advance;
    reg channel_reset;
    reg frequency_advance;
    reg arithmetic_error;
    reg step_accept;
    reg load_reset;

    // A count change is an epoch boundary: discard partial runtime work and
    // start averaging with the new count. Active calibration remains available.
    wire [15:0] count_q;
    wire count_legal;
    assign count_legal = average_sample_count != 16'd0;
    wire count_changed;
    assign count_changed = average_sample_count != count_q;
    wire runtime_clear;
    assign runtime_clear = clear | count_changed;
    fe_dfflr #(16) u_count (
        clk,
        rst_n,
        count_changed,        // enable
        average_sample_count, // next
        count_q               // value
    );

    wire [15:0] count;
    assign count = count_q;
    // Each controller operation has an address phase and an execute phase.
    // RAM addresses remain stable during the first phase; side effects occur
    // only during the second phase, after synchronous read data is available.
    // Arithmetic submodules still run on every clock. Their handshakes are
    // qualified by operation strobes so a held state cannot launch twice.
    reg phase;
    reg execute;
    // phase is registered with state in FSM part 1. No encoded action bus
    // is needed: FSM part 3 directly drives the named operation enables.

    // Capture all 64 samples atomically on the input handshake, then reuse
    // the captured frame while the serial channel controller is busy.
    wire [767:0] frame;
    fe_dffl #(768) u_frame (
        clk,
        accept,   // enable
        in_frame, // next
        frame     // value
    );

    // One calibration read port serves normalization, algorithm coefficients,
    // and output denominators. A pending shadow image activates at an epoch boundary.
    reg [13:0] parameter_address;
    wire [23:0] parameter_data;
    wire parameter_pending, parameter_error;
    fe_config_store #(WORDS, 14) u_config (
        clk,
        rst_n,
        clear,
        runtime_clear,     // activate
        param_word,
        param_valid,
        param_first,
        param_last,
        params_valid,
        parameter_pending, // pending
        parameter_error,   // error
        parameter_address, // read_address
        parameter_data     // read_data
    );

    // Loop indexes visit 64 channels and five target features. Calibration,
    // pool RAM, and the temporary token use feature_index = channel*5+frequency.
    wire [5:0] channel;
    wire [2:0] frequency;
    wire [4:0] load_index;
    wire channel_last;
    assign channel_last = channel == 6'd63;
    wire frequency_last;
    assign frequency_last = frequency == 3'd4;
    wire [8:0] feature_index;
    assign feature_index = {3'd0, channel} * 9'd5 + {6'd0, frequency};

    // Frame-lifetime control advances once at FINISH; averaging boundaries
    // do not restart OSSM state or gain age.
    wire first_sample, pool_first, pool_last;
    wire [9:0] age;
    fe_stream_control u_stream (
        clk,
        rst_n,
        runtime_clear, // clear
        frame_done,    // frame_done
        count,         // average_count
        first_sample,
        pool_first,
        pool_last,
        age            // gain_age
    );


    // Input affine transform is performed once per channel.
    wire signed [23:0] inverse;
    wire signed [31:0] normalized;
    wire input_overflow;
    fe_dffl #(24) u_inverse (
        clk,
        inverse_load,   // enable
        parameter_data, // next
        inverse         // value
    );

    fe_input_affine u_normalize (
        $signed(frame[channel * 12 +: 12]), // sample
        inverse,                            // inverse_std
        $signed(parameter_data),            // bias
        normalized,
        input_overflow                      // overflow
    );

    wire [31:0] normalized_read;
    fe_ram #(32, 64, 6) u_normalized (
        clk,
        normalized_write, // write_enable
        channel,          // write_address
        normalized,       // write_data
        channel,          // read_address
        normalized_read   // read_data
    );

    // OSSM has eight oscillator lanes and one persistent state per channel.
    // OSSM vector layout: component 2*k is real and component 2*k+1 is
    // imaginary for oscillator k. saved_state stores baseline in bits [31:0]
    // and the 16 state components above it.
    wire [383:0] rotation, gains;
    wire signed [23:0] alpha;
    wire [543:0] saved_state;
    wire [511:0] updated_state;
    wire [31:0] updated_baseline;
    wire [319:0] power, power_q;
    wire step_ready, step_valid, step_overflow;
    genvar element;
    // Two signed Q13 gains share each parameter word. Restore Q22 with
    // sign extension and nine zero LSBs, leaving the arithmetic unchanged.
    wire [191:0] packed_gains;
    generate
        for (element = 0; element < 8; element = element + 1) begin : g_gain_words
            fe_dffl #(24) u_gain (
                clk,
                gain_load & (load_index == element), // enable
                parameter_data,                      // next
                packed_gains[element * 24 +: 24]     // value
            );

        end
        for (element = 0; element < 16; element = element + 1) begin : g_coefficients
            fe_dffl #(24) u_rotation (
                clk,
                rotation_load & (load_index == element), // enable
                parameter_data,                          // next
                rotation[element * 24 +: 24]             // value
            );

            wire [11:0] stored_gain;
            assign stored_gain = packed_gains[element * 12 +: 12];
            assign gains[element * 24 +: 24] = {{3{stored_gain[11]}}, stored_gain, 9'd0};
        end
    endgenerate

    fe_dffl #(24) u_alpha (
        clk,
        rotation_load & (load_index == 5'd16), // enable
        parameter_data,                        // next
        alpha                                  // value
    );

    // Initialize the first update through input selection instead of clearing
    // RAM. Thereafter read and overwrite one persistent state record per channel.
    fe_ram #(544, 64, 6) u_states (
        clk,
        step_accept,                       // write_enable
        channel,                           // write_address
        {updated_state, updated_baseline}, // write_data
        channel,                           // read_address
        saved_state                        // read_data
    );

    fe_ossm_step u_step (
        clk,
        rst_n,
        runtime_clear,                               // clear
        step_start,                                  // in_valid
        step_ready,                                  // in_ready
        $signed(normalized_read),                    // sample
        first_sample ? 32'd0 : saved_state[31:0],    // baseline_in
        first_sample ? 512'd0 : saved_state[543:32], // state_in
        rotation,
        gains,                                       // gain
        alpha,
        step_valid,                                  // out_valid
        step_result_ready,                           // out_ready
        updated_baseline,                            // baseline_out
        updated_state,                               // state_out
        power,                                       // power_out
        step_overflow                                // overflow
    );

    // Hold all five target powers while the pool loop processes them one at
    // a time. Power values are unsigned64 Q40, unlike CWT Q20 magnitudes.
    fe_dffl #(320) u_power (
        clk,
        step_accept, // enable
        power,       // next
        power_q      // value
    );

    // Block averaging stage. A new block logically starts from zero, so
    // old accumulator RAM contents need not be cleared physically.
    wire [79:0] pool_read;
    wire [63:0] instantaneous;
    assign instantaneous = power_q[frequency * 64 +: 64];
    // 80 bits cover 65535 samples of an unsigned 64-bit feature without wrap.
    wire [80:0] pool_sum_wide;
    assign pool_sum_wide = {1'b0,(pool_first ? 80'd0 : pool_read)} + {17'd0, instantaneous};
    wire [79:0] pool_sum;
    assign pool_sum = pool_sum_wide[79:0];
    wire [79:0] numerator;
    fe_ram #(80, 320, 9) u_pool (
        clk,
        pool_write,    // write_enable
        feature_index, // write_address
        pool_sum,      // write_data
        feature_index, // read_address
        pool_read      // read_data
    );

    fe_dffl #(80) u_numerator (
        clk,
        pool_write & pool_last, // enable
        pool_sum,               // next
        numerator               // value
    );

    // Each per-feature denominator arrives as low24 followed by high24.
    // Multiply the complete unsigned48 value by the 16-bit block count before
    // division, fusing averaging and output calibration into one rounding step.
    wire [23:0] denominator_low;
    wire [63:0] denominator;
    wire [47:0] denominator_per_sample;
    assign denominator_per_sample = {parameter_data, denominator_low};
    wire [63:0] denominator_next;
    assign denominator_next = {16'd0, denominator_per_sample} * {48'd0, count};
    fe_dffl #(24) u_den_low (
        clk,
        den_low_load,   // enable
        parameter_data, // next
        denominator_low // value
    );

    fe_dffl #(64) u_denominator (
        clk,
        den_high_load,    // enable
        denominator_next, // next
        denominator       // value
    );

    // Quantization runs only on the last frame of a block. Earlier frames
    // update pool RAM and continue directly to the next feature/channel.
    wire divide_ready, divide_valid, divide_error;
    wire [7:0] code;
    fe_output_quant #(80) u_quantize (
        clk,
        rst_n,
        runtime_clear,       // clear
        divide_start,        // in_valid
        divide_ready,        // in_ready
        numerator,
        denominator,
        divide_valid,        // out_valid
        divide_result_ready, // out_ready
        code,                // out_data
        divide_error         // error
    );

    // Loop indexes advance only when the corresponding operation completes.
    fe_dfflr #(6) u_channel (
        clk,
        rst_n,
        runtime_clear | channel_reset | channel_advance,         // enable
        (runtime_clear | channel_reset) ? 6'd0 : channel + 6'd1, // next
        channel                                                  // value
    );

    fe_dfflr #(3) u_frequency (
        clk,
        rst_n,
        runtime_clear | accept | process_start | frequency_advance, // enable
                                                                    // next
        (runtime_clear | accept | process_start | frequency_last) ? 3'd0 : frequency + 3'd1,
        frequency                                                   // value
    );

    // Reuse load_index for 17 rotation/alpha words and eight packed gain
    // words. PROCESS_START resets the index before reading the frame's gain row.
    fe_dfflr #(5) u_load_index (
        clk,
        rst_n,
        runtime_clear | load_reset | rotation_load | gain_load,  // enable
        (runtime_clear | load_reset) ? 5'd0 : load_index + 5'd1, // next
        load_index                                               // value
    );

    // Publish only a complete 320-byte token; never accumulate ten tokens.
    // Stage the fully quantized token before publishing it to the output
    // queue. If the queue is full, COPY stalls and the next input frame waits.
    wire [8:0] copy_index;
    wire output_write_ready, output_error, token_done;
    wire copy_last;
    assign copy_last = copy_index == 9'd319;
    fe_dfflr #(9) u_copy_index (
        clk,
        rst_n,
        runtime_clear | accept | copy_accept,                // enable
        (runtime_clear | accept) ? 9'd0 : copy_index + 9'd1, // next
        copy_index                                           // value
    );

    wire [7:0] token_read;
    fe_ram #(8, 320, 9) u_token (
        clk,
        quantized,     // write_enable
        feature_index, // write_address
        code,          // write_data
        copy_index,    // read_address
        token_read     // read_data
    );

    fe_token_output #(FREQUENCY_MAJOR) u_output (
        clk,
        rst_n,
        runtime_clear,      // clear
        output_write_valid, // write_valid
        output_write_ready, // write_ready
        token_read,         // write_data
        copy_last,          // write_last
        out_valid,
        out_ready,
        out_index,
        out_data,
        out_last,
        token_done,
        output_error        // error
    );

    // FSM part 1: register the operation state and its RAM timing phase.
    // The address phase precedes the execute phase; cancellation has priority.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            phase <= 1'b0;
        end else if (runtime_clear) begin
            state <= IDLE;
            phase <= 1'b0;
        end else begin
            if (execute)
                state <= state_next;
            if (state == IDLE)
                phase <= 1'b0;
            else
                phase <= ~phase;
        end
    end

    // FSM part 2: complete combinational next-state logic.
    // Handshake waits hold state; no datapath result is consumed twice.
    always @* begin
        state_next = state;
        case (state)
            IDLE: begin
                if (accept)
                    state_next = NORM_INV;
            end
            NORM_INV: state_next = NORM_BIAS;
            NORM_BIAS: begin
                if (!channel_last)
                    state_next = NORM_INV;
                else
                    state_next = ROTATE_LOAD;
            end
            ROTATE_LOAD: begin
                if (load_index == 5'd16)
                    state_next = PROCESS_START;
            end
            PROCESS_START: state_next = GAIN_LOAD;
            GAIN_LOAD: begin
                if (load_index == 5'd7)
                    state_next = STEP_START;
            end
            STEP_START: begin
                if (step_ready)
                    state_next = STEP_WAIT;
            end
            STEP_WAIT: begin
                if (step_accept)
                    state_next = POOL;
            end
            POOL: begin
                if (pool_last)
                    state_next = DEN_LOW;
                else begin
                    if (!frequency_last)
                        state_next = POOL;
                    else if (!channel_last)
                        state_next = STEP_START;
                    else if (pool_last)
                        state_next = COPY;
                    else
                        state_next = FINISH;
                end
            end
            DEN_LOW: state_next = DEN_HIGH;
            DEN_HIGH: state_next = DIV_START;
            DIV_START: begin
                if (divide_ready)
                    state_next = DIV_WAIT;
            end
            DIV_WAIT: begin
                if (quantized) begin
                    if (!frequency_last)
                        state_next = POOL;
                    else if (!channel_last)
                        state_next = STEP_START;
                    else if (pool_last)
                        state_next = COPY;
                    else
                        state_next = FINISH;
                end
            end
            COPY: begin
                if (copy_accept && copy_last)
                    state_next = FINISH;
            end
            FINISH: state_next = IDLE;
            default: state_next = IDLE;
        endcase
    end

    // FSM part 3: state outputs, parameter read address, and operation strobes.
    // Defaults prevent latches. Addresses remain active in the address phase;
    // writes and arithmetic handshakes assert only in the execute phase.
    always @* begin
        execute = phase && !runtime_clear;
        in_ready = 1'b0;
        busy = 1'b1;
        parameter_address = 14'd0;
        inverse_load = 1'b0;
        normalized_write = 1'b0;
        process_start = 1'b0;
        pool_write = 1'b0;
        den_low_load = 1'b0;
        den_high_load = 1'b0;
        divide_start = 1'b0;
        divide_result_ready = 1'b0;
        output_write_valid = 1'b0;
        frame_done = 1'b0;
        rotation_load = 1'b0;
        gain_load = 1'b0;
        step_start = 1'b0;
        step_result_ready = 1'b0;
        accept = 1'b0;
        quantized = 1'b0;
        copy_accept = 1'b0;
        arithmetic_error = 1'b0;
        step_accept = 1'b0;

        case (state)
            IDLE: begin
                execute = !runtime_clear;
                busy = out_valid;
                in_ready = params_valid && count_legal && !runtime_clear && !error;
                accept = in_valid && in_ready;
            end
            NORM_INV: begin
                parameter_address = {8'd0, channel};
                inverse_load = execute;
            end
            NORM_BIAS: begin
                parameter_address = 14'd64 + {8'd0, channel};
                normalized_write = execute;
                arithmetic_error = execute && input_overflow;
            end
            ROTATE_LOAD: begin
                parameter_address = 14'd128 + {9'd0, load_index};
                rotation_load = execute;
            end
            GAIN_LOAD: begin
                parameter_address = 14'd145 + {4'd0, age} * 14'd8 + {9'd0, load_index};
                gain_load = execute;
            end
            STEP_START: step_start = execute;
            STEP_WAIT: begin
                step_result_ready = execute;
                step_accept = execute && step_valid;
                arithmetic_error = step_accept && step_overflow;
            end
            PROCESS_START: process_start = execute;
            POOL: begin
                pool_write = execute;
                arithmetic_error = execute && pool_sum_wide[80];
            end
            DEN_LOW: begin
                parameter_address = DEN_BASE + {5'd0, feature_index} * 14'd2;
                den_low_load = execute;
            end
            DEN_HIGH: begin
                parameter_address = DEN_BASE + {5'd0, feature_index} * 14'd2 + 14'd1;
                den_high_load = execute;
            end
            DIV_START: divide_start = execute;
            DIV_WAIT: begin
                divide_result_ready = execute;
                quantized = execute && divide_valid;
                arithmetic_error = quantized && divide_error;
            end
            COPY: begin
                output_write_valid = execute;
                copy_accept = execute && output_write_ready;
            end
            FINISH: frame_done = execute;
            default: begin
                // An invalid state has no side effects and returns to IDLE.
            end
        endcase

        // Loop control follows completed operations, not elapsed clocks.
        pool_advance = (pool_write && !pool_last) || quantized;
        channel_reset = accept || (normalized_write && channel_last) || process_start;
        channel_advance = (normalized_write && !channel_last) ||
                          (pool_advance && frequency_last && !channel_last);
        frequency_advance = pool_advance;
        load_reset = (normalized_write && channel_last) || process_start;
    end

    // Latch numerical faults only when their corresponding result is
    // consumed. The output case above selects the active numerical check.
    wire runtime_error;
    fe_dfflr #(1) u_error (
        clk,
        rst_n,
        runtime_clear | arithmetic_error, // enable
        ~runtime_clear,                   // next
        runtime_error                     // value
    );

    // Reject unsupported dimensions instead of silently adapting the fixed
    // 64-channel/five-band datapath. All error sources block further frame acceptance.
    wire dimensions_legal;
    assign dimensions_legal = (CH_NUM == 64) & (SAMPLE_WIDTH == 12) & (FEATURE_BYTES == 320);
    assign error =
        parameter_error |
        output_error |
        runtime_error |
        ~count_legal |
        ~dimensions_legal;
endmodule
