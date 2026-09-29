// ============================================================================
// File          : tb_fe_stream.v
// Project       : Feature Extraction Hardware Accelerator
// Organization  : EPFL INL
// Author        : Yuyang Chen
// Last modified : 2026-09-29
// Language      : Verilog HDL (IEEE 1364-2005), simulation only
// ----------------------------------------------------------------------------
// Function
// Standalone real-data replay of one CWT or OSSM engine. The same stimulus and
// scoreboard serve both output layouts. No reference RTL or internal RAM writes
// are used: all parameter words and input samples enter through public ports.
//
// Test sequence
// Reset -> load_parameters -> wait for configuration activation -> send_frame
// repeatedly -> drain output -> verify frame/token counts and report TB_PASS.
// The fixture contains 1000 signed12 frames and 20 fixed-point golden tokens
// for average_sample_count=50. A shorter run compares the corresponding prefix.
//
// Timing / checking
// Drive stimulus at falling edges and sample transfers at rising edges, avoiding
// races with DUT sequential logic. Keep each input stable until its handshake.
// Check every output byte, index, final marker, six-bit range and stall stability.
// Periodically stall the output; insert gaps in configuration loading. The Python
// runner checks the TB_PASS marker because portable Verilog $finish exits with 0.
// ============================================================================
`timescale 1ns/1ps

module tb_fe_stream;
    parameter integer OSSM = 0;
    parameter integer FREQUENCY_MAJOR = 0;
    parameter integer FRAME_COUNT = 1000;
    parameter integer FRAME_INTERVAL = 0;
    localparam integer AVERAGE_COUNT = 50;
    localparam integer FIXTURE_FRAMES = 1000;
    localparam integer GOLDEN_BYTES = 6400;
    localparam integer PARAMETER_WORDS = (OSSM != 0) ? 6785 : 1117;
    localparam integer EXPECTED_BYTES = (FRAME_COUNT / AVERAGE_COUNT) * 320;
    localparam integer CYCLE_LIMIT = FRAME_COUNT *
        ((FRAME_INTERVAL > 200000) ? FRAME_INTERVAL : 200000) + 100000;

    reg clk;
    reg rst_n;
    reg clear;
    reg [15:0] average_sample_count;
    reg in_valid;
    wire in_ready;
    reg [767:0] in_frame;
    reg [23:0] param_word;
    reg param_valid;
    reg param_first;
    reg param_last;
    wire params_valid;
    wire out_valid;
    reg out_ready;
    wire [8:0] out_index;
    wire [7:0] out_data;
    wire out_last;
    wire busy;
    wire error;

    // These arrays belong to the testbench, not to the DUT. readmemh only loads
    // this external source/scoreboard; tasks below then exercise actual ports.
    reg [23:0] parameters [0:PARAMETER_WORDS-1];
    reg [767:0] frames [0:FIXTURE_FRAMES-1];
    reg [7:0] golden [0:GOLDEN_BYTES-1];
    reg [8191:0] output_path;
    reg [8191:0] input_path;
    reg [8191:0] parameter_path;
    reg [8191:0] wave_path;
    integer output_file;
    integer input_file;
    integer parameter_file;
    integer cycles;
    integer accepted_frames;
    integer loaded_words;
    integer received_bytes;
    integer expected_index;
    integer golden_index;
    integer last_input_cycle;
    integer frame_start_cycle;
    integer max_frame_cycles;
    integer frame_number;
    reg frame_active;
    reg stalled_previous;
    reg [7:0] held_data;
    reg [8:0] held_index;
    reg held_last;

    // Only the selected generate branch is elaborated. Both public tops have
    // identical interfaces; the synthesizable .f lists contain no testbench.
    generate
        if (OSSM != 0) begin : g_ossm
            ossm_feature_engine #(
                .FREQUENCY_MAJOR(FREQUENCY_MAJOR)
            ) dut (
                .clk(clk),
                .rst_n(rst_n),
                .clear(clear),
                .average_sample_count(average_sample_count),
                .in_valid(in_valid),
                .in_ready(in_ready),
                .in_frame(in_frame),
                .param_word(param_word),
                .param_valid(param_valid),
                .param_first(param_first),
                .param_last(param_last),
                .params_valid(params_valid),
                .out_valid(out_valid),
                .out_ready(out_ready),
                .out_index(out_index),
                .out_data(out_data),
                .out_last(out_last),
                .busy(busy),
                .error(error)
            );
        end else begin : g_cwt
            cwt_feature_engine #(
                .FREQUENCY_MAJOR(FREQUENCY_MAJOR)
            ) dut (
                .clk(clk),
                .rst_n(rst_n),
                .clear(clear),
                .average_sample_count(average_sample_count),
                .in_valid(in_valid),
                .in_ready(in_ready),
                .in_frame(in_frame),
                .param_word(param_word),
                .param_valid(param_valid),
                .param_first(param_first),
                .param_last(param_last),
                .params_valid(params_valid),
                .out_valid(out_valid),
                .out_ready(out_ready),
                .out_index(out_index),
                .out_data(out_data),
                .out_last(out_last),
                .busy(busy),
                .error(error)
            );
        end
    endgenerate

    // All delays are simulation-only. The test clock is 100 MHz.
    always #5 clk = ~clk;

    task fail;
        input [1279:0] message;
        begin
            $display("TB_FAIL cycle=%0d frames=%0d bytes=%0d: %0s",
                     cycles, accepted_frames, received_bytes, message);
            $finish;
        end
    endtask

    // External configuration master: one word per valid rising edge, with a
    // bubble every 17 words. There is no param_ready signal. first/last must be
    // asserted together with valid on exactly the first/final word respectively.
    task load_parameters;
        integer word_index;
        begin
            $display("PARAM_BEGIN words=%0d width=24", PARAMETER_WORDS);
            for (word_index = 0; word_index < PARAMETER_WORDS;
                 word_index = word_index + 1) begin
                @(negedge clk);
                param_word = parameters[word_index];
                param_valid = 1'b1;
                param_first = (word_index == 0);
                param_last = (word_index == PARAMETER_WORDS - 1);
                @(posedge clk);
                if (word_index % 17 == 0) begin
                    @(negedge clk);
                    param_valid = 1'b0;
                    param_first = 1'b0;
                    param_last = 1'b0;
                end
            end
            @(negedge clk);
            param_valid = 1'b0;
            param_first = 1'b0;
            param_last = 1'b0;
            param_word = 24'd0;
            // The first complete image activates only after valid is released.
            while (params_valid !== 1'b1 || in_ready !== 1'b1)
                @(negedge clk);
            $display("PARAM_ACTIVE cycle=%0d params_valid=1 in_ready=1", cycles);
        end
    endtask

    // External sample source: frame[ch*12 +: 12] is a two's-complement signed12
    // code. Channel 0 occupies the LEAST significant 12 bits of the packed word.
    // in_valid and ALL 768 data bits remain stable while in_ready is low.
    task send_frame;
        input integer index;
        integer channel;
        begin
            @(negedge clk);
            while (index > 0 && cycles - last_input_cycle + 1 < FRAME_INTERVAL)
                @(negedge clk);
            for (channel = 0; channel < 64; channel = channel + 1)
                in_frame[channel * 12 +: 12] = frames[index][channel * 12 +: 12];
            in_valid = 1'b1;
            @(posedge clk);
            while (in_ready !== 1'b1)
                @(posedge clk);
            @(negedge clk);
            in_valid = 1'b0;
            in_frame = 768'd0;
        end
    endtask

    // Emulate a consumer that periodically pauses. The scoreboard checks that
    // a stalled valid byte remains unchanged, including on its acceptance edge.
    always @(negedge clk) begin
        if (!rst_n)
            out_ready = 1'b0;
        else
            out_ready = (cycles % 17 >= 3);
    end

    // Scoreboard uses only public interfaces. No hierarchy-specific state names
    // or debug ports are needed, so a fresh checkout of syn is self-contained.
    always @(posedge clk) begin
        cycles = cycles + 1;
        if (cycles > CYCLE_LIMIT)
            fail("Simulation watchdog expired");
        if (rst_n && !clear) begin
            if ((^{in_ready, params_valid, out_valid, out_last, busy, error}) === 1'bx)
                fail("Unknown public control signal");
            if (error !== 1'b0)
                fail("DUT error asserted");
            if (!params_valid && in_ready)
                fail("Input became ready before configuration activation");

            if (param_valid) begin
                if (loaded_words >= PARAMETER_WORDS ||
                    param_word !== parameters[loaded_words] ||
                    param_first !== (loaded_words == 0) ||
                    param_last !== (loaded_words == PARAMETER_WORDS - 1))
                    fail("Parameter source protocol mismatch");
                $fdisplay(parameter_file, "%0d,%0d,%06h,%0d,%0d",
                          cycles, loaded_words, param_word, param_first, param_last);
                loaded_words = loaded_words + 1;
            end

            // Completion precedes acceptance here: an old frame may finish on
            // the same rising edge that accepts the next already-waiting frame.
            if (frame_active && in_ready) begin
                if (cycles - frame_start_cycle > max_frame_cycles)
                    max_frame_cycles = cycles - frame_start_cycle;
                if (cycles - frame_start_cycle > 200000)
                    fail("Frame service time exceeded 2 ms at 100 MHz");
                frame_active = 1'b0;
            end
            if (in_valid && in_ready) begin
                if (accepted_frames >= FRAME_COUNT)
                    fail("Too many input handshakes");
                if (FRAME_INTERVAL > 0 && accepted_frames > 0 &&
                    cycles - last_input_cycle < FRAME_INTERVAL)
                    fail("Input source violated requested frame interval");
                $fdisplay(input_file, "%0d,%0d,%0d,%0192h", cycles,
                          accepted_frames, $signed(in_frame[11:0]), in_frame);
                if (accepted_frames < 2 || (accepted_frames + 1) % 50 == 0)
                    $display("INPUT_ACCEPT frame=%0d cycle=%0d ch0_signed=%0d",
                             accepted_frames, cycles, $signed(in_frame[11:0]));
                accepted_frames = accepted_frames + 1;
                last_input_cycle = cycles;
                frame_start_cycle = cycles;
                frame_active = 1'b1;
            end

            if (stalled_previous) begin
                if (out_valid !== 1'b1 || out_data !== held_data ||
                    out_index !== held_index || out_last !== held_last)
                    fail("Output changed while stalled");
            end
            stalled_previous = out_valid && !out_ready;
            held_data = out_data;
            held_index = out_index;
            held_last = out_last;

            if (out_valid) begin
                expected_index = received_bytes % 320;
                if ((^{out_data, out_index}) === 1'bx || out_data[7:6] !== 2'b00)
                    fail("Output unknown or outside unsigned range 0..63");
                if (out_index !== expected_index[8:0] ||
                    out_last !== (expected_index == 319))
                    fail("Output index/last mismatch");
                if (accepted_frames < (received_bytes / 320 + 1) * AVERAGE_COUNT)
                    fail("Token published before enough input frames");
            end
            if (out_valid && out_ready) begin
                if (received_bytes >= EXPECTED_BYTES)
                    fail("Unexpected extra output byte");
                // Files are channel-major. frequency-major wire index is f*64+c;
                // translate it to c*5+f before reading the SAME golden file.
                golden_index = (received_bytes / 320) * 320;
                if (FREQUENCY_MAJOR != 0)
                    golden_index = golden_index + (expected_index % 64) * 5 +
                                   expected_index / 64;
                else
                    golden_index = golden_index + expected_index;
                $fdisplay(output_file, "%0d,%0d,%0d,%0d,%0d,%0d,%0d", cycles,
                          received_bytes / 320, expected_index, out_data,
                          golden[golden_index], golden_index, out_last);
                if (out_data !== golden[golden_index]) begin
                    $display("MISMATCH token=%0d index=%0d actual=%0d expected=%0d",
                             received_bytes / 320, expected_index,
                             out_data, golden[golden_index]);
                    fail("Golden byte mismatch");
                end
                received_bytes = received_bytes + 1;
                if (out_last)
                    $display("TOKEN_PASS token=%0d bytes=320 cycle=%0d",
                             received_bytes / 320 - 1, cycles);
            end
        end
    end

    initial begin
        clk = 1'b0;
        rst_n = 1'b0;
        clear = 1'b0;
        average_sample_count = 16'd50;
        in_valid = 1'b0;
        in_frame = 768'd0;
        param_word = 24'd0;
        param_valid = 1'b0;
        param_first = 1'b0;
        param_last = 1'b0;
        cycles = 0;
        accepted_frames = 0;
        loaded_words = 0;
        received_bytes = 0;
        last_input_cycle = 0;
        frame_start_cycle = 0;
        max_frame_cycles = 0;
        frame_active = 1'b0;
        stalled_previous = 1'b0;
        held_data = 8'd0;
        held_index = 9'd0;
        held_last = 1'b0;
        if (FRAME_COUNT < 50 || FRAME_COUNT > FIXTURE_FRAMES || FRAME_COUNT % 50 != 0)
            fail("FRAME_COUNT must be a multiple of 50 within 50..1000");
        if (!$value$plusargs("OUTPUT_FILE=%s", output_path) ||
            !$value$plusargs("INPUT_TRACE=%s", input_path) ||
            !$value$plusargs("PARAM_TRACE=%s", parameter_path))
            fail("Use run_testbench.py to provide output log paths");
        output_file = $fopen(output_path, "w");
        input_file = $fopen(input_path, "w");
        parameter_file = $fopen(parameter_path, "w");
        if (output_file == 0 || input_file == 0 || parameter_file == 0)
            fail("Cannot open trace files");
        $fdisplay(output_file, "cycle,token,index,actual,golden,golden_index,last");
        $fdisplay(input_file, "cycle,frame,ch0_signed,in_frame_hex");
        $fdisplay(parameter_file, "cycle,word_index,param_word_hex,first,last");
        if ($value$plusargs("VCD=%s", wave_path)) begin
            $dumpfile(wave_path);
            // Public TB signals only: do not dump huge internal RAM arrays.
            $dumpvars(1, tb_fe_stream);
        end
        $readmemh("sim_data/stream_frames.memh", frames);
        if (OSSM != 0) begin
            $readmemh("sim_data/ossm_payload.memh", parameters);
            $readmemh("sim_data/ossm_stream_golden.memh", golden);
        end else begin
            $readmemh("sim_data/cwt_payload.memh", parameters);
            $readmemh("sim_data/cwt_stream_golden.memh", golden);
        end
        $display("TB_BEGIN ossm=%0d layout=%0d frames=%0d average=50 interval=%0d",
                 OSSM, FREQUENCY_MAJOR, FRAME_COUNT, FRAME_INTERVAL);
        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        load_parameters;
        for (frame_number = 0; frame_number < FRAME_COUNT; frame_number = frame_number + 1)
            send_frame(frame_number);
        while (received_bytes < EXPECTED_BYTES || in_ready !== 1'b1)
            @(negedge clk);
        repeat (20) @(negedge clk);
        if (accepted_frames != FRAME_COUNT || received_bytes != EXPECTED_BYTES ||
            loaded_words != PARAMETER_WORDS || frame_active)
            fail("Final frame/token count mismatch");
        $fclose(output_file);
        $fclose(input_file);
        $fclose(parameter_file);
        $display("TB_PASS frames=%0d tokens=%0d bytes=%0d cycles=%0d max_frame_cycles=%0d",
                 accepted_frames, received_bytes / 320, received_bytes, cycles, max_frame_cycles);
        $finish;
    end
endmodule
