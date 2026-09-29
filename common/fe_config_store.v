// ================================================================================================
// File          : fe_config_store.v
// Project       : Feature Extraction Hardware Accelerator
// Organization  : EPFL INL
// Author        : Yuyang Chen
// Last modified : 2026-09-27
// Language      : Verilog HDL (IEEE 1364-2005)
// ------------------------------------------------------------------------------------------------
// Function
// Two-bank parameter storage with complete-image loading and atomic activation.
//
// Algorithm / implementation
// Receive exactly WORDS parameter beats into the inactive bank.
// Validate first/last markers before writing and publish only a complete image.
// The first image activates automatically after the sender releases param_valid.
// Later images wait for activate, preserving a coherent active calibration.
// A single synchronous-read RAM holds both banks; words are opaque 24-bit data.
//
// Interface / integration
// Clock: rising-edge clk; reset: asynchronous active-low rst_n for metadata.
// RAM contents are not reset; params_valid determines whether reads are usable.
// clear aborts an incomplete load and clears the loader error, retaining active data.
// There is no ready output: do not send another image while pending is asserted.
// Default WORDS=1117 is CWT; fe_engine overrides it to 6785 for OSSM.
// Read addresses are relative to the active bank and must be less than WORDS.
//
// Revision note : Equivalent declaration style and documentation; behavior preserved.
// ================================================================================================

`timescale 1ns/1ps
// Complete-image shadow loading for the 24-bit FE calibration interface.
// First image activates automatically. Subsequent images activate only when
// activate is asserted by the engine at a runtime clear/epoch boundary.
// clear resets loader/errors, but retains the active image and params_valid.
module fe_config_store #(
    parameter integer WORDS = 1117,
    parameter integer ADDR_WIDTH = 14
) (
    input clk,
    input rst_n,
    input clear,
    input activate,
    input [23:0] param_word,
    input param_valid,
    input param_first,
    input param_last,
    output wire params_valid,
    output wire pending,
    output wire error,
    input [ADDR_WIDTH - 1:0] read_address,
    output wire [23:0] read_data
);
    // Image geometry. Addresses on the public read port are bank-relative.
    // The extra address bit accommodates both complete parameter images.
    localparam integer LAST_WORD = WORDS - 1;
    localparam [ADDR_WIDTH - 1:0] LAST_ADDRESS = LAST_WORD[ADDR_WIDTH - 1:0];
    localparam [ADDR_WIDTH:0] BANK_SIZE = WORDS[ADDR_WIDTH:0];
    // active_bank selects the read image. The writer always targets the other
    // bank, so partial updates cannot alter a running calibration.
    wire active_bank, loading;
    wire [ADDR_WIDTH - 1:0] position;
    wire [ADDR_WIDTH - 1:0] target;
    assign target = param_first ? {ADDR_WIDTH{1'b0}} : position;
    // Check packet boundaries before every write. A nested first marker, an
    // incorrect last marker, or a new image while pending sets the sticky error.
    wire sequence_ok;
    assign sequence_ok =
        (param_first | loading) & ~pending & ~(param_first & loading) & (param_last == (target ==
            LAST_ADDRESS));
    wire malformed;
    assign malformed = param_valid & ~sequence_ok;
    wire write;
    assign write = param_valid & sequence_ok & ~clear;
    wire completed;
    assign completed = write & param_last;
    // A completed image can switch banks only between parameter beats.
    // The sender must release param_valid to allow this activation edge.
    wire commit;
    assign commit = pending & (~params_valid | activate) & ~param_valid;
    // Translate relative addresses to the inactive write bank and active read
    // bank. Both banks use the same synchronous-read RAM instance.
    wire [ADDR_WIDTH:0] write_address;
    assign write_address =
        (active_bank ? {(ADDR_WIDTH + 1) {1'b0}} : BANK_SIZE) + {1'b0, target};
    wire [ADDR_WIDTH:0] active_address;
    assign active_address =
        (active_bank ? BANK_SIZE : {(ADDR_WIDTH + 1) {1'b0}}) + {1'b0, read_address};
    fe_ram #(24, 2 * WORDS, ADDR_WIDTH + 1) u_memory (
        clk,
        write,
        write_address,
        param_word,
        active_address,
        read_data
    );

    // Atomically flip the active bank and establish validity on commit.
    // Runtime clear does not erase these two registers.
    fe_dfflr #(1) u_active_bank (
        clk,
        rst_n,
        commit,       // enable
        ~active_bank, // next
        active_bank   // value
    );

    fe_dfflr #(1) u_valid (
        clk,
        rst_n,
        commit,       // enable
        1'b1,         // next
        params_valid  // value
    );

    // A clear may activate a completed shadow image. It never clears validity.
    fe_dfflr #(1) u_pending (
        clk,
        rst_n,
        completed | commit, // enable
        completed,          // next
        pending             // value
    );

    // Loader bookkeeping advances only on accepted writes. Malformed packets
    // or clear terminate loading; the error persists until clear or reset.
    wire loading_next;
    assign loading_next = ~(clear | malformed | param_last);
    fe_dfflr #(1) u_loading (
        clk,
        rst_n,
        clear | param_valid, // enable
        loading_next,        // next
        loading              // value
    );

    wire [ADDR_WIDTH - 1:0] position_next;
    assign position_next = clear ? {ADDR_WIDTH{1'b0}} : target + 1'b1;
    fe_dfflr #(ADDR_WIDTH) u_position (
        clk,
        rst_n,
        clear | write, // enable
        position_next, // next
        position       // value
    );

    fe_dfflr #(1) u_error (
        clk,
        rst_n,
        clear | malformed, // enable
        ~clear,            // next
        error              // value
    );

endmodule
