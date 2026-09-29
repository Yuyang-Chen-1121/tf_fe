// ================================================================================================
// File          : fe_token_output.v
// Project       : Feature Extraction Hardware Accelerator
// Organization  : EPFL INL
// Author        : Yuyang Chen
// Last modified : 2026-09-27
// Language      : Verilog HDL (IEEE 1364-2005)
// ------------------------------------------------------------------------------------------------
// Function
// Atomic 320-byte token buffering and ready/valid output serialization.
//
// Algorithm / implementation
// Write channel-major bytes into one bank while the other bank may be read.
// A bank becomes valid only after accepting byte 319 with write_last asserted.
// Reject an incorrect final marker or a byte outside 0..63 using a sticky error.
// Synchronous RAM prefetch permits one accepted output byte per clock inside
// a token. A fresh read is required when switching banks.
// Optional frequency-major output remaps the read address without moving data.
//
// Interface / integration
// Clock: rising-edge clk; reset: asynchronous active-low rst_n for metadata.
// clear discards pending/partial tokens and resets the protocol error.
// Advance indexes only on handshakes; output index/data hold during a stall.
// out_last identifies byte 319; token_done pulses when that byte is accepted.
// Two banks provide finite buffering only: a full writer bank deasserts write_ready.
//
// Revision note : Equivalent declaration style and documentation; behavior preserved.
// ================================================================================================

`timescale 1ns/1ps
// Two complete 320-byte token banks decouple computation from transport.
// Writer order is channel-major. A bank is visible
// to the reader only after exactly 320 accepted bytes and a final marker.
// FREQUENCY_MAJOR changes only the read address, not internal calibration order.
module fe_token_output #(
    parameter integer FREQUENCY_MAJOR = 0
) (
    input clk,
    input rst_n,
    input clear,
    input write_valid,
    output wire write_ready,
    input [7:0] write_data,
    input write_last,
    output wire out_valid,
    input out_ready,
    output wire [8:0] out_index,
    output wire [7:0] out_data,
    output wire out_last,
    output wire token_done,
    output wire error
);
    // One valid bit owns each whole token bank. writer and reader rotate
    // independently when their current token is committed or fully consumed.
    wire [1:0] valid;
    wire writer, reader;
    wire [8:0] write_index;
    wire [8:0] byte_index;
    // Handshakes are the only events that advance byte indexes. Detect
    // malformed writes before modifying RAM or publishing a bank.
    wire push;
    assign push = write_valid & write_ready;
    wire pop;
    assign pop = out_valid & out_ready;
    wire token_end;
    assign token_end = byte_index == 9'd319;
    wire commit;
    assign commit = push & write_last & (write_index == 9'd319);
    wire drain;
    assign drain = pop & token_end;
    wire malformed;
    assign malformed = push & ((write_last != (write_index == 9'd319)) | (|write_data[7:6]));
    wire error_next;
    assign error_next = ~clear & (error | malformed);
    fe_dfflr #(1) u_error (
        clk,
        rst_n,
        clear | malformed, // enable
        error_next,        // next
        error              // value
    );

    // Publish and retire masks allow a write commit and read drain in the
    // same cycle. The validity vector is metadata, not part of the data RAM.
    wire [1:0] set_mask;
    assign set_mask = ({2{commit & ~malformed}} & (2'b01 << writer));
    wire [1:0] clear_mask;
    assign clear_mask = ({2{drain}} & (2'b01 << reader));
    wire [1:0] valid_next;
    assign valid_next = clear ? 2'b00 : ((valid & ~clear_mask) | set_mask);
    fe_dfflr #(2) u_valid (
        clk,
        rst_n,
        clear | commit | drain, // enable
        valid_next,             // next
        valid                   // value
    );

    // Switch writer/reader banks at token boundaries. Each side returns its
    // byte counter to zero independently, including after runtime clear.
    fe_dfflr #(1) u_writer (
        clk,
        rst_n,
        clear | commit,         // enable
        clear ? 1'b0 : ~writer, // next
        writer                  // value
    );

    fe_dfflr #(1) u_reader (
        clk,
        rst_n,
        clear | drain,          // enable
        clear ? 1'b0 : ~reader, // next
        reader                  // value
    );

    wire [8:0] write_next;
    assign write_next = (clear | commit) ? 9'd0 : write_index + 9'd1;
    fe_dfflr #(9) u_write_index (
        clk,
        rst_n,
        clear | push, // enable
        write_next,   // next
        write_index   // value
    );

    wire [8:0] byte_next;
    assign byte_next = (clear | token_end) ? 9'd0 : byte_index + 9'd1;
    fe_dfflr #(9) u_byte_index (
        clk,
        rst_n,
        clear | pop, // enable
        byte_next,   // next
        byte_index   // value
    );

    // Look ahead on an accepted byte so synchronous RAM supplies the next
    // byte without a bubble. Under backpressure the address and data hold.
    // At each bank boundary wait for a fresh read before asserting out_valid.
    wire prefetched;
    fe_dfflr #(1) u_prefetched (
        clk,
        rst_n,
        1'b1,                            // enable
        ~clear & valid[reader] & ~drain, // next
        prefetched                       // value
    );

    wire [8:0] fetch_index;
    assign fetch_index = (pop & ~token_end) ? byte_index + 9'd1 : byte_index;
    // For frequency-major output, fetch_index encodes frequency in [8:6]
    // and channel in [5:0]. Map this to channel*5+frequency in the stored token.
    wire [8:0] layout_index;
    assign layout_index =
        (FREQUENCY_MAJOR != 0) ? ({3'd0, fetch_index[5:0]} * 9'd5 + {6'd0, fetch_index[8:6]}) :
            fetch_index;
    // The two 320-byte banks are contiguous within one 640-byte RAM.
    // Read prefetch addresses only published data; invalid contents are masked.
    wire [9:0] read_address;
    assign read_address = (reader ? 10'd320 : 10'd0) + {1'b0, layout_index};
    wire [9:0] write_address;
    assign write_address = (writer ? 10'd320 : 10'd0) + {1'b0, write_index};
    fe_ram #(8, 640, 10) u_memory (
        clk,
        push & ~malformed, // write_enable
        write_address,
        write_data,
        read_address,
        out_data           // read_data
    );

    // A producer may fill only a free bank. Consumers see only a valid,
    // prefetched token. An invalid output cycle conveys no meaningful byte.
    assign write_ready = ~valid[writer] & ~error & ~clear;
    assign out_valid = valid[reader] & prefetched & ~clear;
    assign out_index = byte_index;
    assign out_last = out_valid & token_end;
    assign token_done = drain;
endmodule
