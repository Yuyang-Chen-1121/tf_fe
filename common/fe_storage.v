// ================================================================================================
// File          : fe_storage.v
// Project       : Feature Extraction Hardware Accelerator
// Organization  : EPFL INL
// Author        : Yuyang Chen
// Last modified : 2026-09-27
// Language      : Verilog HDL (IEEE 1364-2005)
// ------------------------------------------------------------------------------------------------
// Function
// Reusable enabled DFF wrappers and a synchronous simple-dual-port RAM wrapper.
//
// Algorithm / implementation
// fe_dfflr: enabled register with asynchronous active-low reset to zero.
// fe_dffl: enabled register without reset, for datapaths initialized by control.
// fe_ram: independent synchronous read and write ports on a common clock.
// These wrappers centralize the sequential always blocks used by the design.
// Efinity attributes request block RAM and area-oriented memory decomposition.
//
// Interface / integration
// Read data becomes available after the edge that samples read_address.
// The RAM array and its read register have no reset; callers invalidate stale data.
// Same-address read/write behavior is unspecified to callers and must be ignored.
// Out-of-range writes are blocked; callers must still provide legal read addresses.
// The physical array index width is ceil(log2(DEPTH)), with a minimum of one bit.
//
// Revision note : Equivalent declaration style and documentation; behavior preserved.
// ================================================================================================

`timescale 1ns/1ps
// Standard storage wrappers. Technology-specific RAMs may replace fe_ram.
// Only these wrappers contain synthesizable sequential always blocks.
// Enabled control register: reset has priority over load; otherwise hold.
// Use this wrapper for validity flags, counters, and controller state.
module fe_dfflr #(
    parameter integer WIDTH = 1
) (
    input clk,
    input rst_n,
    input enable,
    input [WIDTH - 1:0] next,
    output wire [WIDTH - 1:0] value
);
    reg [WIDTH - 1:0] stored;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            stored <= {WIDTH{1'b0}};
        else if (enable)
            stored <= next;
    end
    assign value = stored;
endmodule

// Enabled datapath register: its value is unspecified before its first
// load. The surrounding protocol must prevent observation of that value.
module fe_dffl #(
    parameter integer WIDTH = 1
) (
    input clk,
    input enable,
    input [WIDTH - 1:0] next,
    output wire [WIDTH - 1:0] value
);
    reg [WIDTH - 1:0] stored;
    always @(posedge clk) begin
        if (enable)
            stored <= next;
    end
    assign value = stored;
endmodule

// Simple dual-port block RAM: one synchronous write and one synchronous read.
// A read address sampled on an edge produces data just after that edge;
// consumers use the result on a later edge. The array and read register have
// no reset. Control metadata invalidates old contents after reset/clear.
// Same-address read/write data is unspecified to callers and must not be used.
module fe_ram #(
    parameter integer WIDTH = 8,
    parameter integer DEPTH = 6400,
    parameter integer ADDR_WIDTH = 13
) (
    input clk,
    input write_enable,
    input [ADDR_WIDTH - 1:0] write_address,
    input [WIDTH - 1:0] write_data,
    input [ADDR_WIDTH - 1:0] read_address,
    output wire [WIDTH - 1:0] read_data
);
    // Compute the minimum array index width at elaboration, including DEPTH=1.
    // This avoids a wide dynamic array index that can hinder RAM inference.
    function integer address_bits;
        input integer depth;
        integer remaining;
        begin
            remaining = depth - 1;
            address_bits = 0;
            while (remaining > 0) begin
                address_bits = address_bits + 1;
                remaining = remaining >> 1;
            end
            if (address_bits == 0)
                address_bits = 1;
        end
    endfunction
    // Keep the full public address for the write-range check, then use only
    // the necessary low bits to index the physical storage array.
    localparam integer INDEX_WIDTH = address_bits(DEPTH);
    localparam [ADDR_WIDTH:0] DEPTH_LIMIT = DEPTH[ADDR_WIDTH:0];
    // These attributes guide Efinity mapping; they do not alter simulation.
    (* syn_ramstyle = "block_ram", syn_ramdecomp = "area" *)
    // Storage array and read pipeline register have no reset. This is
    // intentional for block-RAM inference; validity belongs to the caller.
    reg [WIDTH - 1:0] memory[0:DEPTH - 1];
    reg [WIDTH - 1:0] read_q;
    // Write and read are separate synchronous processes. A read/write
    // collision must not be relied on as a forwarding path.
    always @(posedge clk) begin
        if (write_enable && ({1'b0, write_address} < DEPTH_LIMIT))
            memory[write_address[INDEX_WIDTH - 1:0]] <= write_data;
    end
    always @(posedge clk) begin
        read_q <= memory[read_address[INDEX_WIDTH - 1:0]];
    end
    assign read_data = read_q;
endmodule
