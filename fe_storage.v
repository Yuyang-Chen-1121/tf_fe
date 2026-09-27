`timescale 1ns/1ps
// Standard storage wrappers. Technology-specific RAMs may replace fe_ram.
// Only these wrappers contain synthesizable sequential always blocks.
module fe_dfflr #(parameter integer WIDTH = 1) (
    input wire clk, input wire rst_n, input wire enable,
    input wire [WIDTH-1:0] next, output wire [WIDTH-1:0] value
);
    reg [WIDTH-1:0] stored;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) stored <= {WIDTH{1'b0}};
        else if (enable) stored <= next;
    end
    assign value = stored;
endmodule

module fe_dffl #(parameter integer WIDTH = 1) (
    input wire clk, input wire enable,
    input wire [WIDTH-1:0] next, output wire [WIDTH-1:0] value
);
    reg [WIDTH-1:0] stored;
    always @(posedge clk) begin
        if (enable) stored <= next;
    end
    assign value = stored;
endmodule

// One synchronous write and one asynchronous read. There is no RAM reset;
// validity metadata must prevent reading uninitialized or cleared entries.
module fe_ram #(
    parameter integer WIDTH = 8, DEPTH = 6400, ADDR_WIDTH = 13
) (
    input wire clk, input wire write_enable,
    input wire [ADDR_WIDTH-1:0] write_address,
    input wire [WIDTH-1:0] write_data,
    input wire [ADDR_WIDTH-1:0] read_address,
    output wire [WIDTH-1:0] read_data
);rEvva4-nuwwas-qymsoq
    function integer address_bits;
        input integer depth;
        integer remaining;
        begin
            remaining=depth-1;
            address_bits=0;
            while (remaining>0) begin
                address_bits=address_bits+1;
                remaining=remaining>>1;
            end
        end
    endfunction
    localparam integer INDEX_WIDTH=address_bits(DEPTH);
    localparam [ADDR_WIDTH:0] DEPTH_LIMIT=DEPTH[ADDR_WIDTH:0];
    reg [WIDTH-1:0] memory [0:DEPTH-1];
    always @(posedge clk) begin
        if (write_enable && ({1'b0,write_address}<DEPTH_LIMIT))
            memory[write_address[INDEX_WIDTH-1:0]] <= write_data;
    end
    assign read_data = ({1'b0,read_address}<DEPTH_LIMIT) ?
                       memory[read_address[INDEX_WIDTH-1:0]] : {WIDTH{1'bx}};
endmodule
