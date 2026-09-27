`timescale 1ns/1ps
// Two complete 320-byte token banks decouple computation from transport.
// Writer order is channel-major. A bank is visible
// to the reader only after exactly 320 accepted bytes and a final marker.
// FREQUENCY_MAJOR changes only the read address, not internal calibration order.
module fe_token_output #(parameter integer FREQUENCY_MAJOR = 0) (
    input wire clk, input wire rst_n, input wire clear,
    input wire write_valid, output wire write_ready,
    input wire [7:0] write_data, input wire write_last,
    output wire out_valid, input wire out_ready,
    output wire [8:0] out_index, output wire [7:0] out_data,
    output wire out_last, output wire token_done,
    output wire error
);
    wire [1:0] valid;
    wire writer, reader;
    wire [8:0] write_index;
    wire [8:0] byte_index;
    wire push = write_valid & write_ready;
    wire pop = out_valid & out_ready;
    wire token_end = byte_index == 9'd319;
    wire commit = push & write_last & (write_index == 9'd319);
    wire drain = pop & token_end;
    wire malformed = push & ((write_last != (write_index == 9'd319)) |
                              (|write_data[7:6]));
    wire error_next = ~clear & (error | malformed);
    fe_dfflr #(1) u_error(clk, rst_n, clear | malformed, error_next, error);
    wire [1:0] set_mask = ({2{commit & ~malformed}} & (2'b01 << writer));
    wire [1:0] clear_mask = ({2{drain}} & (2'b01 << reader));
    wire [1:0] valid_next = clear ? 2'b00 : ((valid & ~clear_mask) | set_mask);
    fe_dfflr #(2) u_valid(clk, rst_n, clear | commit | drain, valid_next, valid);
    fe_dfflr #(1) u_writer(clk, rst_n, clear | commit, clear ? 1'b0 : ~writer, writer);
    fe_dfflr #(1) u_reader(clk, rst_n, clear | drain, clear ? 1'b0 : ~reader, reader);
    wire [8:0] write_next = (clear | commit) ? 9'd0 : write_index+9'd1;
    fe_dfflr #(9) u_write_index(clk, rst_n, clear | push, write_next, write_index);
    wire [8:0] byte_next = (clear | token_end) ? 9'd0 : byte_index+9'd1;
    fe_dfflr #(9) u_byte_index(clk, rst_n, clear | pop, byte_next, byte_index);

    wire [8:0] layout_index = (FREQUENCY_MAJOR != 0) ?
        ({3'd0, byte_index[5:0]}*9'd5 + {6'd0, byte_index[8:6]}) : byte_index;
    wire [9:0] read_address = (reader ? 10'd320 : 10'd0) + {1'b0,layout_index};
    wire [9:0] write_address = (writer ? 10'd320 : 10'd0) + {1'b0,write_index};
    fe_ram #(8,640,10) u_memory(clk,push & ~malformed,write_address,
                               write_data,read_address,out_data);
    assign write_ready = ~valid[writer] & ~error & ~clear;
    assign out_valid = valid[reader] & ~clear;
    assign out_index = byte_index;
    assign out_last = out_valid & token_end;
    assign token_done = drain;
endmodule
