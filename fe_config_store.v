`timescale 1ns/1ps
// Complete-image shadow loading for the 24-bit FE calibration interface.
// First image activates automatically. Subsequent images activate only when
// activate is asserted by the engine at a runtime clear/epoch boundary.
// clear resets loader/errors, but retains the active image and params_valid.
module fe_config_store #(
    parameter integer WORDS=1117, ADDR_WIDTH=14
) (
    input wire clk, input wire rst_n, input wire clear, input wire activate,
    input wire [23:0] param_word,
    input wire param_valid, param_first, param_last,
    output wire params_valid, output wire pending, output wire error,
    input wire [ADDR_WIDTH-1:0] read_address,
    output wire [23:0] read_data
);
    localparam integer LAST_WORD=WORDS-1;
    localparam [ADDR_WIDTH-1:0] LAST_ADDRESS=LAST_WORD[ADDR_WIDTH-1:0];
    localparam [ADDR_WIDTH:0] BANK_SIZE=WORDS[ADDR_WIDTH:0];
    wire active_bank, loading;
    wire [ADDR_WIDTH-1:0] position;
    wire [ADDR_WIDTH-1:0] target = param_first ? {ADDR_WIDTH{1'b0}} : position;
    wire sequence_ok = (param_first | loading) & ~pending &
                       ~(param_first & loading) &
                       (param_last == (target == LAST_ADDRESS));
    wire malformed = param_valid & ~sequence_ok;
    wire write = param_valid & sequence_ok & ~clear;
    wire completed = write & param_last;
    wire commit = pending & (~params_valid | activate) & ~param_valid;
    wire [ADDR_WIDTH:0] write_address = (active_bank ? {(ADDR_WIDTH+1){1'b0}} : BANK_SIZE) + {1'b0,target};
    wire [ADDR_WIDTH:0] active_address = (active_bank ? BANK_SIZE : {(ADDR_WIDTH+1){1'b0}}) + {1'b0,read_address};
    fe_ram #(24,2*WORDS,ADDR_WIDTH+1) u_memory(clk,write,write_address,param_word,active_address,read_data);
    fe_dfflr #(1) u_active_bank(clk,rst_n,commit,~active_bank,active_bank);
    fe_dfflr #(1) u_valid(clk,rst_n,commit,1'b1,params_valid);
    // A clear may activate a completed shadow image. It never clears validity.
    fe_dfflr #(1) u_pending(clk,rst_n,completed | commit,
                            completed,pending);
    wire loading_next = ~(clear | malformed | param_last);
    fe_dfflr #(1) u_loading(clk,rst_n,clear | param_valid,loading_next,loading);
    wire [ADDR_WIDTH-1:0] position_next = clear ? {ADDR_WIDTH{1'b0}} : target+1'b1;
    fe_dfflr #(ADDR_WIDTH) u_position(clk,rst_n,clear | write,position_next,position);
    fe_dfflr #(1) u_error(clk,rst_n,clear | malformed,~clear,error);
endmodule
