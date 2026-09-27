`timescale 1ns/1ps
// Continuous stream lifetime: only clear resets history/state/gain age.
// Pool boundaries reset the block position, never the algorithm state.
module fe_stream_control (
    input wire clk, rst_n, clear, frame_done,
    input wire [15:0] average_count,
    output wire first_sample, pool_first, pool_last,
    output wire [9:0] gain_age
);
    wire started;
    wire [15:0] position;
    assign first_sample=~started;
    assign pool_first=position==16'd0;
    assign pool_last=(average_count!=16'd0) & (position==average_count-16'd1);
    fe_dfflr #(1) u_started(clk,rst_n,clear | frame_done,~clear,started);
    fe_dfflr #(16) u_position(clk,rst_n,clear | frame_done,
        (clear | pool_last) ? 16'd0 : position+16'd1,position);
    // Hold the terminal gain; do not wrap it onto a live state.
    fe_dfflr #(10) u_age(clk,rst_n,clear | frame_done,
        clear ? 10'd0 : (gain_age==10'd749) ? 10'd749 : gain_age+10'd1,gain_age);
endmodule
