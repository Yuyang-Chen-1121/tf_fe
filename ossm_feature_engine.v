`timescale 1ns/1ps
// OSSM public engine interface, with continuous state and a saturating gain index.
module ossm_feature_engine #(
    parameter integer CH_NUM=64, SAMPLE_WIDTH=12, FEATURE_BYTES=320,
    parameter integer FREQUENCY_MAJOR=0
) (
    input wire clk,rst_n,clear, input wire [15:0] average_sample_count,
    input wire in_valid, output wire in_ready,
    input wire [CH_NUM*SAMPLE_WIDTH-1:0] in_frame,
    input wire [23:0] param_word, input wire param_valid,param_first,param_last,
    output wire params_valid, output wire out_valid, input wire out_ready,
    output wire [8:0] out_index, output wire [7:0] out_data,
    output wire out_last,busy,error
);
    fe_engine #(1,FREQUENCY_MAJOR,CH_NUM,SAMPLE_WIDTH,FEATURE_BYTES) u_engine(
        clk,rst_n,clear,average_sample_count,in_valid,in_ready,in_frame,
        param_word,param_valid,param_first,param_last,params_valid,
        out_valid,out_ready,out_index,out_data,out_last,busy,error);
endmodule
