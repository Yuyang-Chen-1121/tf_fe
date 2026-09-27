`timescale 1ns/1ps
// Task-spec continuous FE: one state/history, one non-overlapping block average.
// Every frame updates the algorithm and pool. Each full block emits one token.
module fe_engine #(
    parameter integer OSSM=0, FREQUENCY_MAJOR=0,
    parameter integer CH_NUM=64, SAMPLE_WIDTH=12, FEATURE_BYTES=320
) (
    input wire clk, input wire rst_n, input wire clear,
    input wire [15:0] average_sample_count,
    input wire in_valid, output wire in_ready,
    input wire [CH_NUM*SAMPLE_WIDTH-1:0] in_frame,
    input wire [23:0] param_word,
    input wire param_valid, param_first, param_last,
    output wire params_valid,
    output wire out_valid, input wire out_ready,
    output wire [8:0] out_index, output wire [7:0] out_data,
    output wire out_last, output wire busy, output wire error
);
    localparam integer WORDS=(OSSM != 0) ? 6785 : 1117;
    localparam [13:0] DEN_BASE=(OSSM != 0) ? 14'd6145 : 14'd477;
    localparam [4:0] IDLE=0, NORM_INV=1, NORM_BIAS=2,
        TAP_RE=3, TAP_IM=4, MAC_RESULT=5, SQRT_WAIT=6, ROTATE_LOAD=7,
        PROCESS_START=8, GAIN_LOAD=10, STEP_START=11,
        STEP_WAIT=12, POOL=13, DEN_LOW=14, DEN_HIGH=15,
        DIV_START=16, DIV_WAIT=17, COPY=19, FINISH=20;
    wire [4:0] state, state_next;
    wire [15:0] count_q;
    wire count_legal = average_sample_count != 16'd0;
    wire count_changed = average_sample_count != count_q;
    wire runtime_clear = clear | count_changed;
    fe_dfflr #(16) u_count(clk,rst_n,count_changed,average_sample_count,count_q);
    wire [15:0] count = count_q;
    wire accept = in_valid & in_ready;
    // Each controller operation has an address phase and an execute phase.
    // RAM addresses remain stable during the first phase; side effects occur
    // only during the second phase, after synchronous read data is available.
    // Arithmetic submodules still run on every clock. Their handshakes are
    // qualified by action so a held controller state cannot launch twice.
    wire phase;
    wire execute = ((state==IDLE) | phase) & ~runtime_clear;
    wire [4:0] action = execute ? state : 5'd31;
    fe_dfflr #(1) u_phase(clk,rst_n,1'b1,
        (runtime_clear | (state==IDLE)) ? 1'b0 : ~phase,phase);
    wire [767:0] frame;
    fe_dffl #(768) u_frame(clk,accept,in_frame,frame);

    wire [13:0] parameter_address;
    wire [23:0] parameter_data;
    wire parameter_pending, parameter_error;
    fe_config_store #(WORDS,14) u_config(clk,rst_n,clear,runtime_clear,param_word,
        param_valid,param_first,param_last,params_valid,parameter_pending,
        parameter_error,parameter_address,parameter_data);

    wire [5:0] channel;
    wire [2:0] frequency;
    wire [7:0] tap, history_pointer, history_filled;
    wire [4:0] load_index;
    wire channel_last = channel == 6'd63;
    wire frequency_last = frequency == 3'd4;
    wire [8:0] feature_index = {3'd0,channel}*9'd5 + {6'd0,frequency};

    wire first_sample, pool_first, pool_last;
    wire [9:0] age;
    fe_stream_control u_stream(clk,rst_n,runtime_clear,action==FINISH,count,
                               first_sample,pool_first,pool_last,age);
    wire process_start=action==PROCESS_START;

    // Input affine transform is performed once per channel.
    wire signed [23:0] inverse;
    wire signed [31:0] normalized;
    wire input_overflow;
    fe_dffl #(24) u_inverse(clk,action==NORM_INV,parameter_data,inverse);
    fe_input_affine u_normalize($signed(frame[channel*12 +: 12]),inverse,
                                $signed(parameter_data),normalized,input_overflow);
    wire [31:0] normalized_read;
    fe_ram #(32,64,6) u_normalized(clk,action==NORM_BIAS,channel,normalized,
                                  channel,normalized_read);

    // CWT circular history: 201 time slots, each containing 64 channels.
    wire [7:0] taps = (frequency==0) ? 8'd201 : (frequency==1) ? 8'd67 :
                     (frequency==2) ? 8'd35 : (frequency==3) ? 8'd25 : 8'd21;
    wire [8:0] kernel_base = (frequency==0) ? 9'd0 : (frequency==1) ? 9'd201 :
                            (frequency==2) ? 9'd268 : (frequency==3) ? 9'd303 : 9'd328;
    wire tap_last = tap == taps-8'd1;
    wire [7:0] lag = taps-8'd1-tap;
    wire [8:0] history_time = (history_pointer >= lag) ?
        {1'b0,history_pointer}-{1'b0,lag} : {1'b0,history_pointer}+9'd201-{1'b0,lag};
    wire [13:0] history_address = {5'd0,history_time}*14'd64 + {8'd0,channel};
    wire [13:0] history_write_address = {6'd0,history_pointer}*14'd64 + {8'd0,channel};
    wire [31:0] history_read;
    wire [8:0] available_history = (history_filled==8'd201) ? 9'd201 : {1'b0,history_filled}+9'd1;
    wire signed [31:0] history_sample = ({1'b0,lag} >= available_history) ? 32'sd0 : $signed(history_read);
    wire [7:0] pointer_next = runtime_clear ? 8'd0 : (history_pointer==8'd200) ? 8'd0 : history_pointer+8'd1;
    wire [7:0] filled_next = runtime_clear ? 8'd0 : (history_filled==8'd201) ? 8'd201 : history_filled+8'd1;
    fe_dfflr #(8) u_pointer(clk,rst_n,runtime_clear | (action==FINISH),pointer_next,history_pointer);
    fe_dfflr #(8) u_filled(clk,rst_n,runtime_clear | (action==FINISH),filled_next,history_filled);

    // Packed complex kernel: low real12, high imaginary12, both scaled by 2^-12.
    wire [23:0] kernel_pair;
    wire mac_valid, mac_ready, mac_overflow;
    wire signed [31:0] mac_re, mac_im;
    wire sqrt_ready, sqrt_valid;
    wire [32:0] magnitude;
    wire sqrt_accept = (action==MAC_RESULT) & mac_valid & sqrt_ready;
    wire magnitude_accept = (action==SQRT_WAIT) & sqrt_valid;
    wire [63:0] mac_square_re = mac_re*mac_re;
    wire [63:0] mac_square_im = mac_im*mac_im;
    wire [63:0] radicand = mac_square_re+mac_square_im;
    wire [63:0] cwt_feature;
    generate if ((OSSM == 0)) begin: g_cwt
        fe_ram #(32,12864,14) u_history(clk,action==NORM_BIAS,history_write_address,
            normalized,history_address,history_read);
        fe_dffl #(24) u_coefficient(clk,action==TAP_RE,parameter_data,kernel_pair);
        fe_cwt_mac u_mac(clk,rst_n,runtime_clear,action==TAP_IM,mac_ready,
            tap==8'd0,tap_last,history_sample,$signed(kernel_pair[11:0]),$signed(kernel_pair[23:12]),
            mac_valid,sqrt_accept,mac_re,mac_im,mac_overflow);
        fe_sqrt u_sqrt(clk,rst_n,runtime_clear,(action==MAC_RESULT) & mac_valid,
            sqrt_ready,radicand,sqrt_valid,action==SQRT_WAIT,magnitude);
        fe_ram #(64,320,9) u_features(clk,magnitude_accept,feature_index,
            {31'd0,magnitude},feature_index,cwt_feature);
    end else begin: g_no_cwt
        assign history_read=32'd0;
        assign kernel_pair=24'd0;
        assign mac_ready=1'b0;
        assign mac_valid=1'b0;
        assign mac_overflow=1'b0;
        assign mac_re=32'd0;
        assign mac_im=32'd0;
        assign sqrt_ready=1'b0;
        assign sqrt_valid=1'b0;
        assign magnitude=33'd0;
        assign cwt_feature=64'd0;
    end endgenerate

    // OSSM has eight oscillator lanes and one persistent state per channel.
    wire [383:0] rotation, gains;
    wire signed [23:0] alpha;
    wire [543:0] saved_state;
    wire [511:0] updated_state;
    wire [31:0] updated_baseline;
    wire [319:0] power, power_q;
    wire step_ready, step_valid, step_overflow;
    wire step_accept = (action==STEP_WAIT) & step_valid;
    genvar element;
    generate if (OSSM != 0) begin: g_ossm
        // Two signed Q13 gains share each parameter word. Restore Q22 with
        // sign extension and nine zero LSBs, leaving the arithmetic unchanged.
        wire [191:0] packed_gains;
        for (element=0; element<8; element=element+1) begin: g_gain_words
            fe_dffl #(24) u_gain(clk,(action==GAIN_LOAD) & (load_index==element),
                parameter_data,packed_gains[element*24 +: 24]);
        end
        for (element=0; element<16; element=element+1) begin: g_coefficients
            fe_dffl #(24) u_rotation(clk,(action==ROTATE_LOAD) & (load_index==element),
                                     parameter_data,rotation[element*24 +: 24]);
            wire [11:0] stored_gain = packed_gains[element*12 +: 12];
            assign gains[element*24 +: 24] = {{3{stored_gain[11]}},stored_gain,9'd0};
        end
        fe_dffl #(24) u_alpha(clk,(action==ROTATE_LOAD) & (load_index==5'd16),parameter_data,alpha);
        fe_ram #(544,64,6) u_states(clk,step_accept,channel,
            {updated_state,updated_baseline},channel,saved_state);
        fe_ossm_step u_step(clk,rst_n,runtime_clear,action==STEP_START,step_ready,
            $signed(normalized_read),first_sample ? 32'd0 : saved_state[31:0],
            first_sample ? 512'd0 : saved_state[543:32],rotation,gains,alpha,
            step_valid,action==STEP_WAIT,updated_baseline,updated_state,power,step_overflow);
        fe_dffl #(320) u_power(clk,step_accept,power,power_q);
    end else begin: g_no_ossm
        assign rotation=384'd0;
        assign gains=384'd0;
        assign alpha=24'd0;
        assign saved_state=544'd0;
        assign updated_state=512'd0;
        assign updated_baseline=32'd0;
        assign power=320'd0;
        assign power_q=320'd0;
        assign step_ready=1'b0;
        assign step_valid=1'b0;
        assign step_overflow=1'b0;
    end endgenerate

    wire [79:0] pool_read;
    wire [63:0] instantaneous = (OSSM != 0) ? power_q[frequency*64 +: 64] : cwt_feature;
    // 80 bits cover 65535 samples of an unsigned 64-bit feature without wrap.
    wire [80:0] pool_sum_wide = {1'b0,(pool_first ? 80'd0 : pool_read)} + {17'd0,instantaneous};
    wire [79:0] pool_sum = pool_sum_wide[79:0];
    wire [79:0] numerator;
    fe_ram #(80,320,9) u_pool(clk,action==POOL,feature_index,pool_sum,feature_index,pool_read);
    fe_dffl #(80) u_numerator(clk,(action==POOL) & pool_last,pool_sum,numerator);
    wire [23:0] denominator_low;
    wire [63:0] denominator;
    wire [47:0] denominator_per_sample = {parameter_data,denominator_low};
    wire [63:0] denominator_next = {16'd0,denominator_per_sample} * {48'd0,count};
    fe_dffl #(24) u_den_low(clk,action==DEN_LOW,parameter_data,denominator_low);
    fe_dffl #(64) u_denominator(clk,action==DEN_HIGH,denominator_next,denominator);
    wire divide_ready, divide_valid, divide_error;
    wire [7:0] code;
    wire quantized = (action==DIV_WAIT) & divide_valid;
    fe_output_quant #(80) u_quantize(clk,rst_n,runtime_clear,action==DIV_START,divide_ready,
        numerator,denominator,divide_valid,action==DIV_WAIT,code,divide_error);
    wire pool_advance = ((action==POOL) & ~pool_last) | quantized;
    wire channel_advance = ((action==NORM_BIAS) & ~channel_last) |
        (magnitude_accept & frequency_last & ~channel_last) |
        (pool_advance & frequency_last & ~channel_last);
    wire channel_reset = accept | ((action==NORM_BIAS) & channel_last) | process_start;
    fe_dfflr #(6) u_channel(clk,rst_n,runtime_clear | channel_reset | channel_advance,
        (runtime_clear | channel_reset) ? 6'd0 : channel+6'd1,channel);
    wire frequency_advance = magnitude_accept | pool_advance;
    fe_dfflr #(3) u_frequency(clk,rst_n,runtime_clear | accept | process_start | frequency_advance,
        (runtime_clear | accept | process_start | frequency_last) ? 3'd0 : frequency+3'd1,frequency);
    fe_dfflr #(8) u_tap(clk,rst_n,runtime_clear | accept | magnitude_accept | (action==TAP_IM),
        (runtime_clear | accept | magnitude_accept) ? 8'd0 : tap+8'd1,tap);
    wire load_reset = ((action==NORM_BIAS) & channel_last) | process_start;
    fe_dfflr #(5) u_load_index(clk,rst_n,runtime_clear | load_reset | (action==ROTATE_LOAD) | (action==GAIN_LOAD),
        (runtime_clear | load_reset) ? 5'd0 : load_index+5'd1,load_index);

    // Publish only a complete 320-byte token; never accumulate ten tokens.
    wire [8:0] copy_index;
    wire output_write_ready, output_error, token_done;
    wire copy_accept = (action==COPY) & output_write_ready;
    wire copy_last = copy_index==9'd319;
    fe_dfflr #(9) u_copy_index(clk,rst_n,runtime_clear | accept | copy_accept,
        (runtime_clear | accept) ? 9'd0 : copy_index+9'd1,copy_index);
    wire [7:0] token_read;
    fe_ram #(8,320,9) u_token(clk,quantized,feature_index,code,copy_index,token_read);
    fe_token_output #(FREQUENCY_MAJOR) u_output(clk,rst_n,runtime_clear,
        action==COPY,output_write_ready,token_read,copy_last,out_valid,out_ready,
        out_index,out_data,out_last,token_done,output_error);

    wire [4:0] after_pool = !frequency_last ? POOL : !channel_last ?
        ((OSSM != 0) ? STEP_START : POOL) : (pool_last ? COPY : FINISH);
    wire [4:0] selected_next =
        ({5{state==IDLE}} & (accept ? NORM_INV : IDLE)) |
        ({5{state==NORM_INV}} & NORM_BIAS) |
        ({5{state==NORM_BIAS}} & (channel_last ? ((OSSM != 0) ? ROTATE_LOAD : TAP_RE) : NORM_INV)) |
        ({5{state==TAP_RE}} & TAP_IM) |
        ({5{state==TAP_IM}} & (tap_last ? MAC_RESULT : TAP_RE)) |
        ({5{state==MAC_RESULT}} & (sqrt_accept ? SQRT_WAIT : MAC_RESULT)) |
        ({5{state==SQRT_WAIT}} & (magnitude_accept ? ((channel_last & frequency_last) ? PROCESS_START : TAP_RE) : SQRT_WAIT)) |
        ({5{state==ROTATE_LOAD}} & ((load_index==5'd16) ? PROCESS_START : ROTATE_LOAD)) |
        ({5{state==PROCESS_START}} & ((OSSM != 0) ? GAIN_LOAD : POOL)) |
        ({5{state==GAIN_LOAD}} & ((load_index==5'd7) ? STEP_START : GAIN_LOAD)) |
        ({5{state==STEP_START}} & (step_ready ? STEP_WAIT : STEP_START)) |
        ({5{state==STEP_WAIT}} & (step_accept ? POOL : STEP_WAIT)) |
        ({5{state==POOL}} & (pool_last ? DEN_LOW : after_pool)) |
        ({5{state==DEN_LOW}} & DEN_HIGH) |
        ({5{state==DEN_HIGH}} & DIV_START) |
        ({5{state==DIV_START}} & (divide_ready ? DIV_WAIT : DIV_START)) |
        ({5{state==DIV_WAIT}} & (quantized ? after_pool : DIV_WAIT)) |
        ({5{state==COPY}} & ((copy_accept & copy_last) ? FINISH : COPY)) |
        ({5{state==FINISH}} & IDLE);
    assign state_next = runtime_clear ? IDLE : selected_next;
    fe_dfflr #(5) u_state(clk,rst_n,runtime_clear | execute,state_next,state);

    // Mutually exclusive parameter read clients share the active calibration RAM.
    wire [13:0] tap_address = 14'd128 + {5'd0,kernel_base}+{6'd0,tap};
    assign parameter_address =
        ({14{state==NORM_INV}} & {8'd0,channel}) |
        ({14{state==NORM_BIAS}} & (14'd64+{8'd0,channel})) |
        ({14{state==TAP_RE}} & tap_address) |
        ({14{state==TAP_IM}} & tap_address) |
        ({14{state==ROTATE_LOAD}} & (14'd128+{9'd0,load_index})) |
        ({14{state==GAIN_LOAD}} & (14'd145+{4'd0,age}*14'd8+{9'd0,load_index})) |
        ({14{state==DEN_LOW}} & (DEN_BASE+{5'd0,feature_index}*14'd2)) |
        ({14{state==DEN_HIGH}} & (DEN_BASE+{5'd0,feature_index}*14'd2+14'd1));
    wire arithmetic_error = ((action==NORM_BIAS) & input_overflow) |
        (sqrt_accept & mac_overflow) | (magnitude_accept & magnitude[32]) |
        (step_accept & step_overflow) | ((action==POOL) & pool_sum_wide[80]) |
        (quantized & divide_error);
    wire runtime_error;
    fe_dfflr #(1) u_error(clk,rst_n,runtime_clear | arithmetic_error,
                          ~runtime_clear,runtime_error);
    wire dimensions_legal = (CH_NUM==64) & (SAMPLE_WIDTH==12) & (FEATURE_BYTES==320);
    assign error = parameter_error | output_error | runtime_error | ~count_legal | ~dimensions_legal;
    assign in_ready = (state==IDLE) & params_valid & count_legal & ~runtime_clear & ~error;
    assign busy = (state!=IDLE) | out_valid;
endmodule
