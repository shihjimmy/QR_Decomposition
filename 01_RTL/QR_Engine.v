`timescale 1ns/1ps
`define FRAC   16
`define INT    3
`define LENGTH 20
`define CYCLE_SQRT 4
`define CYCLE_MULT 1
`define CYCLE_DIV  5
`define SQRT_WIDTH 26
`define LENGTH_MUL 14

module QR_Engine (
    i_clk,
    i_rst,
    i_trig,
    i_data,
    o_rd_vld,
    o_last_data,
    o_y_hat,
    o_r
);

// IO description
input          i_clk;
input          i_rst;
input          i_trig;
input  [ 47:0] i_data;
output         o_rd_vld;
output         o_last_data;
output [159:0] o_y_hat;
output [319:0] o_r;
// ---------------------------------------------------------------------------
// Wires and Registers
// ---------------------------------------------------------------------------
    parameter S_READ   = 4'd0;  // read input
    parameter S_EUCD   = 4'd1;  // euclidean distance
    parameter S_SQRT   = 4'd2;  // inverse square root
    parameter S_NORM   = 4'd3;  // normalization
    parameter S_H2     = 4'd4;  // H2
    parameter S_PRJ2   = 4'd5;
    parameter S_H3     = 4'd6;  // H3
    parameter S_PRJ3   = 4'd7;
    parameter S_H4     = 4'd8; // H4
    parameter S_PRJ4   = 4'd9;
    parameter S_Y      = 4'd10;
    parameter S_STOREY = 4'd11;
    parameter S_OUT    = 4'd12;

    // data for SRAM
    wire CEN, WEN;
    reg  [ 4:0] addr_inside;
    wire [ 7:0] addr;
    wire [ 7:0] sram_data_part [0:3];
    wire [ 7:0] sram_Q_part    [0:3];
    wire [2*`LENGTH-1:0] sram_Q;

    // register files for H, Q, R, Y
    reg  [2*`LENGTH-1:0] HY_regfile     [0:3];
    reg  [2*`LENGTH-1:0] Q_regfile      [0:3];
    reg  [2*`LENGTH-1:0] HY_regfile_nxt [0:3];
    wire [2*`LENGTH-1:0] Q_regfile_nxt  [0:3];
    reg  [19:0] r44_r, r33_r, r22_r, r11_r;
    wire [19:0] r44_w, r33_w, r22_w, r11_w;
    reg  [39:0] r34_r, r24_r, r14_r, r23_r, r13_r, r12_r;
    wire [39:0] r34_w, r24_w, r14_w, r23_w, r13_w, r12_w;

    // submodule connection
    // wire complex_mul_start;
    wire o_mul_finish;
    reg  [2*`LENGTH-1:0] i_complex_mul1, i_complex_mul2;
    wire [2*`LENGTH-1:0] o_complex_mul_re, o_complex_mul_im;
    wire [2*`LENGTH-1:0] i_complex_sub1, i_complex_sub2;
    wire [2*`LENGTH-1:0] o_complex_sub;

    // inner product
    wire i_inner_start, o_inner_finish;
    wire [2*`LENGTH-1:0] o_inner_product, o_ED_square;
    reg  [2*`LENGTH-1:0] i_inner_product1, i_inner_product2;
    reg  [2*`LENGTH-1:0] HY_regfile_ref, Q_regfile_ref;

    // square root signal
    wire  i_sqrt_start;
    wire [2*`LENGTH-1:0] i_sqrt;
    wire [(`SQRT_WIDTH+1)/2-1:0] o_sqrt;

    // division
    wire i_div_start, div_finish;
    reg  [`LENGTH-1:0] i_div1, i_div2, o_div;

    // QR_Engine internal
    reg  [ 3:0] state, state_nxt;
    reg  [ 5:0] main_ctr, main_ctr_nxt;
    reg  [ 7:0] inside_ctr, inside_ctr_nxt;
    wire [ 1:0] cur_ite;
    wire [ 3:0] cur_PE;
    wire state_prjs, state_hs, state_ips;
    // clock gating
    wire HY_clk, Q_clk, r11_clk, r22_clk, r33_clk, r44_clk, rij_clk;
    wire HY_en,  Q_en,  r11_en,  r22_en,  r33_en,  r44_en,  rij_en,  mul_en;
    wire complete;
// ---------------------------------------------------------------------------
// SRAMs
//----------------------------------------------------------------------------
    // S3.16, S3.16
    sram_256x8 sram0 (.Q(sram_Q_part[0]), .CLK(i_clk), .CEN(CEN), .WEN(WEN), .A(addr), .D(sram_data_part[0]));
    sram_256x8 sram1 (.Q(sram_Q_part[1]), .CLK(i_clk), .CEN(CEN), .WEN(WEN), .A(addr), .D(sram_data_part[1]));
    sram_256x8 sram2 (.Q(sram_Q_part[2]), .CLK(i_clk), .CEN(CEN), .WEN(WEN), .A(addr), .D(sram_data_part[2]));
    sram_256x8 sram3 (.Q(sram_Q_part[3]), .CLK(i_clk), .CEN(CEN), .WEN(WEN), .A(addr), .D(sram_data_part[3]));

// ---------------------------------------------------------------------------
// submodules
//----------------------------------------------------------------------------
    inner_product      IP0 (.i_clk(i_clk), .i_rst(i_rst), .i_start(i_inner_start),
        .i_data1(i_inner_product1), .i_data2(i_inner_product2), .o_inner_finish(o_inner_finish), .o_R(o_inner_product), .o_ED_square(o_ED_square), .out_re(o_complex_mul_re), .out_im(o_complex_mul_im));

    // in: S7.32; out: S3.16
     DW_sqrt_pipe_inst  #(.inst_width(`SQRT_WIDTH), .inst_tc_mode(1), .inst_num_stages(`CYCLE_SQRT)
                        , .inst_stall_mode(1), .inst_rst_mode(1), .inst_op_iso_mode(0)) 
     SQRT1  (.inst_clk(i_clk), .inst_rst_n(!i_rst), .inst_en(i_sqrt_start), .inst_a(i_sqrt[39-:`SQRT_WIDTH]), .root_inst(o_sqrt));

    complex_subtractor CS2 (.a(i_complex_sub1), .b(i_complex_sub2), .out(o_complex_sub)); // S3.16 - S3.16 get S3.16

    // Division module: S3.16 / S3.16 get S3.16
    division           DV3 (.i_clk(i_clk), .i_rst(i_rst), .i_start(i_div_start), .i_a(i_div1), .i_b(i_div2), .result(o_div));

// ---------------------------------------------------------------------------
// combinational circuits
//----------------------------------------------------------------------------
// output signal
    assign o_rd_vld    = (state == S_OUT);
    assign o_last_data = (state == S_OUT && cur_PE == 4'd9);
    assign o_r         = {r44_r, r34_r[19:0], r34_r[39:20],r24_r[19:0], r24_r[39:20],r14_r[19:0], r14_r[39:20], r33_r, r23_r[19:0], r23_r[39:20], r13_r[19:0], r13_r[39:20], r22_r, r12_r[19:0], r12_r[39:20], r11_r};
    assign o_y_hat     = {HY_regfile[3][19:0], HY_regfile[3][39:20], HY_regfile[2][19:0], HY_regfile[2][39:20], HY_regfile[1][19:0], HY_regfile[1][39:20], HY_regfile[0][19:0], HY_regfile[0][39:20]};

// SRAM signal
    assign CEN = 0;
    assign WEN = !(i_trig || ((state == S_PRJ2 || state == S_PRJ3 || state == S_PRJ4) && inside_ctr >= `CYCLE_MULT) || (state == S_STOREY && cur_ite != 3));
    assign addr = i_trig ? inside_ctr : ( (state == S_STOREY) ? ( (cur_ite == 3) ? {5'b11010, addr_inside[2:0]} : {5'b11010, cur_ite, inside_ctr[0]}) : addr_inside + 20*cur_PE);
    assign sram_data_part[0] = i_trig ? {{(`INT-1){i_data[23]}}, i_data[23 -:(9-`INT)]} : ((state == S_STOREY) ? (inside_ctr[0] ? HY_regfile[3][19:12] : HY_regfile[3][39:32]) : o_complex_sub[2*`LENGTH-1 -: 8]);
    assign sram_data_part[1] = i_trig ?                          i_data[(23-9+`INT)-:8] : ((state == S_STOREY) ? (inside_ctr[0] ? HY_regfile[3][11: 4] : HY_regfile[3][31:24]) : o_complex_sub[2*`LENGTH-9 -: 8]);
    assign sram_data_part[2] = i_trig ? {{(`INT-1){i_data[47]}}, i_data[47 -:(9-`INT)]} : ((state == S_STOREY) ? (inside_ctr[0] ? {HY_regfile[3][3:0], 4'b0} : {HY_regfile[3][23:20], 4'b0}) : o_complex_sub[  `LENGTH-1 -: 8]);
    assign sram_data_part[3] = i_trig ?                          i_data[(38 +`INT)-:8]  : ((state == S_STOREY) ? (inside_ctr[0] ? 8'b0                : 8'b0               ) : o_complex_sub[  `LENGTH-9 -: 8]);
    assign sram_Q    = {sram_Q_part[0], sram_Q_part[1], {(`LENGTH-16){1'b0}}, sram_Q_part[2], sram_Q_part[3], {(`LENGTH-16){1'b0}}};
    assign state_prjs = ((state == S_PRJ2) || (state == S_PRJ3) || (state == S_PRJ4));
    assign state_hs   = ((state == S_H2)   || (state == S_H3)   || (state == S_H4));
    assign state_ips  = ((state == S_H2)   || (state == S_H3)   || (state == S_H4) || (state == S_EUCD) || (state == S_Y));

// Finite state machine
    always@(*) begin
        case (state)
            S_READ:
                if (inside_ctr == 199) state_nxt = S_EUCD;
                else                   state_nxt = state;
            S_EUCD:
                if (o_inner_finish)  state_nxt = S_SQRT;
                else                 state_nxt = state;
            S_SQRT:
                if (inside_ctr == `CYCLE_SQRT-1)   state_nxt = S_NORM;
                else                 state_nxt = state;
            S_NORM:
                if      (inside_ctr == (`CYCLE_DIV + 6) && cur_ite == 0) state_nxt = S_H2;
                else if (inside_ctr == (`CYCLE_DIV + 6) && cur_ite == 1) state_nxt = S_H3;
                else if (inside_ctr == (`CYCLE_DIV + 6) && cur_ite == 2) state_nxt = S_H4;
                else if (inside_ctr == (`CYCLE_DIV + 6) && cur_ite == 3) state_nxt = S_Y;
                else                                                     state_nxt = state;
            S_H2:
                if (o_inner_finish) state_nxt = S_PRJ2;
                else                state_nxt = state;
            S_PRJ2:
                if (inside_ctr == (3 + `CYCLE_MULT)) state_nxt = S_H3;
                else                                 state_nxt = state;
            S_H3:
                if (o_inner_finish) state_nxt = S_PRJ3;
                else                state_nxt = state;
            S_PRJ3:
                if (inside_ctr == (3 + `CYCLE_MULT)) state_nxt = S_H4;
                else                                 state_nxt = state;
            S_H4:
                if (o_inner_finish) state_nxt = S_PRJ4;
                else                state_nxt = state;
            S_PRJ4:
                if (inside_ctr == (3 + `CYCLE_MULT)) state_nxt = S_Y;
                else                                 state_nxt = state;
            S_OUT :
                if (cur_PE == 4'd9) state_nxt = S_READ;
                else                state_nxt = S_EUCD;
            S_Y:
                if (o_inner_finish) state_nxt = S_STOREY;
                else                state_nxt = state;
            S_STOREY:
                if (cur_ite == 3 && inside_ctr == 6) state_nxt = S_OUT;
                else if (cur_ite != 3 && inside_ctr == 1)   state_nxt = S_EUCD;
                else                        state_nxt = state;
            default: 
                state_nxt = S_READ;
        endcase    
    end

// counter for main module, including iterations and PEs
    assign cur_ite = main_ctr[1:0];
    assign cur_PE  = main_ctr[5:2];
    always@(*) begin
        if      (state == S_READ   && state_nxt == S_EUCD) main_ctr_nxt = 0;
        else if (state == S_STOREY && state_nxt == S_EUCD) main_ctr_nxt = main_ctr + 1'b1;
        else if (state == S_OUT)                           main_ctr_nxt = main_ctr + 1'b1;
        else                                               main_ctr_nxt = main_ctr;
    end


// counter inside state
    always@(*) begin
        case (state)
            S_STOREY:
                if      (cur_ite != 3 && inside_ctr == 1) inside_ctr_nxt = 0;
                else if (cur_ite == 3 && inside_ctr == 6) inside_ctr_nxt = 0;
                else                                      inside_ctr_nxt = inside_ctr + 1;
            S_READ:
                if      (state != state_nxt) inside_ctr_nxt = 0;
                else if (i_trig)             inside_ctr_nxt = inside_ctr +1;
                else                         inside_ctr_nxt = 0;
            default:
                if (state != state_nxt)      inside_ctr_nxt = 0;
                else                         inside_ctr_nxt = inside_ctr + 1;  
        endcase
    end

// address inside iteration
    always@(*) begin
        case (state)
            S_EUCD:
                addr_inside = cur_ite + inside_ctr * 5;
            S_H2:   // fetch h2
                addr_inside = inside_ctr[1:0] * 5 + 1;
            S_PRJ2:
                addr_inside = (inside_ctr-`CYCLE_MULT) * 5 + 1; // 0: x; 1: 1; 2:6; 3:11; 4:16 // parameterize?
            S_H3:
                addr_inside = inside_ctr[1:0] * 5 + 2;
            S_PRJ3:
                addr_inside = inside_ctr * 5 -`CYCLE_MULT * 5 + 2;
            S_H4:
                addr_inside = inside_ctr[1:0] * 5 + 3;
            S_PRJ4:
                addr_inside = (inside_ctr-`CYCLE_MULT) * 5 + 3;
            S_Y:
                addr_inside = inside_ctr[1:0] * 5 + 4;
            S_STOREY:
                case (cur_ite)
                    2'b00:
                        addr_inside = {cur_ite, inside_ctr[0]};
                    2'b01:
                        addr_inside = {cur_ite, inside_ctr[0]};
                    2'b10:
                        addr_inside = {cur_ite, inside_ctr[0]};
                    2'b11:
                        addr_inside = {inside_ctr[2:0]};
                endcase
            default:
                addr_inside = 0;
        endcase
    end

// calculate
// Square Root
    assign i_sqrt_start = (state == S_SQRT);
    assign i_sqrt       = o_ED_square;

// Inner Product Interface
    assign i_inner_start = state_ips && (inside_ctr != 0 && inside_ctr != 1);
    always@(*) begin
        case (inside_ctr[2:0])
            3'b010:  begin HY_regfile_ref = HY_regfile[0]; Q_regfile_ref = Q_regfile[0]; end
            3'b011:  begin HY_regfile_ref = HY_regfile[1]; Q_regfile_ref = Q_regfile[1]; end
            3'b100:  begin HY_regfile_ref = HY_regfile[2]; Q_regfile_ref = Q_regfile[2]; end
            3'b101:  begin HY_regfile_ref = HY_regfile[3]; Q_regfile_ref = Q_regfile[3]; end
            default: begin HY_regfile_ref =             0; Q_regfile_ref =            0; end
        endcase
    end


    always@(*) begin
        case (state)
            S_EUCD: begin
                i_inner_product1 = HY_regfile_ref;
                i_inner_product2 = HY_regfile_ref;
            end
            S_H2: begin
                i_inner_product1 = HY_regfile_ref;
                i_inner_product2 = Q_regfile_ref;
            end
            S_H3: begin
                i_inner_product1 = HY_regfile_ref;
                i_inner_product2 = Q_regfile_ref;
            end        
            S_H4: begin
                i_inner_product1 = HY_regfile_ref;
                i_inner_product2 = Q_regfile_ref;
            end
            S_Y: begin
                i_inner_product1 = HY_regfile[0];
                i_inner_product2 = Q_regfile_ref;
            end
            default: begin
                i_inner_product1 = i_complex_mul1;
                i_inner_product2 = i_complex_mul2;
            end
        endcase
    end

// Complex Multiplier Interfac
    // assign complex_mul_start = state_prjs && (inside_ctr != 5 && inside_ctr != 4);
    /* S3.16 * S3.16 get S7.32 */
    always@(*) begin
        // if      (cur_ite == 0 && state == S_PRJ2) i_complex_mul1 = {r12_r[39], r12_r[(35+`INT) -: (`INT + `FRAC)], r12_r[19], r12_r[(15+`INT) -: (`INT + `FRAC)]}; //{r12_r[39:(40-`LENGTH)], r12_r[19:(20-`LENGTH)]};
        // else if (cur_ite == 0 && state == S_PRJ3) i_complex_mul1 = {r13_r[39], r13_r[(35+`INT) -: (`INT + `FRAC)], r13_r[19], r13_r[(15+`INT) -: (`INT + `FRAC)]};
        // else if (cur_ite == 0 && state == S_PRJ4) i_complex_mul1 = {r14_r[39], r14_r[(35+`INT) -: (`INT + `FRAC)], r14_r[19], r14_r[(15+`INT) -: (`INT + `FRAC)]};
        // else if (cur_ite == 1 && state == S_PRJ3) i_complex_mul1 = {r23_r[39], r23_r[(35+`INT) -: (`INT + `FRAC)], r23_r[19], r23_r[(15+`INT) -: (`INT + `FRAC)]};
        // else if (cur_ite == 1 && state == S_PRJ4) i_complex_mul1 = {r24_r[39], r24_r[(35+`INT) -: (`INT + `FRAC)], r24_r[19], r24_r[(15+`INT) -: (`INT + `FRAC)]};
        // else if (cur_ite == 2 && state == S_PRJ4) i_complex_mul1 = {r34_r[39], r34_r[(35+`INT) -: (`INT + `FRAC)], r34_r[19], r34_r[(15+`INT) -: (`INT + `FRAC)]};
        // else                                      i_complex_mul1 = 0;
        // i_complex_mul1 = {r_tmp[39], r_tmp[(35+`INT) -: (`INT + `FRAC)], r_tmp[19], r_tmp[(15+`INT) -: (`INT + `FRAC)]};
        i_complex_mul1 = o_inner_product;
        i_complex_mul2 = Q_regfile[inside_ctr[1:0]];
    end

// Dividor
    assign i_div_start = (state == S_NORM);
    always@(*) begin
        case(cur_ite)
            2'b00: i_div2 = r11_r;
            2'b01: i_div2 = r22_r;
            2'b10: i_div2 = r33_r;
            2'b11: i_div2 = r44_r;
        endcase

        case({i_div_start, inside_ctr[2:0]})
            4'b1000: i_div1 = HY_regfile[0][2*`LENGTH-1 : `LENGTH];
            4'b1001: i_div1 = HY_regfile[0][  `LENGTH-1 :       0];
            4'b1010: i_div1 = HY_regfile[1][2*`LENGTH-1 : `LENGTH];
            4'b1011: i_div1 = HY_regfile[1][  `LENGTH-1 :       0];
            4'b1100: i_div1 = HY_regfile[2][2*`LENGTH-1 : `LENGTH];
            4'b1101: i_div1 = HY_regfile[2][  `LENGTH-1 :       0];
            4'b1110: i_div1 = HY_regfile[3][2*`LENGTH-1 : `LENGTH];
            4'b1111: i_div1 = HY_regfile[3][  `LENGTH-1 :       0];
            default: i_div1 = 0;
        endcase
    end
    assign div_finish = (state == S_NORM && inside_ctr >= `CYCLE_DIV - 1);

// Complex Substractor Interface
    /* S3.16 - S3.16 get S3.16 */;

    assign i_complex_sub1 = (inside_ctr == 1) ? HY_regfile[0] : (inside_ctr == 2 ? HY_regfile[1] : (inside_ctr == 3 ? HY_regfile[2] : (inside_ctr == 4 ? HY_regfile[3] : 0)));
    assign i_complex_sub2 = {o_complex_mul_re[2*`LENGTH-1], o_complex_mul_re[(2*`FRAC+`INT-1)-:`LENGTH-1], o_complex_mul_im[2*`LENGTH-1], o_complex_mul_im[(2*`FRAC+`INT-1)-:`LENGTH-1]}; // check

// Q, H, Y, R
    // H
    always@(*) begin
        if ((state == S_EUCD || state_hs) && inside_ctr == 1) HY_regfile_nxt[0] = sram_Q;
        else if (state == S_Y)                                                                       HY_regfile_nxt[0] = sram_Q;
        else if (state == S_STOREY && inside_ctr == 1 && cur_ite == 3)                               HY_regfile_nxt[0] = {sram_Q_part[0], sram_Q_part[1], sram_Q_part[2][7:4], HY_regfile[0][19:0]};
        else if (state == S_STOREY && inside_ctr == 2 && cur_ite == 3)                               HY_regfile_nxt[0] = {HY_regfile[0][39:20], sram_Q_part[0], sram_Q_part[1], sram_Q_part[2][7:4]};
        else                                                                                         HY_regfile_nxt[0] = HY_regfile[0];
        
        if ((state == S_EUCD || state_hs) && inside_ctr == 2) HY_regfile_nxt[1] = sram_Q;
        else if (state == S_STOREY && inside_ctr == 3 && cur_ite == 3)                               HY_regfile_nxt[1] = {sram_Q_part[0], sram_Q_part[1], sram_Q_part[2][7:4], HY_regfile[1][19:0]};
        else if (state == S_STOREY && inside_ctr == 4 && cur_ite == 3)                               HY_regfile_nxt[1] = {HY_regfile[1][39:20], sram_Q_part[0], sram_Q_part[1], sram_Q_part[2][7:4]};
        else                                                                                         HY_regfile_nxt[1] = HY_regfile[1];        
        
        if ((state == S_EUCD || state_hs) && inside_ctr == 3) HY_regfile_nxt[2] = sram_Q;
        else if (state == S_STOREY && inside_ctr == 5 && cur_ite == 3)                               HY_regfile_nxt[2] = {sram_Q_part[0], sram_Q_part[1], sram_Q_part[2][7:4], HY_regfile[2][19:0]};
        else if (state == S_STOREY && inside_ctr == 6 && cur_ite == 3)                               HY_regfile_nxt[2] = {HY_regfile[2][39:20], sram_Q_part[0], sram_Q_part[1], sram_Q_part[2][7:4]};
        else                                                                                         HY_regfile_nxt[2] = HY_regfile[2];        
        
        if ((state == S_EUCD || state_hs) && inside_ctr == 4) HY_regfile_nxt[3] = sram_Q;
        else if (state == S_Y && o_inner_finish)                                                     HY_regfile_nxt[3] = o_inner_product;
        else                                                                                         HY_regfile_nxt[3] = HY_regfile[3];
    end

// Q
    assign Q_regfile_nxt[0][2*`LENGTH-1:`LENGTH] = (state == S_NORM && inside_ctr == (`CYCLE_DIV - 1)) ? o_div : Q_regfile[0][2*`LENGTH-1:`LENGTH]; // may remove o_div_finish
    assign Q_regfile_nxt[0][  `LENGTH-1:0]       = (state == S_NORM && inside_ctr == (`CYCLE_DIV + 0)) ? o_div : Q_regfile[0][  `LENGTH-1:0];
    assign Q_regfile_nxt[1][2*`LENGTH-1:`LENGTH] = (state == S_NORM && inside_ctr == (`CYCLE_DIV + 1)) ? o_div : Q_regfile[1][2*`LENGTH-1:`LENGTH];
    assign Q_regfile_nxt[1][  `LENGTH-1:0]       = (state == S_NORM && inside_ctr == (`CYCLE_DIV + 2)) ? o_div : Q_regfile[1][  `LENGTH-1:0];
    assign Q_regfile_nxt[2][2*`LENGTH-1:`LENGTH] = (state == S_NORM && inside_ctr == (`CYCLE_DIV + 3)) ? o_div : Q_regfile[2][2*`LENGTH-1:`LENGTH];
    assign Q_regfile_nxt[2][  `LENGTH-1:0]       = (state == S_NORM && inside_ctr == (`CYCLE_DIV + 4)) ? o_div : Q_regfile[2][  `LENGTH-1:0];
    assign Q_regfile_nxt[3][2*`LENGTH-1:`LENGTH] = (state == S_NORM && inside_ctr == (`CYCLE_DIV + 5)) ? o_div : Q_regfile[3][2*`LENGTH-1:`LENGTH];
    assign Q_regfile_nxt[3][  `LENGTH-1:0]       = (state == S_NORM && inside_ctr == (`CYCLE_DIV + 6)) ? o_div : Q_regfile[3][  `LENGTH-1:0];

// R
    assign r11_w = (cur_ite == 0 && state == S_SQRT) ? {o_sqrt, {(20-(`SQRT_WIDTH+1)/2){1'b0}}} : r11_r;
    assign r22_w = (cur_ite == 1 && state == S_SQRT) ? {o_sqrt, {(20-(`SQRT_WIDTH+1)/2){1'b0}}} : r22_r;
    assign r33_w = (cur_ite == 2 && state == S_SQRT) ? {o_sqrt, {(20-(`SQRT_WIDTH+1)/2){1'b0}}} : r33_r;
    assign r44_w = (cur_ite == 3 && state == S_SQRT) ? {o_sqrt, {(20-(`SQRT_WIDTH+1)/2){1'b0}}} : r44_r;

    assign r12_w = (cur_ite == 0 && state == S_H2)  ? o_inner_product : r12_r; // may remove o_inner_finish
    assign r13_w = (cur_ite == 0 && state == S_H3)  ? o_inner_product : r13_r;
    assign r14_w = (cur_ite == 0 && state == S_H4)  ? o_inner_product : r14_r;
    assign r23_w = (cur_ite == 1 && state == S_H3)  ? o_inner_product : r23_r;
    assign r24_w = (cur_ite == 1 && state == S_H4)  ? o_inner_product : r24_r;
    assign r34_w = (cur_ite == 2 && state == S_H4)  ? o_inner_product : r34_r;
// ---------------------------------------------------------------------------
// sequential circuits
//----------------------------------------------------------------------------
    // state, main_ctr, inside_ctr no clock gating
    always@(posedge i_clk or posedge i_rst) begin
        if (i_rst) begin
            state      <= 0;
            main_ctr   <= 0;
            inside_ctr <= 0;
        end else begin
            state      <= state_nxt;
            main_ctr   <= main_ctr_nxt;
            inside_ctr <= inside_ctr_nxt;
        end
    end

    // HY stops when S_READ
    assign HY_en  = (state != S_READ && !state_prjs);
    assign HY_clk = i_clk & HY_en;
    `ifdef SYN
        always@(posedge HY_clk or posedge i_rst) begin
            if (i_rst) begin
                HY_regfile[0] <= 0;
                HY_regfile[1] <= 0;
                HY_regfile[2] <= 0;
                HY_regfile[3] <= 0;
            end else begin
                HY_regfile[0] <= HY_regfile_nxt[0];
                HY_regfile[1] <= HY_regfile_nxt[1];
                HY_regfile[2] <= HY_regfile_nxt[2];
                HY_regfile[3] <= HY_regfile_nxt[3];        
            end
        end
    `else
        always@(posedge i_clk or posedge i_rst) begin
            if (i_rst) begin
                HY_regfile[0] <= 0;
                HY_regfile[1] <= 0;
                HY_regfile[2] <= 0;
                HY_regfile[3] <= 0;
            end else begin
                HY_regfile[0] <= HY_regfile_nxt[0];
                HY_regfile[1] <= HY_regfile_nxt[1];
                HY_regfile[2] <= HY_regfile_nxt[2];
                HY_regfile[3] <= HY_regfile_nxt[3];        
            end
        end
    `endif

// Q
    assign Q_en  = (state == S_NORM);
    assign Q_clk = i_clk & Q_en;
    `ifdef SYN
        always@(posedge Q_clk or posedge i_rst) begin
            if (i_rst) begin
                Q_regfile[0] <= 0;
                Q_regfile[1] <= 0;
                Q_regfile[2] <= 0;
                Q_regfile[3] <= 0;
            end else begin
                Q_regfile[0] <= Q_regfile_nxt[0];
                Q_regfile[1] <= Q_regfile_nxt[1];
                Q_regfile[2] <= Q_regfile_nxt[2];
                Q_regfile[3] <= Q_regfile_nxt[3];        
            end
        end
    `else
        always@(posedge i_clk or posedge i_rst) begin
            if (i_rst) begin
                Q_regfile[0] <= 0;
                Q_regfile[1] <= 0;
                Q_regfile[2] <= 0;
                Q_regfile[3] <= 0;
            end else if (Q_en) begin
                Q_regfile[0] <= Q_regfile_nxt[0];
                Q_regfile[1] <= Q_regfile_nxt[1];
                Q_regfile[2] <= Q_regfile_nxt[2];
                Q_regfile[3] <= Q_regfile_nxt[3];        
            end
        end
    `endif

// R
    assign r11_en  = ((state == S_SQRT || state == S_NORM)  && cur_ite == 0);
    assign r22_en  = ((state == S_SQRT || state == S_NORM)  && cur_ite == 1);
    assign r33_en  = ((state == S_SQRT || state == S_NORM)  && cur_ite == 2);
    assign r44_en  = ((state == S_SQRT || state == S_NORM)  && cur_ite == 3);
    assign r11_clk = i_clk & r11_en;
    assign r22_clk = i_clk & r22_en;
    assign r33_clk = i_clk & r33_en;
    assign r44_clk = i_clk & r44_en;
    
    `ifdef SYN
        always@(posedge r11_clk or posedge i_rst) begin
            if (i_rst) r11_r <= 0;
            else       r11_r <= r11_w;
        end
    `else
        always@(posedge i_clk or posedge i_rst) begin
            if (i_rst)       r11_r <= 0;
            else if (r11_en) r11_r <= r11_w;
        end
    `endif

    `ifdef SYN
        always@(posedge r22_clk or posedge i_rst) begin
            if (i_rst) r22_r <= 0;
            else       r22_r <= r22_w;
        end
    `else
        always@(posedge i_clk or posedge i_rst) begin
            if (i_rst)       r22_r <= 0;
            else if (r22_en) r22_r <= r22_w;
        end
    `endif

    `ifdef SYN
        always@(posedge r33_clk or posedge i_rst) begin
            if (i_rst) r33_r <= 0;
            else       r33_r <= r33_w;
        end
    `else
        always@(posedge i_clk or posedge i_rst) begin
            if (i_rst)       r33_r <= 0;
            else if (r33_en) r33_r <= r33_w;
        end
    `endif

    `ifdef SYN
        always@(posedge r44_clk or posedge i_rst) begin
            if (i_rst) r44_r <= 0;
            else       r44_r <= r44_w;
        end
    `else
        always@(posedge i_clk or posedge i_rst) begin
            if (i_rst)       r44_r <= 0;
            else if (r44_en) r44_r <= r44_w;
        end
    `endif
    
    assign rij_en  = (state == S_H2 || state == S_H3 || state == S_H4 || state == S_PRJ2 || state == S_PRJ3 || state == S_PRJ4);
    assign rij_clk = i_clk & rij_en;
    
    `ifdef SYN
        always@(posedge rij_clk or posedge i_rst) begin
            if (i_rst) begin
                r34_r <= 0;
                r24_r <= 0;
                r14_r <= 0;
                r23_r <= 0;
                r13_r <= 0;
                r12_r <= 0;
            end else begin
                r34_r <= r34_w;
                r24_r <= r24_w;
                r14_r <= r14_w;
                r23_r <= r23_w;
                r13_r <= r13_w;
                r12_r <= r12_w;
            end
        end
    `else
        always@(posedge i_clk or posedge i_rst) begin
            if (i_rst) begin
                r34_r <= 0;
                r24_r <= 0;
                r14_r <= 0;
                r23_r <= 0;
                r13_r <= 0;
                r12_r <= 0;
            end else if (rij_en) begin
                r34_r <= r34_w;
                r24_r <= r24_w;
                r14_r <= r14_w;
                r23_r <= r23_w;
                r13_r <= r13_w;
                r12_r <= r12_w;
            end
        end
    `endif
endmodule

/*  
    Complex mult module      : S3.16 * S3.16 get S7.32
    Complex sub  module      : S3.16 - S3.16 get S3.16
    Inner Product            : accumulate S7.32 then truncate into S3.16 output, also it can output S7.32 Euclidean square
    Division module          : S3.16 / S3.16 get S3.16
    Square Root module       : input S7.32 get output S3.16
*/
module division(
    /*
        Division module          : S3.16 / S3.16 get S3.16
    */
    input  i_clk,
    input  i_rst,
    input  i_start,
    input  [`LENGTH-1:0] i_a,
    input  [`LENGTH-1:0] i_b,
    output [`LENGTH-1:0] result
);  
    wire zero;

    // div_a: {S2.12 + 10'b0}
    // div_b: {S2.10}
    parameter a_width = 22;
    parameter b_width = 11;
    
    wire [a_width-1:0] div_a, quotient;
    wire [b_width-1:0] div_b, remainder;

    assign div_a  = {i_a[19], i_a[17:0], 3'b0}; // 
    assign div_b  = {i_b[19], i_b[17-:10]};
    assign result = {quotient[0+:15],5'b0};     // 
    // assign  i_a_2 = {i_a, 10'b0};
    // assign  result = {quotient[0 +: 14], 6'd0};

    DW_div_pipe_inst #(.inst_a_width(a_width), .inst_b_width(b_width), .inst_tc_mode(1),
                        .inst_rem_mode(1), .inst_num_stages(`CYCLE_DIV), .inst_stall_mode(1), .inst_rst_mode(1), .inst_op_iso_mode(0)) 
    U0  (.inst_clk(i_clk), .inst_rst_n(!i_rst), .inst_en(i_start), .inst_a(div_a), .inst_b(div_b)
        ,.quotient_inst(quotient), .remainder_inst(remainder), .divide_by_0_inst(zero));
endmodule

module DW_div_pipe_inst(inst_clk, inst_rst_n, inst_en, inst_a, inst_b, quotient_inst, remainder_inst, divide_by_0_inst );
    parameter inst_a_width = 8;
    parameter inst_b_width = 8;
    parameter inst_tc_mode = 0;
    parameter inst_rem_mode = 1;
    parameter inst_num_stages = 2;
    parameter inst_stall_mode = 1;
    parameter inst_rst_mode = 1;
    parameter inst_op_iso_mode = 0;

    input inst_clk;
    input inst_rst_n;
    input inst_en;
    input [inst_a_width-1 : 0] inst_a;
    input [inst_b_width-1 : 0] inst_b;
    output [inst_a_width-1 : 0] quotient_inst;
    output [inst_b_width-1 : 0] remainder_inst;
    output divide_by_0_inst;
    
    // Instance of DW_div_pipe
    DW_div_pipe #(inst_a_width, inst_b_width, inst_tc_mode, inst_rem_mode,
    inst_num_stages, inst_stall_mode, inst_rst_mode, 
    inst_op_iso_mode) 
    U1 (.clk(inst_clk), .rst_n(inst_rst_n), .en(inst_en),
    .a(inst_a), .b(inst_b), .quotient(quotient_inst),
    .remainder(remainder_inst), .divide_by_0(divide_by_0_inst) );
endmodule

module DW_sqrt_pipe_inst( inst_clk, inst_rst_n, inst_en, inst_a, root_inst );
    parameter inst_width = 2;
    parameter inst_tc_mode = 0;
    parameter inst_num_stages = 2;
    parameter inst_stall_mode = 1;
    parameter inst_rst_mode = 1;
    parameter inst_op_iso_mode = 0;
    
    input inst_clk;
    input inst_rst_n;
    input inst_en;
    input [inst_width-1 : 0] inst_a;
    output [(inst_width+1)/2-1 : 0] root_inst;
    
    // Instance of DW_sqrt_pipe
    DW_sqrt_pipe #(inst_width, inst_tc_mode, inst_num_stages, 
    inst_stall_mode, inst_rst_mode, inst_op_iso_mode) 
    U1 (.clk(inst_clk), .rst_n(inst_rst_n),
    .en(inst_en), .a(inst_a), .root(root_inst) );
endmodule

module inner_product(
    /*
        Inner Product accumulate S7.32 then truncate into S3.16 output
        o_ED_square is the square of Euclidean distance, and it is in S7.32 format
        When i_start==1,  input-data should be the right one
        When o_finish==1, output-data is valid
        By the way, i_start should remain 1
    */
    input i_clk,
    input i_rst,
    input i_start,
    input [2*`LENGTH-1:0] i_data1,
    input [2*`LENGTH-1:0] i_data2,
    output o_inner_finish,
    output [2*`LENGTH-1:0] o_R,
    output [2*`LENGTH-1:0] o_ED_square,
    output [2*`LENGTH-1:0] out_re, out_im
);  
    localparam IDLE  = 3'd0;
    localparam STALL = 3'd1;
    localparam CALC1 = 3'd2;
    localparam CALC2 = 3'd3;
    localparam CALC3 = 3'd4;
    localparam CALC4 = 3'd5;
    localparam OUT   = 3'd6;

    reg  [2:0]           state_r, state_w;
    reg signed [2*`LENGTH+3:0] o_R_re, o_R_im;
    reg signed [2*`LENGTH+3:0] o_R_re_nxt, o_R_im_nxt;
    wire [`LENGTH-1:0]   tmp;
    wire [2*`LENGTH-1:0] tmp_b;

    assign o_inner_finish = state_r==OUT;
    assign o_R[2*`LENGTH-1 -: `LENGTH] =  (o_R_re[2*`LENGTH+3-:9]==9'b111111111 || o_R_re[2*`LENGTH+3-:9]==9'b000000000) ? o_R_re[(2*`LENGTH-5) -: (`LENGTH)]
                            : (o_R_re[2*`LENGTH+3]) ? {1'b1, {(`LENGTH-1){1'd0}}} : {1'b0, {(`LENGTH-1){1'd1}}};
    assign o_R[0 +: `LENGTH]         =  (o_R_im[2*`LENGTH+3-:9]==9'b111111111 || o_R_im[2*`LENGTH+3-:9]==9'b000000000) ? o_R_im[(2*`LENGTH-5) -: (`LENGTH)] 
                            : (o_R_im[2*`LENGTH+3]) ? {1'b1, {(`LENGTH-1){1'd0}}} : {1'b0, {(`LENGTH-1){1'd1}}};
    assign o_ED_square    = (&(~o_R_re[2*`LENGTH+3 : 2*`LENGTH-1])) ? o_R_re[2*`LENGTH-1:0] : {1'b0, {(2*`LENGTH-1){1'b1}}};


    assign tmp = (i_start) ? ~(i_data2[0 +: `LENGTH])+1 : i_data2[0+:`LENGTH];
    assign tmp_b = {i_data2[2*`LENGTH-1-:`LENGTH], tmp};

    complex_multiplier cm1(.i_clk(i_clk), .i_rst(i_rst), .a(i_data1), .b(tmp_b), .out_re(out_re), .out_im(out_im));

    always@ (*) begin
        case(state_r)
            IDLE: state_w = (i_start) ? STALL : IDLE;
            STALL:   state_w = CALC1;
            CALC1:   state_w = CALC2;
            CALC2:   state_w = CALC3;
            CALC3:   state_w = CALC4;
            CALC4:   state_w = OUT;
            OUT  :   state_w = IDLE;
            default: state_w = IDLE;
        endcase
    end

    always@ (*) begin
        o_R_re_nxt = ((state_r==IDLE && !i_start) || state_r==OUT) ? o_R_re :
                      (state_r==IDLE &&  i_start)      ?      0 : $signed(o_R_re) + $signed(out_re);
        o_R_im_nxt = ((state_r==IDLE && !i_start) || state_r==OUT) ? o_R_im :
                      (state_r==IDLE &&  i_start)      ?      0 : $signed(o_R_im) + $signed(out_im);
    end

    always@ (posedge i_clk or posedge i_rst) begin
        if(i_rst) begin
            state_r <= IDLE;
            o_R_re  <= 0;
            o_R_im  <= 0;
        end
        else begin
            state_r <= state_w;
            o_R_re  <= o_R_re_nxt;
            o_R_im  <= o_R_im_nxt;
        end
    end
endmodule

module complex_subtractor(
    /*
        Complex sub module      : S3.16 - S3.16 get S3.16
        only combinational logic
    */
    input  [2*`LENGTH-1:0] a,
    input  [2*`LENGTH-1:0] b,
    output [2*`LENGTH-1:0] out 
);
    wire [`LENGTH-1:0] a_re, a_im;
    wire [`LENGTH-1:0] b_re, b_im;
    wire [`LENGTH-1:0] temp1, temp2;
    
    assign a_re = a[2*`LENGTH-1 -:`LENGTH];
    assign a_im = a[0 +: `LENGTH];
    assign b_re = b[2*`LENGTH-1 -:`LENGTH];
    assign b_im = b[0 +: `LENGTH];

    assign temp1 = $signed(a_re) - $signed(b_re);
    assign temp2 = $signed(a_im) - $signed(b_im);
    assign out[2*`LENGTH-1 -: `LENGTH] = (a_re[`LENGTH-1] ~^ b_re[`LENGTH-1])  ? temp1 :
                                         (a_re[`LENGTH-1] ~^ temp1[`LENGTH-1]) ? temp1 :
                                         (a_re[`LENGTH-1]) ? { 1'b1, {(`LENGTH-1){1'b0}} } : { 1'b0, {(`LENGTH-1){1'b1}} };

    assign out[0 +: `LENGTH]           = (a_im[`LENGTH-1] ~^ b_im[`LENGTH-1])  ? temp2 :
                                         (a_im[`LENGTH-1] ~^ temp2[`LENGTH-1]) ? temp2 :
                                         (a_im[`LENGTH-1]) ? { 1'b1, {(`LENGTH-1){1'b0}} } : { 1'b0, {(`LENGTH-1){1'b1}} };
endmodule

module complex_multiplier(
    /*
        Complex mult module      : S3.16 * S3.16 get S7.32
        When i_start  = 1, input-data should be the right one
        When o_finish = 1, output-data is valid
    */
    input  i_clk,
    input  i_rst,
    input  [2*`LENGTH-1:0] a,
    input  [2*`LENGTH-1:0] b,
    output [2*`LENGTH-1:0] out_re,
    output [2*`LENGTH-1:0] out_im
);  
    wire [`LENGTH-1:0]   a_re, a_im;
    wire [`LENGTH-1:0]   b_re, b_im;
    wire [2*`LENGTH_MUL-1:0] temp1, temp2, temp3, temp4, temp5, temp6;

    // reg  [2*`LENGTH_MUL-1:0] op1, op2, op3, op4;

    assign a_re = a[2*`LENGTH-1 -:`LENGTH];
    assign a_im = a[0 +: `LENGTH];
    assign b_re = b[2*`LENGTH-1 -:`LENGTH];
    assign b_im = b[0 +: `LENGTH];
    
    DW02_mult_2_stage #(`LENGTH_MUL, `LENGTH_MUL) U0(.A(a_re[19-:`LENGTH_MUL]), .B(b_re[19-:`LENGTH_MUL]), .TC(1'b1), .CLK(i_clk), .PRODUCT(temp1));
    DW02_mult_2_stage #(`LENGTH_MUL, `LENGTH_MUL) U1(.A(a_im[19-:`LENGTH_MUL]), .B(b_im[19-:`LENGTH_MUL]), .TC(1'b1), .CLK(i_clk), .PRODUCT(temp2));
    DW02_mult_2_stage #(`LENGTH_MUL, `LENGTH_MUL) U2(.A(a_re[19-:`LENGTH_MUL]), .B(b_im[19-:`LENGTH_MUL]), .TC(1'b1), .CLK(i_clk), .PRODUCT(temp3));
    DW02_mult_2_stage #(`LENGTH_MUL, `LENGTH_MUL) U3(.A(a_im[19-:`LENGTH_MUL]), .B(b_re[19-:`LENGTH_MUL]), .TC(1'b1), .CLK(i_clk), .PRODUCT(temp4));

    assign temp5 = $signed(temp1)  - $signed(temp2);
    assign temp6 = $signed(temp3)  + $signed(temp4); 
    
    assign out_re = {temp5, {(40-2*`LENGTH_MUL){1'b0}}};
    assign out_im = {temp6, {(40-2*`LENGTH_MUL){1'b0}}};
endmodule

// module DW02_mult_2_stage_inst( inst_A, inst_B, inst_TC, 
//                                inst_CLK, PRODUCT_inst );

//   parameter A_width = 8;
//   parameter B_width = 8;

//   input [A_width-1 : 0] inst_A;
//   input [B_width-1 : 0] inst_B;
//   input inst_TC;
//   input inst_CLK;
//   output [A_width+B_width-1 : 0] PRODUCT_inst;

//   // Instance of DW02_mult_2_stage
//   DW02_mult_2_stage #(A_width, B_width)
//     U1 ( .A(inst_A),   .B(inst_B),   .TC(inst_TC), 
//          .CLK(inst_CLK),   .PRODUCT(PRODUCT_inst) );

// endmodule