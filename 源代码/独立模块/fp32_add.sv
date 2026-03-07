`timescale 1ns / 1ps

module fp32_add(
    input               clk,
    input               rst,
    input [31:0]        a_in,
    input [31:0]        b_in,
    output reg [31:0]   new_fp_acc    
);

reg [31:0] fp_acc, fp_product;
    
function [31:0] fp32_add;
    input [31:0] a, b;
    
    // 1. 提取符号位、指数位、尾数
    reg sign_a, sign_b;
    reg [7:0] exp_a, exp_b;
    reg [22:0] frac_a, frac_b;
    reg [23:0] mant_a, mant_b;
    
    // 2. 特殊值处理
    reg a_is_nan, b_is_nan, a_is_inf, b_is_inf, a_is_zero, b_is_zero;
    reg is_nan, is_inf, both_inf_opposite_sign;
    
    // 3. 对齐处理
    reg a_bigger;
    reg [9:0] exp_large, exp_small; // 无符号数扩大一位防溢出
    reg [23:0] mant_large, mant_small;
    reg sign_large, sign_small;
    reg [7:0] exp_diff;
    reg [27:0] mant_small_ext, mant_large_ext;
    
    // 4. 加减运算
    reg [26:0] mant_sum_ext;
    reg result_sign;
    
    // 5. 归一化与舍入
    reg signed [9:0] exp_out; // 改为有符号数 -512到511 保证大于正数最大值255
    reg [23:0] mant_out;
    reg [27:0] mant_tmp;
    
    // 6. 舍入位
    reg guard, round, sticky, lsb, round_up;
    reg [22:0] final_frac;
    reg [7:0] final_exp;
    reg final_sign;
    
    begin
        // 1. 提取符号、指数和尾数
        sign_a = a[31];
        sign_b = b[31];
        exp_a = a[30:23];
        exp_b = b[30:23];
        frac_a = a[22:0];
        frac_b = b[22:0];
        
        mant_a = (exp_a == 0) ? {1'b0, frac_a} : {1'b1, frac_a};
        mant_b = (exp_b == 0) ? {1'b0, frac_b} : {1'b1, frac_b};
        
        // 2. 特殊值检测
        a_is_nan = (exp_a == 8'hFF) && (frac_a != 0);
        b_is_nan = (exp_b == 8'hFF) && (frac_b != 0);
        a_is_inf = (exp_a == 8'hFF) && (frac_a == 0);
        b_is_inf = (exp_b == 8'hFF) && (frac_b == 0);
        a_is_zero = (exp_a == 0) && (frac_a == 0);
        b_is_zero = (exp_b == 0) && (frac_b == 0);
        
        is_nan = a_is_nan || b_is_nan;
        is_inf = a_is_inf || b_is_inf;
        both_inf_opposite_sign = a_is_inf && b_is_inf && (sign_a != sign_b);
        
        // 3. 对齐尾数
        a_bigger = (exp_a > exp_b) || ((exp_a == exp_b) && (mant_a >= mant_b));
        exp_large = a_bigger ? {1'b0, exp_a} : {1'b0, exp_b};
        exp_small = a_bigger ? {1'b0, exp_b} : {1'b0, exp_a};
        mant_large = a_bigger ? mant_a : mant_b;
        mant_small = a_bigger ? mant_b : mant_a;
        sign_large = a_bigger ? sign_a : sign_b;
        sign_small = a_bigger ? sign_b : sign_a;
        
        exp_diff = exp_large - exp_small;
        mant_small_ext = (exp_diff > 26) ? 27'b0 : {1'b0, mant_small, 3'b000} >> exp_diff;
        mant_large_ext = {1'b0, mant_large, 3'b000};
        
        // 4. 加减运算
        if (sign_large == sign_small) begin
            mant_sum_ext = mant_large_ext + mant_small_ext;
            result_sign = sign_large;
        end else begin
            if (mant_large_ext >= mant_small_ext) begin
                mant_sum_ext = mant_large_ext - mant_small_ext;
                result_sign = sign_large;
            end else begin
                mant_sum_ext = mant_small_ext - mant_large_ext;
                result_sign = sign_small;
            end
        end
        
        // 5. 归一化处理
        mant_tmp = mant_sum_ext;
        
        if (mant_tmp[27]) begin
            mant_out = mant_tmp[27:4];
            exp_out = exp_large + 1;
        end else if (mant_tmp[26]) begin
            mant_out = mant_tmp[26:3];
            exp_out = exp_large;
        end else if (mant_tmp[25]) begin
            mant_out = mant_tmp[25:2];
            exp_out = exp_large - 1;
        end else if (mant_tmp[24]) begin
            mant_out = mant_tmp[24:1];
            exp_out = exp_large - 2;
        end else if (mant_tmp[23]) begin
            mant_out = mant_tmp[23:0];
            exp_out = exp_large - 3;
        end else if (mant_tmp[22]) begin
            mant_out = {mant_tmp[22:0], 1'b0};
            exp_out = exp_large - 4;
        end else if (mant_tmp[21]) begin
            mant_out = {mant_tmp[21:0], 2'b0};
            exp_out = exp_large - 5;
        end else if (mant_tmp[20]) begin
            mant_out = {mant_tmp[20:0], 3'b0};
            exp_out = exp_large - 6;
        end else if (mant_tmp[19]) begin
            mant_out = {mant_tmp[19:0], 4'b0};
            exp_out = exp_large - 7;
        end else if (mant_tmp[18]) begin
            mant_out = {mant_tmp[18:0], 5'b0};
            exp_out = exp_large - 8;
        end else if (mant_tmp[17]) begin
            mant_out = {mant_tmp[17:0], 6'b0};
            exp_out = exp_large - 9;
        end else if (mant_tmp[16]) begin
            mant_out = {mant_tmp[16:0], 7'b0};
            exp_out = exp_large - 10;
        end else if (mant_tmp[15]) begin
            mant_out = {mant_tmp[15:0], 8'b0};
            exp_out = exp_large - 11;
        end else if (mant_tmp[14]) begin
            mant_out = {mant_tmp[14:0], 9'b0};
            exp_out = exp_large - 12;
        end else if (mant_tmp[13]) begin
            mant_out = {mant_tmp[13:0], 10'b0};
            exp_out = exp_large - 13;
        end else if (mant_tmp[12]) begin
            mant_out = {mant_tmp[12:0], 11'b0};
            exp_out = exp_large - 14;
        end else if (mant_tmp[11]) begin
            mant_out = {mant_tmp[11:0], 12'b0};
            exp_out = exp_large - 15;
        end else if (mant_tmp[10]) begin
            mant_out = {mant_tmp[10:0], 13'b0};
            exp_out = exp_large - 16;
        end else if (mant_tmp[9]) begin
            mant_out = {mant_tmp[9:0], 14'b0};
            exp_out = exp_large - 17;
        end else if (mant_tmp[8]) begin
            mant_out = {mant_tmp[8:0], 15'b0};
            exp_out = exp_large - 18;
        end else if (mant_tmp[7]) begin
            mant_out = {mant_tmp[7:0], 16'b0};
            exp_out = exp_large - 19;
        end else if (mant_tmp[6]) begin
            mant_out = {mant_tmp[6:0], 17'b0};
            exp_out = exp_large - 20;
        end else if (mant_tmp[5]) begin
            mant_out = {mant_tmp[5:0], 18'b0};
            exp_out = exp_large - 21;
        end else if (mant_tmp[4]) begin
            mant_out = {mant_tmp[4:0], 19'b0};
            exp_out = exp_large - 22;
        end else if (mant_tmp[3]) begin
            mant_out = {mant_tmp[3:0], 20'b0};
            exp_out = exp_large - 23;
        end else if (mant_tmp[2]) begin
            mant_out = {mant_tmp[2:0], 21'b0};
            exp_out = exp_large - 24;
        end else if (mant_tmp[1]) begin
            mant_out = {mant_tmp[1:0], 22'b0};
            exp_out = exp_large - 25;
        end else if (mant_tmp[0]) begin
            mant_out = {mant_tmp[0], 23'b0};
            exp_out = exp_large - 26;
        end else begin
            mant_out = 24'b0;
            exp_out = 0;
        end
        
        // 6. 舍入处理 (向偶舍入)
        guard = mant_tmp[2];
        round = mant_tmp[1];
        sticky = |mant_tmp[0];
        lsb = mant_out[0];
        round_up = guard && (round || sticky || lsb);
        
        if (round_up) begin
            if (mant_out == 24'hFFFFFF) begin
                mant_out = 24'h800000;
                exp_out = exp_out + 1;
            end else begin
                mant_out = mant_out + 1;
            end
        end
        
        // 7. 上溢出/下溢出检查
        if (exp_out >= 255) begin
            final_exp = 8'hFF;
            final_frac = 23'b0;
        end else if (exp_out <= 0) begin
            if (exp_out < -26) begin
                final_exp = 0;
                final_frac = 0;
            end else begin
                final_exp = 0;
                final_frac = mant_out[22:0] >> (-exp_out);
            end
        end else begin
            final_exp = exp_out[7:0];
            final_frac = mant_out[22:0];
        end
        
        // 8. 结果组装
        if (is_nan || both_inf_opposite_sign) begin
            fp32_add = 32'h7FC00000; // NaN
        end else if (is_inf) begin
            fp32_add = {result_sign, 8'hFF, 23'b0}; // Inf
        end else if (a_is_zero && b_is_zero) begin
            fp32_add = {result_sign, 31'b0}; // 带符号的零
        end else begin
            fp32_add = {result_sign, final_exp, final_frac};
        end
    end
endfunction

always @(posedge clk) begin
    if (!rst) begin
        fp_product <= 0;
        fp_acc <= 0;
        new_fp_acc <= 0;
    end else begin
        fp_acc <= a_in;
        fp_product <= b_in;
        new_fp_acc = fp32_add(fp_acc, fp_product);
    end
end

endmodule