function [31:0] fp32_add;
    input [31:0] a, b;
    
    reg [7:0] exp_a, exp_b, exp_max;
    reg [24:0] man_a, man_b, man_sum;
    reg sign_a, sign_b, sign_result;
    reg [4:0] shift_amt;
    reg [31:0] result;
    
    begin
        // 提取字段
        sign_a = a[31];
        sign_b = b[31];
        exp_a = a[30:23];
        exp_b = b[30:23];
        
        // 特殊值检查
        if (exp_a == 8'hFF || exp_b == 8'hFF) begin
            // 处理无穷大和NaN
            if ((exp_a == 8'hFF && a[22:0] != 0) || (exp_b == 8'hFF && b[22:0] != 0)) begin
                result = 32'hFFC00000; // NaN
            end else if (exp_a == 8'hFF && exp_b == 8'hFF && sign_a != sign_b) begin
                result = 32'hFFC00000; // inf - inf = NaN
            end else if (exp_a == 8'hFF) begin
                result = a; // 返回a的无穷大
            end else begin
                result = b; // 返回b的无穷大
            end
        end else if (a == 32'h00000000) begin
            result = b;
        end else if (b == 32'h00000000) begin
            result = a;
        end else begin
            // 构建尾数（包含隐含的1位）
            man_a = (exp_a == 0) ? {2'b00, a[22:0]} : {2'b01, a[22:0]};
            man_b = (exp_b == 0) ? {2'b00, b[22:0]} : {2'b01, b[22:0]};
            
            // 处理指数为0的情况
            if (exp_a == 0) exp_a = 8'h01;
            if (exp_b == 0) exp_b = 8'h01;
            
            // 对齐尾数到较大的指数
            if (exp_a >= exp_b) begin
                exp_max = exp_a;
                shift_amt = exp_a - exp_b;
                if (shift_amt > 24) begin
                    man_b = 25'h0;
                end else begin
                    man_b = man_b >> shift_amt;
                end
            end else begin
                exp_max = exp_b;
                shift_amt = exp_b - exp_a;
                if (shift_amt > 24) begin
                    man_a = 25'h0;
                end else begin
                    man_a = man_a >> shift_amt;
                end
            end
            
            // 执行加法或减法
            if (sign_a == sign_b) begin
                // 同号相加
                man_sum = man_a + man_b;
                sign_result = sign_a;
                
                // 检查是否需要右移
                if (man_sum[24]) begin
                    man_sum = man_sum >> 1;
                    exp_max = exp_max + 1;
                end
            end else begin
                // 异号相减
                if (man_a >= man_b) begin
                    man_sum = man_a - man_b;
                    sign_result = sign_a;
                end else begin
                    man_sum = man_b - man_a;
                    sign_result = sign_b;
                end
                
                // 简单的左移处理（只处理最常见的情况）
                if (man_sum[23] == 0 && man_sum != 0) begin
                    if (man_sum[22]) begin
                        man_sum = man_sum << 1;
                        exp_max = exp_max - 1;
                    end else if (man_sum[21]) begin
                        man_sum = man_sum << 2;
                        exp_max = exp_max - 2;
                    end else if (man_sum[20]) begin
                        man_sum = man_sum << 3;
                        exp_max = exp_max - 3;
                    end
                    // 更多位数的左移可以继续扩展
                end
            end
            
            // 打包结果
            if (man_sum == 0) begin
                result = 32'h00000000; // 零
            end else if (exp_max >= 8'hFF) begin
                result = {sign_result, 8'hFF, 23'h0}; // 无穷大
            end else if (exp_max == 0) begin
                result = {sign_result, 8'h00, man_sum[22:0]}; // 非规格化数
            end else begin
                result = {sign_result, exp_max, man_sum[22:0]};
            end
        end
        
        fp32_add = result;
    end
endfunction


function [31:0] fp32_mult;
    input [31:0] a;
    input [31:0] b;
    
    // 内部变量声明
    reg a_sign, b_sign, z_sign;
    reg [7:0] a_exp, b_exp;
    reg [22:0] a_frac, b_frac;
    reg a_zero, b_zero, a_inf, b_inf, a_nan, b_nan;
    reg a_denorm, b_denorm;
    reg [23:0] a_man, b_man;
    reg [7:0] a_exp_adj, b_exp_adj;
    reg [47:0] product;
    reg signed [9:0] exp_sum;
    reg need_shift;
    reg [47:0] norm_product;
    reg signed [9:0] norm_exp;
    reg [24:0] pre_round;
    reg guard, round_bit, sticky;
    reg round_up;
    reg [24:0] rounded;
    reg round_overflow;
    reg [22:0] final_frac;
    reg signed [9:0] final_exp;
    reg underflow, overflow;
    reg result_nan, result_inf, result_zero;
    reg [31:0] normal_result, zero_result, inf_result, nan_result;
    
    begin
        // 提取符号位、指数和尾数
        a_sign = a[31];
        b_sign = b[31];
        a_exp = a[30:23];
        b_exp = b[30:23];
        a_frac = a[22:0];
        b_frac = b[22:0];
        
        // 计算结果符号位
        z_sign = a_sign ^ b_sign;
        
        // 特殊值检测
        a_zero = (a[30:0] == 31'b0);
        b_zero = (b[30:0] == 31'b0);
        a_inf = (a_exp == 8'hFF) && (a_frac == 0);
        b_inf = (b_exp == 8'hFF) && (b_frac == 0);
        a_nan = (a_exp == 8'hFF) && (a_frac != 0);
        b_nan = (b_exp == 8'hFF) && (b_frac != 0);
        
        // 非正规数检测
        a_denorm = (a_exp == 0) && (a_frac != 0);
        b_denorm = (b_exp == 0) && (b_frac != 0);
        
        // 构造尾数（24位，包含隐含位）
        a_man = (a_exp == 0) ? {1'b0, a_frac} : {1'b1, a_frac};
        b_man = (b_exp == 0) ? {1'b0, b_frac} : {1'b1, b_frac};
        
        // 调整指数（非正规数的有效指数是1）
        a_exp_adj = (a_exp == 0) ? 8'd1 : a_exp;
        b_exp_adj = (b_exp == 0) ? 8'd1 : b_exp;
        
        // 尾数相乘（48位结果）
        product = a_man * b_man;
        
        // 指数相加（使用足够宽度防止溢出）
        exp_sum = {2'b0, a_exp_adj} + {2'b0, b_exp_adj} - 10'd127;
        
        // 规格化：检查乘积的最高位
        need_shift = ~product[47];
        norm_product = need_shift ? (product << 1) : product;
        norm_exp = exp_sum - (need_shift ? 1 : 0);
        
        // 提取舍入位
        pre_round = norm_product[47:23];
        guard = norm_product[22];
        round_bit = norm_product[21];
        sticky = |norm_product[20:0];
        
        // IEEE 754舍入到最近偶数
        round_up = guard & (pre_round[0] | round_bit | sticky);
        rounded = pre_round + round_up;
        
        // 检查舍入后的进位
        round_overflow = rounded[24];
        final_frac = round_overflow ? rounded[23:1] : rounded[22:0];
        final_exp = norm_exp + (round_overflow ? 1 : 0);
        
        // 检查下溢和上溢
        underflow = (final_exp <= 0) || (product == 0);
        overflow = (final_exp >= 255);
        
        // 特殊情况判断
        result_nan = a_nan | b_nan | (a_inf & b_zero) | (b_inf & a_zero);
        result_inf = ((a_inf | b_inf) & ~result_nan) | overflow;
        result_zero = (a_zero | b_zero) | underflow;
        
        // 最终结果组合
        normal_result = {z_sign, final_exp[7:0], final_frac};
        zero_result = {z_sign, 31'b0};
        inf_result = {z_sign, 8'hFF, 23'b0};
        nan_result = {1'b0, 8'hFF, 1'b1, 22'b0};
        
        fp32_mult = result_nan ? nan_result :
                        result_inf ? inf_result :
                        result_zero ? zero_result :
                        normal_result;
    end
endfunction

function [63:0] fp64_mult;
    input  [63:0] a, b;
    
    // 内部变量声明
    reg a_sign, b_sign, z_sign;
    reg [10:0] a_exp, b_exp;
    reg [51:0] a_frac, b_frac;
    reg a_zero, b_zero, a_inf, b_inf, a_nan, b_nan;
    reg a_denorm, b_denorm;
    reg [52:0] a_man, b_man;
    reg [10:0] a_exp_adj, b_exp_adj;
    reg [105:0] product;
    reg signed [12:0] exp_sum;
    reg need_shift;
    reg [105:0] norm_product;
    reg signed [12:0] norm_exp;
    reg [53:0] pre_round;
    reg guard, round_bit, sticky;
    reg round_up;
    reg [53:0] rounded;
    reg round_overflow;
    reg [51:0] final_frac;
    reg signed [12:0] final_exp;
    reg underflow, overflow;
    reg result_nan, result_inf, result_zero;
    reg [63:0] normal_result, zero_result, inf_result, nan_result;
    
    begin
        // 提取符号位、指数和尾数
        a_sign = a[63];
        b_sign = b[63];
        a_exp = a[62:52];
        b_exp = b[62:52];
        a_frac = a[51:0];
        b_frac = b[51:0];
        
        // 计算结果符号位
        z_sign = a_sign ^ b_sign;
        
        // 特殊值检测
        a_zero = (a[62:0] == 63'b0);
        b_zero = (b[62:0] == 63'b0);
        a_inf = (a_exp == 11'h7FF) && (a_frac == 0);
        b_inf = (b_exp == 11'h7FF) && (b_frac == 0);
        a_nan = (a_exp == 11'h7FF) && (a_frac != 0);
        b_nan = (b_exp == 11'h7FF) && (b_frac != 0);
        
        // 非正规数检测
        a_denorm = (a_exp == 0) && (a_frac != 0);
        b_denorm = (b_exp == 0) && (b_frac != 0);
        
                
        // 构造尾数（53位，包含隐含位）
        a_man = (a_exp == 0) ? {1'b0, a_frac} : {1'b1, a_frac};
        b_man = (b_exp == 0) ? {1'b0, b_frac} : {1'b1, b_frac};
        
        
        // 调整指数（非正规数的有效指数是1）
        a_exp_adj = (a_exp == 0) ? 11'd1 : a_exp;
        b_exp_adj = (b_exp == 0) ? 11'd1 : b_exp;
        
        // 尾数相乘（48位结果）
        product = a_man * b_man;
        
        // 指数相加（使用足够宽度防止溢出）
        exp_sum = {2'b0, a_exp_adj} + {2'b0, b_exp_adj} - 13'd1023;
        
        // 规格化：检查乘积的最高位
        need_shift = ~product[105];
        norm_product = need_shift ? (product << 1) : product;
        norm_exp = exp_sum - (need_shift ? 1 : 0);
        
        // 提取舍入位
        pre_round = norm_product[105:52];
        guard = norm_product[51];
        round_bit = norm_product[49];
        sticky = |norm_product[48:0];
        
        // IEEE 754舍入到最近偶数
        round_up = guard & (pre_round[0] | round_bit | sticky);
        rounded = pre_round + round_up;
        
        // 检查舍入后的进位
        round_overflow = rounded[53];
        final_frac = round_overflow ? rounded[52:1] : rounded[51:0];
        final_exp = norm_exp + (round_overflow ? 1 : 0);
        
        // 检查下溢和上溢
        underflow = (final_exp <= 0) || (product == 0);
        overflow = (final_exp > 2048);
        
        // 特殊情况判断
        result_nan = a_nan | b_nan | (a_inf & b_zero) | (b_inf & a_zero);
        result_inf = ((a_inf | b_inf) & ~result_nan) | overflow;
        result_zero = (a_zero | b_zero) | underflow;
        
        // 最终结果组合
        normal_result = {z_sign, final_exp[10:0], final_frac};
        zero_result = {z_sign, 63'b0};
        inf_result = {z_sign, 11'h7FF, 52'b0};
        nan_result = {1'b0, 11'h7FF, 1'b1, 51'b0};
        
        fp64_mult = result_nan ? nan_result :
                        result_inf ? inf_result :
                        result_zero ? zero_result :
                        normal_result;
    end    

endfunction

function [63:0] fp64_add;
    input [63:0] a, b;
    
    reg [10:0] exp_a, exp_b, exp_max;
    reg [53:0] man_a, man_b, man_sum;
    reg sign_a, sign_b, sign_result;
    reg [8:0] shift_amt;
    reg [63:0] result;
    
    begin
        // 提取字段
        sign_a = a[63];
        sign_b = b[63];
        exp_a = a[62:52];
        exp_b = b[62:52];
        
        // 特殊值检查
        if (exp_a == 11'h7FF || exp_b == 11'h7FF) begin
            // 处理无穷大和NaN
            if ((exp_a == 11'h7FF && a[51:0] != 0) || (exp_b == 11'h7FF && b[51:0] != 0)) begin
                result = 64'h7FF8000000000000; // NaN
            end else if (exp_a == 11'h7FF && exp_b == 11'h7FF && sign_a != sign_b) begin
                result = 64'h7FF8000000000000; // inf - inf = NaN
            end else if (exp_a == 11'h7FF) begin
                result = a; // 返回a的无穷大
            end else begin
                result = b; // 返回b的无穷大
            end
        end else if (a == 64'h00000000) begin
            result = b;
        end else if (b == 64'h00000000) begin
            result = a;
        end else begin
            // 构建尾数（包含隐含的1位）
            man_a = (exp_a == 0) ? {2'b00, a[51:0]} : {2'b01, a[51:0]};
            man_b = (exp_b == 0) ? {2'b00, b[51:0]} : {2'b01, b[51:0]};
            
            // 处理指数为0的情况
            if (exp_a == 0) exp_a = 11'h01;
            if (exp_b == 0) exp_b = 11'h01;
            
            // 对齐尾数到较大的指数
            if (exp_a >= exp_b) begin
                exp_max = exp_a;
                shift_amt = exp_a - exp_b;
                if (shift_amt > 24) begin
                    man_b = 53'h0;
                end else begin
                    man_b = man_b >> shift_amt;
                end
            end else begin
                exp_max = exp_b;
                shift_amt = exp_b - exp_a;
                if (shift_amt > 24) begin
                    man_a = 53'h0;
                end else begin
                    man_a = man_a >> shift_amt;
                end
            end
            
            // 执行加法或减法
            if (sign_a == sign_b) begin
                // 同号相加
                man_sum = man_a + man_b;
                sign_result = sign_a;
                
                // 检查是否需要右移
                if (man_sum[53]) begin
                    man_sum = man_sum >> 1;
                    exp_max = exp_max + 1;
                end
            end else begin
                // 异号相减
                if (man_a >= man_b) begin
                    man_sum = man_a - man_b;
                    sign_result = sign_a;
                end else begin
                    man_sum = man_b - man_a;
                    sign_result = sign_b;
                end
                
                // 简单的左移处理（只处理最常见的情况）
                if (man_sum[52] == 0 && man_sum != 0) begin
                    if (man_sum[51]) begin
                        man_sum = man_sum << 1;
                        exp_max = exp_max - 1;
                    end else if (man_sum[50]) begin
                        man_sum = man_sum << 2;
                        exp_max = exp_max - 2;
                    end else if (man_sum[49]) begin
                        man_sum = man_sum << 3;
                        exp_max = exp_max - 3;
                    end
                    // 更多位数的左移可以继续扩展
                end
            end
            
            // 打包结果
            if (man_sum == 0) begin
                result = 64'h00000000; // 零
            end else if (exp_max >= 11'h7FF) begin
                result = {sign_result, 11'hFF, 52'h0}; // 无穷大
            end else if (exp_max == 0) begin
                result = {sign_result, 11'h00, man_sum[51:0]}; // 非规格化数
            end else begin
                result = {sign_result, exp_max, man_sum[51:0]};
            end
        end
        
        fp64_add = result;
    end
    
endfunction


function [63:0] fp32_to_fp64;
        input [31:0] fp32;
        reg [63:0] z;
        reg [10:0] z_e;
        reg [52:0] z_m;
        reg [5:0] shift_count; // 最多需要23次移位(对于非规格化数)
        begin
            // 默认值
            z[63] = fp32[31];  // 符号位
    
            // 处理指数和尾数
            if (fp32[30:23] == 8'hFF) begin
                // NaN 或 Inf
                z[62:52] = 11'h7FF;  // FP64 指数全1
                z[51:0] = (fp32[22:0] != 0) ? {1'b1, 51'b0} : 52'b0;  // NaN 或 Inf
            end
            else if (fp32[30:23] == 0) begin
                // 零或非规格化数
                if (fp32[22:0] == 0) begin
                    // 零
                    z[62:52] = 0;
                    z[51:0] = 0;
                end else begin
                    // 非规格化数：规格化处理
                    z_m = {1'b0, fp32[22:0], 29'b0};  // 隐含前导0
                    z_e = 11'd897;  // FP64 偏置1023 - FP32实际指数-126
                    
                    // 可综合的移位操作 - 替代while循环
                    shift_count = 0;
                    
                    // 完全展开的移位判断
                    if (!z_m[52]) begin shift_count = shift_count + 1; z_m = z_m << 1; end
                    if (!z_m[52]) begin shift_count = shift_count + 1; z_m = z_m << 1; end
                    if (!z_m[52]) begin shift_count = shift_count + 1; z_m = z_m << 1; end
                    if (!z_m[52]) begin shift_count = shift_count + 1; z_m = z_m << 1; end
                    if (!z_m[52]) begin shift_count = shift_count + 1; z_m = z_m << 1; end
                    if (!z_m[52]) begin shift_count = shift_count + 1; z_m = z_m << 1; end
                    if (!z_m[52]) begin shift_count = shift_count + 1; z_m = z_m << 1; end
                    if (!z_m[52]) begin shift_count = shift_count + 1; z_m = z_m << 1; end
                    if (!z_m[52]) begin shift_count = shift_count + 1; z_m = z_m << 1; end
                    if (!z_m[52]) begin shift_count = shift_count + 1; z_m = z_m << 1; end
                    if (!z_m[52]) begin shift_count = shift_count + 1; z_m = z_m << 1; end
                    if (!z_m[52]) begin shift_count = shift_count + 1; z_m = z_m << 1; end
                    if (!z_m[52]) begin shift_count = shift_count + 1; z_m = z_m << 1; end
                    if (!z_m[52]) begin shift_count = shift_count + 1; z_m = z_m << 1; end
                    if (!z_m[52]) begin shift_count = shift_count + 1; z_m = z_m << 1; end
                    if (!z_m[52]) begin shift_count = shift_count + 1; z_m = z_m << 1; end
                    if (!z_m[52]) begin shift_count = shift_count + 1; z_m = z_m << 1; end
                    if (!z_m[52]) begin shift_count = shift_count + 1; z_m = z_m << 1; end
                    if (!z_m[52]) begin shift_count = shift_count + 1; z_m = z_m << 1; end
                    if (!z_m[52]) begin shift_count = shift_count + 1; z_m = z_m << 1; end
                    if (!z_m[52]) begin shift_count = shift_count + 1; z_m = z_m << 1; end
                    if (!z_m[52]) begin shift_count = shift_count + 1; z_m = z_m << 1; end
                    if (!z_m[52]) begin shift_count = shift_count + 1; z_m = z_m << 1; end
                    
                    z_e = z_e - shift_count;
                    z[62:52] = z_e;
                    z[51:0] = z_m[51:0];
                end
            end
            else begin
                // 规格化数
                z[62:52] = {3'b0, fp32[30:23]} + 11'd896;  // FP32指数+896 (1023-127)
                z[51:0] = {fp32[22:0], 29'b0};  // 尾数右补29位
            end
    
            fp32_to_fp64 = z;
        end
endfunction