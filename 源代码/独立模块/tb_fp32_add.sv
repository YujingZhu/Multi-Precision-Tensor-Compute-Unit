`timescale 1ns / 1ps

module tb_fp32_add();
    reg clk;
    reg rst;
    reg [31:0] a_in;
    reg [31:0] b_in;
    wire [31:0] new_fp_acc;

    // 自动测试变量声明（必须提前）
    integer i;
    real ra, rb;
    reg [31:0] fpa, fpb;

    real expected, actual, error;
    real epsilon = 1e-4;  // 容差，可调

    // 实例化被测模块
    fp32_add uut (
        .clk(clk),
        .rst(rst),
        .a_in(a_in),
        .b_in(b_in),
        .new_fp_acc(new_fp_acc)
    );

    // 时钟生成
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // 浮点数显示任务
    task display_fp;
        input [31:0] fp;
        real value;
        begin
            value = $bitstoshortreal(fp);
            $display("Hex: %h, Float: %f", fp, value);
        end
    endtask

    // 主测试过程
    initial begin
        // 初始化
        rst = 0;
        a_in = 0;
        b_in = 0;
        #20;

        // 释放复位
        rst = 1;
        #10;

        $display("=== FP32加法器测试开始 ===");

        // 1. 常规数值测试
        $display("\n[测试1] 常规加法: 1.5 + 2.25 = 3.75");
        a_in = $shortrealtobits(1.5);
        b_in = $shortrealtobits(2.25);
        #10;
        $display("输入A:"); display_fp(a_in);
        $display("输入B:"); display_fp(b_in);
        $display("结果:"); display_fp(new_fp_acc);

        // 1. 常规数值测试
        $display("\n[测试1] 常规加法: 1.0 + 2.0 = 3.0");
        a_in = $shortrealtobits(1.0);
        b_in = $shortrealtobits(2.0);
        #10;
        $display("输入A:"); display_fp(a_in);
        $display("输入B:"); display_fp(b_in);
        $display("结果:"); display_fp(new_fp_acc);

        // 1. 常规数值测试
        $display("\n[测试1] 常规加法: 3.0 + 4.0 = 7.0");
        a_in = $shortrealtobits(3.0);
        b_in = $shortrealtobits(4.0);
        #10;
        $display("输入A:"); display_fp(a_in);
        $display("输入B:"); display_fp(b_in);
        $display("结果:"); display_fp(new_fp_acc);


        // 1. 常规数值测试
        $display("\n[测试1] 常规加法: 7.0 + 13.0 = 20.0");
        a_in = $shortrealtobits(7.0);
        b_in = $shortrealtobits(13.0);
        #10;
        $display("输入A:"); display_fp(a_in);
        $display("输入B:"); display_fp(b_in);
        $display("结果:"); display_fp(new_fp_acc);

        // 1. 常规数值测试
        $display("\n[测试1] 常规加法: 87.0 + 184.0 = 271.0");
        a_in = $shortrealtobits(87.0);
        b_in = $shortrealtobits(184.0);
        #10;
        $display("输入A:"); display_fp(a_in);
        $display("输入B:"); display_fp(b_in);
        $display("结果:"); display_fp(new_fp_acc);

        // 2. 零值测试
        $display("\n[测试2] 零值加法: 0.0 + 3.0 = 3.0");
        a_in = $shortrealtobits(0.0);
        b_in = $shortrealtobits(3.0);
        #10;
        $display("输入A:"); display_fp(a_in);
        $display("输入B:"); display_fp(b_in);
        $display("结果:"); display_fp(new_fp_acc);

        // 3. 符号相反测试
        $display("\n[测试3] 符号相反: 1.5 + (-2.25) = -0.75");
        a_in = $shortrealtobits(1.5);
        b_in = $shortrealtobits(-2.25);
        #10;
        $display("输入A:"); display_fp(a_in);
        $display("输入B:"); display_fp(b_in);
        $display("结果:"); display_fp(new_fp_acc);

        // 4. 无穷大测试
        $display("\n[测试4] 无穷大加法: Inf + 1.0 = Inf");
        a_in = {1'b0, 8'hFF, 23'h0};  // +Inf
        b_in = $shortrealtobits(1.0);
        #10;
        $display("输入A:"); display_fp(a_in);
        $display("输入B:"); display_fp(b_in);
        $display("结果:"); display_fp(new_fp_acc);

        // 5. NaN测试
        $display("\n[测试5] NaN加法: NaN + 1.0 = NaN");
        a_in = {1'b0, 8'hFF, 23'h400000};  // NaN
        b_in = $shortrealtobits(1.0);
        #10;
        $display("输入A:"); display_fp(a_in);
        $display("输入B:"); display_fp(b_in);
        $display("结果:"); display_fp(new_fp_acc);

        // 6. 非规格化数测试
        $display("\n[测试6] 非规格化数加法: 最小非规格化数 + 0.0");
        a_in = {1'b0, 8'h00, 23'h000001};  // ≈1.4e-45
        b_in = $shortrealtobits(0.0);
        #10;
        $display("输入A:"); display_fp(a_in);
        $display("输入B:"); display_fp(b_in);
        $display("结果:"); display_fp(new_fp_acc);

        // 7. 边界条件测试（指数差>26）
        $display("\n[测试7] 大指数差: 1.0 + 1e-30 ≈ 1.0");
        a_in = $shortrealtobits(1.0);
        b_in = $shortrealtobits(1.0e-30);
        #10;
        $display("输入A:"); display_fp(a_in);
        $display("输入B:"); display_fp(b_in);
        $display("结果:"); display_fp(new_fp_acc);

        // 8. 舍入测试（向偶舍入）
        $display("\n[测试8] 舍入测试: 1.0 + 2^(-24)");
        a_in = $shortrealtobits(1.0);
        b_in = {1'b0, 8'h6A, 23'h0};  // 2^(-24)
        #10;
        $display("输入A:"); display_fp(a_in);
        $display("输入B:"); display_fp(b_in);
        $display("结果:"); display_fp(new_fp_acc);

        // 9. 正负零测试
        $display("\n[测试9] 正负零加法: +0.0 + (-0.0) = +0.0");
        a_in = 32'h00000000;  // +0.0
        b_in = 32'h80000000;  // -0.0
        #10;
        $display("输入A:"); display_fp(a_in);
        $display("输入B:"); display_fp(b_in);
        $display("结果:"); display_fp(new_fp_acc);

        // 10. 溢出测试
        $display("\n[测试10] 溢出测试: 最大规格化数 + 最大规格化数 = Inf");
        a_in = {1'b0, 8'hFE, 23'h7FFFFF};  // ≈3.4e38
        b_in = a_in;
        #10;
        $display("输入A:"); display_fp(a_in);
        $display("输入B:"); display_fp(b_in);
        $display("结果:"); display_fp(new_fp_acc);

        // 11. 自动测试：生成40组 [0~1000] 的浮点加法测试
        $display("\n[测试11] 自动生成40组 [0~1000] 浮点加法：");
        
        for (i = 0; i < 40; i = i + 1) begin
            // 生成浮点数输入
            ra = i * 17.3;
            if (ra >= 1000.0) ra = ra - 1000.0;

            rb = i * 29.7 + 13.1;
            if (rb >= 1000.0) rb = rb - 1000.0;

            fpa = $shortrealtobits(ra);
            fpb = $shortrealtobits(rb);
            a_in = fpa;
            b_in = fpb;

            #10;

            // 比较与报告
            expected = ra + rb;
            actual = $bitstoshortreal(new_fp_acc);
            error = expected - actual;
            if (error < 0) error = -error;

            $display("\n[自动测试 %0d] %f + %f", i+1, ra, rb);
            $display("输入A:"); display_fp(fpa);
            $display("输入B:"); display_fp(fpb);
            $display("期望值: %f, 实际输出: %f, 误差: %f", expected, actual, error);

            if (error < epsilon) begin
                $display("? 测试通过！");
            end else begin
                $display("? 测试失败！");
            end
        end




        // 结束测试
        #100;
        $display("\n=== 测试完成 ===");
        $finish;
    end
endmodule
