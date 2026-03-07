`timescale 1ns / 1ps

module ProcessingElement_tb_fp32;

    // 时钟和复位信号
    reg          clk;
    reg          rst_n;
    
    // PE控制信号
    reg          pe_enable;
    reg  [1:0]   precision_mode;
    reg          pe_load_c_en;
    
    // 数据输入
    reg  [31:0]  a_in;
    reg  [31:0]  b_in;
    reg  [31:0]  c_in;
    
    // 输出
    wire [31:0]  a_out;
    wire [31:0]  b_out;
    wire [31:0]  d_out;
    wire         overflow;
    wire signed [63:0]  int_acc;
    wire [31:0]  fp_acc;

    // 实例化被测模块
    ProcessingElement pe (
        .clk(clk),
        .rst_n(rst_n),
        .pe_enable(pe_enable),
        .precision_mode(precision_mode),
        .pe_load_c_en(pe_load_c_en),
        .a_in(a_in),
        .b_in(b_in),
        .c_in(c_in),
        .a_out(a_out),
        .b_out(b_out),
        .d_out(d_out),
        .overflow(overflow),
        .int_acc(int_acc),
        .fp_acc(fp_acc)
    );

    // 测试数据 - FP32格式
    reg [31:0] test_data_a [0:7];
    reg [31:0] test_data_b [0:7];
    reg [31:0] expected_results [0:7];
    
    integer cycle_idx;
    integer error_count;

    // 时钟生成（100MHz）
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // 初始化测试数据
    initial begin
        // A矩阵数据 (简化为8个数据)
        test_data_a[0] = 32'h3F800000; // 1.0
        test_data_a[1] = 32'h40000000; // 2.0
        test_data_a[2] = 32'h40400000; // 3.0
        test_data_a[3] = 32'h40800000; // 4.0
        test_data_a[4] = 32'h40A00000; // 5.0
        test_data_a[5] = 32'h40C00000; // 6.0
        test_data_a[6] = 32'h40E00000; // 7.0
        test_data_a[7] = 32'h41000000; // 8.0
        
        // B矩阵数据
        test_data_b[0] = 32'h40000000; // 2.0
        test_data_b[1] = 32'h40400000; // 3.0
        test_data_b[2] = 32'h40800000; // 4.0
        test_data_b[3] = 32'h40A00000; // 5.0
        test_data_b[4] = 32'h40C00000; // 6.0
        test_data_b[5] = 32'h40E00000; // 7.0
        test_data_b[6] = 32'h41000000; // 8.0
        test_data_b[7] = 32'h41100000; // 9.0
        
        expected_results[0] = 32'h40400000; // 3.0
        expected_results[1] = 32'h41100000; // 9.0
        expected_results[2] = 32'h41A80000; // 21.0
        expected_results[3] = 32'h42240000; // 41.0
        expected_results[4] = 32'h428E0000; // 71.0
        expected_results[5] = 32'h42E20000; // 113.0
        expected_results[6] = 32'h43290000; // 169.0
        expected_results[7] = 32'h43710000; // 241.0
    end

    // 主测试序列
    initial begin
        // 初始化
        rst_n = 0;
        pe_enable = 0;
        precision_mode = 2'b11; // FP32模式
        pe_load_c_en = 0;
        a_in = 0;
        b_in = 0;
        c_in = 0;
        cycle_idx = 0;
        error_count = 0;

        $display("=== ProcessingElement FP32 Test Start ===");

        // 复位
        #20;
        rst_n = 1;
        #10;
        $display("[%0t] Reset completed", $time);

        // 第一阶段：加载C值
        @(posedge clk);
        pe_load_c_en = 1;
        precision_mode = 2'b11; // FP32模式
        c_in = 32'h3F800000; // C = 1.0
        
        @(posedge clk);
        pe_load_c_en = 0;
        $display("[%0t] C value loaded: 1.0", $time);

        // 第二阶段：开始矩阵乘法计算
        @(posedge clk);
        pe_enable = 1;
        precision_mode = 2'b11; // FP32模式
        
        $display("\nCycle |   A_in      |   B_in      |   D_out     | Expected    | Status");
        $display("-----------------------------------------------------------------------");

        // 执行8个周期的计算
        for (cycle_idx = 0; cycle_idx < 8; cycle_idx = cycle_idx + 1) begin
            @(posedge clk);
            a_in = test_data_a[cycle_idx];
            b_in = test_data_b[cycle_idx];
            
            
            // 检查结果
            $display("%4d  | 0x%08h | 0x%08h | 0x%08h | 0x%08h | %s",
                    cycle_idx,
                    test_data_a[cycle_idx],
                    test_data_b[cycle_idx],
                    d_out,
                    expected_results[cycle_idx],
                    (d_out == expected_results[cycle_idx]) ? "PASS" : "FAIL");
            
            if (d_out !== expected_results[cycle_idx]) begin
                error_count = error_count + 1;
                $display("    ERROR: Expected 0x%08h, Got 0x%08h", 
                        expected_results[cycle_idx], d_out);
            end
            
            // 检查溢出
            if (overflow) begin
                $display("    WARNING: Overflow detected");
            end
        end

        // 测试总结
        pe_enable = 0;
        #20;
        
        $display("\n=== Test Summary ===");
        $display("Total Tests: 8");
        $display("Passed: %0d", 8 - error_count);
        $display("Failed: %0d", error_count);
        $display("Pass Rate: %.1f%%", (8.0 - error_count) / 8.0 * 100.0);
        
        if (error_count == 0) begin
            $display("\n=== ALL TESTS PASSED ===");
        end else begin
            $display("\n=== SOME TESTS FAILED ===");
        end

        #50;
        $finish;
    end

    // 监控关键信号
    initial begin
        $monitor("[%0t] pe_enable=%b, d_out=0x%08h, fp_acc=0x%08h, overflow=%b", 
                $time, pe_enable, d_out, fp_acc, overflow);
    end

endmodule