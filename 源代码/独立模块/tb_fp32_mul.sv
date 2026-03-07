`timescale 1ns / 1ps

module tb_fp32_mul();

// 测试参数
localparam CLK_PERIOD = 10; // 100MHz时钟

// 输入输出信号
reg clk;
reg [31:0] a_preprocessed;
reg [31:0] b_preprocessed;
wire [31:0] result;

// 实例化被测设计
float_multiplier uut (
    .a(a_preprocessed),
    .b(b_preprocessed),
    .result(result)
);

// 时钟生成
initial begin
    clk = 0;
    forever #(CLK_PERIOD/2) clk = ~clk;
end

// 测试任务：检查结果
task check_result;
    input [31:0] expected;
    input string test_name;
    begin
        // 组合逻辑不需要等待时钟，直接比较
        if (result !== expected) begin
            $display("[ERROR] %s: Expected %h, Got %h", 
                     test_name, expected, result);
        end else begin
            $display("[PASS] %s: Result %h", test_name, result);
        end
        #(CLK_PERIOD);
    end
endtask

// 主测试流程
initial begin
    // 初始化
    a_preprocessed = 0;
    b_preprocessed = 0;
    #(CLK_PERIOD*2);
    
    $display("Starting 40 random test cases...\n");
    
    // ================ 测试用例1-40: 随机数测试 ================
    
    // Test 1
    a_preprocessed = 32'h3F800000; // 1.0
    b_preprocessed = 32'h40000000; // 2.0
    #(CLK_PERIOD);
    $display("[Test 1] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 2
    a_preprocessed = 32'h40400000; // 3.0
    b_preprocessed = 32'h40800000; // 4.0
    #(CLK_PERIOD);
    $display("[Test 2] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 3
    a_preprocessed = 32'hC0000000; // -2.0
    b_preprocessed = 32'h40A00000; // 5.0
    #(CLK_PERIOD);
    $display("[Test 3] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 4
    a_preprocessed = 32'h3FC00000; // 1.5
    b_preprocessed = 32'h40C00000; // 6.0
    #(CLK_PERIOD);
    $display("[Test 4] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 5
    a_preprocessed = 32'h41200000; // 10.0
    b_preprocessed = 32'h3E800000; // 0.25
    #(CLK_PERIOD);
    $display("[Test 5] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 6
    a_preprocessed = 32'h3F000000; // 0.5
    b_preprocessed = 32'h41000000; // 8.0
    #(CLK_PERIOD);
    $display("[Test 6] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 7
    a_preprocessed = 32'h42C80000; // 100.0
    b_preprocessed = 32'h3D4CCCCD; // 0.05
    #(CLK_PERIOD);
    $display("[Test 7] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 8
    a_preprocessed = 32'h40E00000; // 7.0
    b_preprocessed = 32'h41100000; // 9.0
    #(CLK_PERIOD);
    $display("[Test 8] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 9
    a_preprocessed = 32'hC0A00000; // -5.0
    b_preprocessed = 32'hC0400000; // -3.0
    #(CLK_PERIOD);
    $display("[Test 9] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 10
    a_preprocessed = 32'h3DCCCCCD; // 0.1
    b_preprocessed = 32'h41200000; // 10.0
    #(CLK_PERIOD);
    $display("[Test 10] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 11
    a_preprocessed = 32'h447A0000; // 1000.0
    b_preprocessed = 32'h3A83126F; // 0.001
    #(CLK_PERIOD);
    $display("[Test 11] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 12
    a_preprocessed = 32'h40490FDB; // π (3.14159...)
    b_preprocessed = 32'h40000000; // 2.0
    #(CLK_PERIOD);
    $display("[Test 12] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 13
    a_preprocessed = 32'h402DF854; // e (2.71828...)
    b_preprocessed = 32'h40400000; // 3.0
    #(CLK_PERIOD);
    $display("[Test 13] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 14
    a_preprocessed = 32'h00000000; // +0.0
    b_preprocessed = 32'h42960000; // 75.0
    #(CLK_PERIOD);
    $display("[Test 14] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 15
    a_preprocessed = 32'h80000000; // -0.0
    b_preprocessed = 32'h41A00000; // 20.0
    #(CLK_PERIOD);
    $display("[Test 15] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 16
    a_preprocessed = 32'h7F800000; // +Inf
    b_preprocessed = 32'h40000000; // 2.0
    #(CLK_PERIOD);
    $display("[Test 16] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 17
    a_preprocessed = 32'hFF800000; // -Inf
    b_preprocessed = 32'h3F800000; // 1.0
    #(CLK_PERIOD);
    $display("[Test 17] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 18
    a_preprocessed = 32'h7FC00000; // NaN
    b_preprocessed = 32'h40400000; // 3.0
    #(CLK_PERIOD);
    $display("[Test 18] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 19
    a_preprocessed = 32'h7F800000; // +Inf
    b_preprocessed = 32'h00000000; // 0.0
    #(CLK_PERIOD);
    $display("[Test 19] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 20
    a_preprocessed = 32'h3E4CCCCD; // 0.2
    b_preprocessed = 32'h40A00000; // 5.0
    #(CLK_PERIOD);
    $display("[Test 20] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 21
    a_preprocessed = 32'h41880000; // 17.0
    b_preprocessed = 32'h3F19999A; // 0.6
    #(CLK_PERIOD);
    $display("[Test 21] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 22
    a_preprocessed = 32'hC1900000; // -18.0
    b_preprocessed = 32'h3ECCCCCD; // 0.4
    #(CLK_PERIOD);
    $display("[Test 22] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 23
    a_preprocessed = 32'h43480000; // 200.0
    b_preprocessed = 32'h3C23D70A; // 0.01
    #(CLK_PERIOD);
    $display("[Test 23] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 24
    a_preprocessed = 32'h3F266666; // 0.65
    b_preprocessed = 32'h41F00000; // 30.0
    #(CLK_PERIOD);
    $display("[Test 24] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 25
    a_preprocessed = 32'h7F7FFFFF; // Max float
    b_preprocessed = 32'h3F800000; // 1.0
    #(CLK_PERIOD);
    $display("[Test 25] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 26
    a_preprocessed = 32'h00800000; // Min normal
    b_preprocessed = 32'h40000000; // 2.0
    #(CLK_PERIOD);
    $display("[Test 26] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 27
    a_preprocessed = 32'h00000001; // Min denormal
    b_preprocessed = 32'h40000000; // 2.0
    #(CLK_PERIOD);
    $display("[Test 27] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 28
    a_preprocessed = 32'h41640000; // 14.25
    b_preprocessed = 32'h40E00000; // 7.0
    #(CLK_PERIOD);
    $display("[Test 28] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 29
    a_preprocessed = 32'hC1640000; // -14.25
    b_preprocessed = 32'hC0E00000; // -7.0
    #(CLK_PERIOD);
    $display("[Test 29] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 30
    a_preprocessed = 32'h3FA66666; // 1.3
    b_preprocessed = 32'h40533333; // 3.3
    #(CLK_PERIOD);
    $display("[Test 30] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 31
    a_preprocessed = 32'h44FA0000; // 2000.0
    b_preprocessed = 32'h39D1B717; // 0.0004
    #(CLK_PERIOD);
    $display("[Test 31] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 32
    a_preprocessed = 32'h3E99999A; // 0.3
    b_preprocessed = 32'h3F4CCCCD; // 0.8
    #(CLK_PERIOD);
    $display("[Test 32] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 33
    a_preprocessed = 32'h42480000; // 50.0
    b_preprocessed = 32'h3F666666; // 0.9
    #(CLK_PERIOD);
    $display("[Test 33] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 34
    a_preprocessed = 32'h7F000000; // Large number (2^127)
    b_preprocessed = 32'h3F800000; // 1.0
    #(CLK_PERIOD);
    $display("[Test 34] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 35
    a_preprocessed = 32'h7F000000; // Large number
    b_preprocessed = 32'h7F000000; // Large number
    #(CLK_PERIOD);
    $display("[Test 35] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 36
    a_preprocessed = 32'h3FAAAAAB; // 1.33333...
    b_preprocessed = 32'h40000000; // 2.0
    #(CLK_PERIOD);
    $display("[Test 36] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 37
    a_preprocessed = 32'h41B40000; // 22.5
    b_preprocessed = 32'h3E800000; // 0.25
    #(CLK_PERIOD);
    $display("[Test 37] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 38
    a_preprocessed = 32'hC2C80000; // -100.0
    b_preprocessed = 32'h3DCCCCCD; // 0.1
    #(CLK_PERIOD);
    $display("[Test 38] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 39
    a_preprocessed = 32'h3F333333; // 0.7
    b_preprocessed = 32'h41300000; // 11.0
    #(CLK_PERIOD);
    $display("[Test 39] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    // Test 40
    a_preprocessed = 32'h40B00000; // 5.5
    b_preprocessed = 32'h40600000; // 3.5
    #(CLK_PERIOD);
    $display("[Test 40] a=%h, b=%h, result=%h", a_preprocessed, b_preprocessed, result);
    
    $display("\nAll 40 random test cases completed!");
    $finish;
end

// 波形记录
initial begin
    $dumpfile("fp32_mul.vcd");
    $dumpvars(0, tb_fp32_mul);
end

endmodule
