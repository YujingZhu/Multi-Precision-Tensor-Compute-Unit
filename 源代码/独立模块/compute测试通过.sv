`timescale 1ns/1ps

module tb_tensor_compute_unit();

// 参数设置
parameter MAX_M = 4;
parameter MAX_N = 4;
parameter CLK_PERIOD = 10;  // 100MHz时钟

// 状态定义
localparam IDLE     = 4'd0;
localparam LOAD_C   = 4'd1;
localparam LOAD_A   = 4'd2;
localparam LOAD_B   = 4'd3;
localparam COMPUTE  = 4'd4;
localparam OUTPUT   = 4'd5;

// 信号声明
reg clk;
reg rst_n;
reg [1:0] precision_mode;
reg [5:0] m, n, k;
reg [3:0] state;
reg [8:0] cycle_counter;
reg [5:0] compute_counter;
reg pe_enable;
reg pe_load_c_en;

wire overflow;
wire [31:0] d_out;

// 数据存储器
reg [31:0] a_data [0:MAX_M-1][0:MAX_N-1];
reg [31:0] b_data [0:MAX_M-1][0:MAX_N-1];
reg [31:0] c_data [0:MAX_M-1][0:MAX_N-1];

// 实例化被测设计
tensor_compute_unit #(
    .MAX_M(MAX_M),
    .MAX_N(MAX_N)
) dut (
    .clk(clk),
    .rst_n(rst_n),
    .precision_mode(precision_mode),
    .overflow(overflow),
    .d_out(d_out),
    .m(m),
    .n(n),
    .k(k),
    .state(state),
    .cycle_counter(cycle_counter),
    .compute_counter(compute_counter),
    .pe_enable(pe_enable),
    .pe_load_c_en(pe_load_c_en),
    .a_data(a_data),
    .b_data(b_data),
    .c_data(c_data)
);

// 时钟生成
initial begin
    clk = 0;
    forever #(CLK_PERIOD/2) clk = ~clk;
end


// FP32数据初始化
initial begin
    // A矩阵
    a_data[0][0] = 32'h3F800000; // 1.0
    a_data[0][1] = 32'h40000000; // 2.0
    a_data[0][2] = 32'h40400000; // 3.0
    a_data[0][3] = 32'h40800000; // 4.0
    a_data[1][0] = 32'h40A00000; // 5.0
    a_data[1][1] = 32'h40C00000; // 6.0
    a_data[1][2] = 32'h40E00000; // 7.0
    a_data[1][3] = 32'h41000000; // 8.0
    a_data[2][0] = 32'h41100000; // 9.0
    a_data[2][1] = 32'h41200000; // 10.0
    a_data[2][2] = 32'h41300000; // 11.0
    a_data[2][3] = 32'h41400000; // 12.0
    a_data[3][0] = 32'h41500000; // 13.0
    a_data[3][1] = 32'h41600000; // 14.0
    a_data[3][2] = 32'h41700000; // 15.0
    a_data[3][3] = 32'h41800000; // 16.0

    // B矩阵：简单的FP32数值
    b_data[0][0] = 32'h41800000; // 16.0
    b_data[0][1] = 32'h41700000; // 15.0
    b_data[0][2] = 32'h41600000; // 14.0
    b_data[0][3] = 32'h41500000; // 13.0
    b_data[1][0] = 32'h41400000; // 12.0
    b_data[1][1] = 32'h41300000; // 11.0
    b_data[1][2] = 32'h41200000; // 10.0
    b_data[1][3] = 32'h41100000; // 9.0
    b_data[2][0] = 32'h41000000; // 8.0
    b_data[2][1] = 32'h40E00000; // 7.0
    b_data[2][2] = 32'h40C00000; // 6.0
    b_data[2][3] = 32'h40A00000; // 5.0
    b_data[3][0] = 32'h40800000; // 4.0
    b_data[3][1] = 32'h40400000; // 3.0
    b_data[3][2] = 32'h40000000; // 2.0
    b_data[3][3] = 32'h3F800000; // 1.0

    // C矩阵：初始值设为0.5
    for (int i = 0; i < MAX_M; i++) begin
        for (int j = 0; j < MAX_N; j++) begin
            c_data[i][j] = 32'h3F000000; // 0.5
        end
    end

end


// 测试控制
initial begin
    // 初始化信号
    rst_n = 0;
    precision_mode = 2'b11;  // FP32模式
    m = 4;
    n = 4;
    k = 4;
    state = IDLE;
    pe_enable = 0;
    pe_load_c_en = 0;
    
    // 复位系统
    #20;
    rst_n = 1;
    #20;
    
    // 测试序列
    test_sequence();
    
    // 结束仿真
    #200;
    $display("Simulation completed successfully!");
    $finish;
end

// 主测试序列
task test_sequence;
    integer i, j, idx;
    begin
        // COMPUTE阶段 - 需要运行k个周期
        $display("Starting COMPUTE phase...");
        state = COMPUTE;
        pe_load_c_en = 1;
        @(posedge clk);

        pe_load_c_en = 0;
        @(posedge clk);
        pe_enable = 1;

        // 等待k+m+n-1个计算周期完成
        repeat(k+m+n-1) @(posedge clk);
        
        pe_enable = 0;
        @(posedge clk);
        // 等待1个周期以确保计算完成
        @(posedge clk);
        
        // OUTPUT阶段 - 需要运行m*n个周期来输出所有结果
        state = OUTPUT;
        
        // 监控输出结果
        for (i = 0; i < m * n; i++) begin
            @(posedge clk);
            $display("Output[%0d]: d_out = %h (cycle_counter = %0d)", 
                     i, d_out, cycle_counter);
        end
        
        // 返回IDLE状态
        state = IDLE;
        @(posedge clk);
    end
endtask

// // 关键：补充计数器更新逻辑
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cycle_counter <= 0;
        compute_counter <= 0;
        pe_enable <= 0;
    end else begin
        case (state)
                COMPUTE: begin
                    integer propagation_delay;
                    propagation_delay = m + n;
                    pe_enable <= 1; // 在计算期间打开

                    if (cycle_counter == 0) begin
                        compute_counter <= 0;
                        pe_load_c_en    <= 1;
                        pe_enable <= 0; // 可能确保在计算开始时关闭
                    end else begin
                        pe_load_c_en <= 0; // 加载完成后可以关闭
                    end

                    if (compute_counter <= k) begin // 这里保持compute_counter递增
                        compute_counter <= compute_counter + 1;
                    end

                    // 退出条件：总计算周期 = k（乘累加） + m + n（传播）
                    // （-1 是因为从0开始计数）
                    if (cycle_counter >= (k + propagation_delay )) begin//！！逻辑发群里了，详细跳转逻辑的讲解
                        state <= OUTPUT; 
                        cycle_counter <= 0;
                        // 结束计算，将pe_enable置为0
                        pe_enable <= 0;
                    end else begin
                        cycle_counter <= cycle_counter + 1;
                    end
                end

                OUTPUT: begin
                    if (cycle_counter < (m * n)) begin
                        cycle_counter <=  cycle_counter + 1;  // 计数器递增（beat_cnt控制）;tb自增
                    end else begin
                        state <= IDLE;
                    end
                    
                end

                default: begin
                    state <= IDLE;
                    pe_enable <= 0; // 在默认状态下也确保关闭PE
                end


        endcase

    end
end

// 所有PE的监控信号定义
wire [31:0] monitor_pe_a_in [1:MAX_M][1:MAX_N];
wire [31:0] monitor_pe_b_in [1:MAX_M][1:MAX_N];
wire [31:0] monitor_pe_c_in [1:MAX_M][1:MAX_N];
wire [31:0] monitor_pe_a_out [1:MAX_M][1:MAX_N];
wire [31:0] monitor_pe_b_out [1:MAX_M][1:MAX_N];
wire [31:0] monitor_pe_d_out [1:MAX_M][1:MAX_N];
wire        monitor_pe_overflow [1:MAX_M][1:MAX_N];
wire [31:0] monitor_pe_fp_acc [1:MAX_M][1:MAX_N];

// 连接所有PE的监控信号
generate
    genvar i, j;
    for (i = 1; i <= MAX_M; i = i + 1) begin : pe_row_monitor
        for (j = 1; j <= MAX_N; j = j + 1) begin : pe_col_monitor
            assign monitor_pe_a_in[i][j] = dut.a_bus[i][j-1];
            assign monitor_pe_b_in[i][j] = dut.b_bus[i-1][j];
            assign monitor_pe_c_in[i][j] = dut.c_data[i][j];
            assign monitor_pe_a_out[i][j] = (i <= MAX_M) ? dut.a_bus[i][j] : 32'b0;
            assign monitor_pe_b_out[i][j] = (j <= MAX_N) ? dut.b_bus[i][j] : 32'b0;
            assign monitor_pe_d_out[i][j] = dut.d_bus[i][j];
            assign monitor_pe_overflow[i][j] = dut.overflow_bus[i][j];
        end
    end
endgenerate

endmodule