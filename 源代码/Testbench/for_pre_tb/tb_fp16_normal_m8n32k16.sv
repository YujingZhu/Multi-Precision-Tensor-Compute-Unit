`timescale 1ns / 1ps

module tb_tpu_top();

localparam CLK_PERIOD = 10;          // 100MHz时钟
localparam APB_DATA_WIDTH = 9;
localparam C_S_AXI_ADDR_WIDTH = 32;
localparam C_S_AXI_DATA_WIDTH = 32;
localparam MAX_M = 32;
localparam MAX_N = 32;
localparam MAX_K = 16;
localparam PE_ROW_MAX = 8;//修改尺寸
localparam PE_COL_MAX = 8;//修改尺寸

reg clk = 0;
reg setn = 0;

reg s_axi_awvalid = 0;
wire s_axi_awready;
reg [C_S_AXI_ADDR_WIDTH-1:0] s_axi_awaddr = 0;
reg s_axi_wvalid = 0;
wire s_axi_wready;
reg [C_S_AXI_DATA_WIDTH-1:0] s_axi_wdata = 0;
wire s_axi_bvalid;
reg s_axi_bready = 0;
wire [1:0] s_axi_bresp;

wire [C_S_AXI_ADDR_WIDTH-1:0] m_axi_awaddr;
wire m_axi_awvalid;
reg m_axi_awready = 0;
wire [C_S_AXI_DATA_WIDTH-1:0] m_axi_wdata;
wire m_axi_wvalid;
reg m_axi_wready = 0;
reg [1:0] m_axi_bresp = 0;
reg m_axi_bvalid = 0;
wire m_axi_bready;
wire write_done;

reg apb_psel = 0;
reg apb_penable = 0;
reg apb_pwrite = 0;
reg [APB_DATA_WIDTH-1:0] apb_pwdata = 0;

wire [31:0] d_out;
wire        done;
wire [11:0] cycle_counter;
wire        load_c_en;
wire        load_a_en;
wire        load_b_en;       
wire        out_en;
wire [3:0]  current_state;      
wire        pe_load_c_en;      
wire [6:0]  compute_counter; 
wire [2:0]  pe_counter;       //新增PE计数器   
wire        pe_enable;          
wire [31:0] s_addr;            
wire [11:0] beat_cnt;           

integer file_handle;          // 文件句柄
integer data_count = 0;       // 已读取数据计数
integer i;                    // 循环变量
integer char;                 // 字符读取变量
reg [31:0] current_data;      // 当前构建的数据
integer bit_count = 0;        // 当前数据位数
reg [31:0] test_data [0:511];  // 存储从文件读取的数据
always #(CLK_PERIOD/2) clk = ~clk;

TOP_TPU #(
    .C_S_AXI_ADDR_WIDTH(C_S_AXI_ADDR_WIDTH),
    .C_S_AXI_DATA_WIDTH(C_S_AXI_DATA_WIDTH),
    .C_M_AXI_DATA_WIDTH(C_S_AXI_DATA_WIDTH),
    .C_M_AXI_ADDR_WIDTH(C_S_AXI_ADDR_WIDTH),
    .APB_DATA_WIDTH(APB_DATA_WIDTH),
    .MAX_M(MAX_M),
    .MAX_N(MAX_N),
    .MAX_K(MAX_K),
    .PE_ROW_MAX(PE_ROW_MAX),
    .PE_COL_MAX(PE_COL_MAX)
) dut (
    // AXI slave 接口
    .clk(clk),
    .setn(setn),
    .s_axi_awvalid(s_axi_awvalid),
    .s_axi_awready(s_axi_awready),
    .s_axi_awaddr(s_axi_awaddr),
    .s_axi_wvalid(s_axi_wvalid),
    .s_axi_wready(s_axi_wready),
    .s_axi_wdata(s_axi_wdata),
    .s_axi_bvalid(s_axi_bvalid),
    .s_axi_bready(s_axi_bready),
    .s_axi_bresp(s_axi_bresp),

    // AXI master 接口
    .m_axi_awaddr(m_axi_awaddr),
    .m_axi_awvalid(m_axi_awvalid),
    .m_axi_awready(m_axi_awready),
    .m_axi_wdata(m_axi_wdata),
    .m_axi_wvalid(m_axi_wvalid),
    .m_axi_wready(m_axi_wready),
    .m_axi_bresp(m_axi_bresp),
    .m_axi_bvalid(m_axi_bvalid),
    .m_axi_bready(m_axi_bready),
    .write_done(write_done),
    
    // APB接口
    .apb_psel(apb_psel),
    .apb_penable(apb_penable),
    .apb_pwrite(apb_pwrite),
    .apb_pwdata(apb_pwdata),
    
    // 测试接口
    .cycle_counter(cycle_counter),
    .load_c_en(load_c_en),
    .load_a_en(load_a_en),
    .load_b_en(load_b_en),
    .current_state(current_state),
    .pe_load_c_en(pe_load_c_en),
    .compute_counter(compute_counter),
    .out_en(out_en),
    .pe_enable(pe_enable),
    .pe_counter(pe_counter),
    .done(done),
    
    .s_addr(s_addr),
    .beat_cnt(beat_cnt),
    
    .d_out(d_out)
    
);

// 初始化
task initialize;
begin
    // 初始化所有输入信号
    s_axi_awvalid = 0;
    s_axi_awaddr = 0;
    s_axi_wvalid = 0;
    s_axi_wdata = 0;
    s_axi_bready = 0;
    
    m_axi_awready = 0;
    m_axi_wready = 0;
    m_axi_bresp = 0;
    m_axi_bvalid = 0;
    
    apb_psel = 0;
    apb_penable = 0;
    apb_pwrite = 0;
    apb_pwdata = 0;       
    
    // 复位系统
    setn = 0;
    #100;
    setn = 1;
    #100;
    
    $display("[INITIALIZATION] @%0t: System initialized and reset released", $time);

end
endtask

// slave 写入
task axi_slave_write;
    input [31:0] addr;
    input [31:0] data;
    input integer delay;
    input [1:0] mem_sel;
begin
    // 阶段1：地址传输
    s_axi_awaddr = addr;
    s_axi_awvalid = 1;
    while(!s_axi_awready) @(posedge clk);
    @(posedge clk) #1 s_axi_awvalid = 0;

    // 阶段2：数据传输
    s_axi_wdata = data;
    s_axi_wvalid = 1;
    while(!s_axi_wready) @(posedge clk);
    @(posedge clk) #1 s_axi_wvalid = 0;

    // 阶段3：响应接收
    s_axi_bready = 1;
    while(!s_axi_bvalid) @(posedge clk);
    @(posedge clk) #20 s_axi_bready = 0;

    // 显示写入信息
    case(mem_sel)
        2'b00: $display("[AXI-SLAVE-WR-A] @%0t: Addr=0x%h Data=0x%h", 
                       $time, addr, data);
        2'b01: $display("[AXI-SLAVE-WR-B] @%0t: Addr=0x%h Data=0x%h", 
                       $time, addr, data);
        2'b10: $display("[AXI-SLAVE-WR-C] @%0t: Addr=0x%h Data=0x%h", 
                       $time, addr, data);
    endcase

    // 自定义延迟
    repeat(delay) @(posedge clk);
end
endtask

// master写响应
task axi_master_response;
    input integer m, n;
    integer row, col;
begin
    wait(dut.u_fsm_controller.current_state == 4'd5); // 等待OUTPUT状态
    
    $display("\n=== AXI Master响应模拟启动 ===");
    
    for (row = 0; row < m; row = row + 1) begin
        for (col = 0; col < n; col = col + 1) begin
            // 阶段1：地址通道握手
            while (!m_axi_awvalid) @(posedge clk); 
            @(posedge clk) #1 m_axi_awready = 1;    
            @(posedge clk) #1 m_axi_awready = 0;    

            // 阶段2：数据通道握手
            while (!m_axi_wvalid) @(posedge clk);   
            @(posedge clk) #1 m_axi_wready = 1;     
            @(posedge clk) #1 m_axi_wready = 0;     

            // 显示写入数据
            $display("[AXI-MASTER-WR] @%0t: Addr=0x%h Data=0x%h", 
                    $time, m_axi_awaddr, m_axi_wdata);

            // 阶段3：响应通道
            @(posedge clk);
            m_axi_bvalid = 1;
            m_axi_bresp = 2'b00;       
            @(posedge clk) #1 m_axi_bvalid = 0;

            // 传输间隔
            repeat(2) @(posedge clk);
        end
    end
    $display("=== AXI Master传输完成 ===");
end
endtask


task axi_slave_write_from_file;
    input [31:0] base_addr;
    input integer num_transfers;
    input integer delay;
    input [1:0] mem_sel;
    input string file_path;
begin
    // 打开文件
    file_handle = $fopen(file_path, "r");
    if (file_handle == 0) begin
        $display("Error: Could not open file %s", file_path);
        $finish;
    end
    
    // 初始化当前数据
    current_data = 16'b0;  // 现在只需要16位存储
    bit_count = 0;
    data_count = 0;
    
    // 读取文件内容
    while (!$feof(file_handle)) begin
        char = $fgetc(file_handle);
        
        // 忽略空格、换行、制表符等空白字符
        if (char == " " || char == "\n" || char == "\t" || char == "\r") begin
            continue;
        end
        
        // 只处理0和1字符
        if (char == "0" || char == "1") begin
            // 将字符转换为bit并添加到当前数据的低16位
            current_data = {current_data[14:0], (char == "1") ? 1'b1 : 1'b0};
            bit_count = bit_count + 1;
            
            // 当收集到16位时，存储数据并重置
            if (bit_count == 16) begin
                // 高16位补0，形成32位数据
                test_data[data_count] = {16'b0, current_data};
                data_count = data_count + 1;
                current_data = 16'b0;
                bit_count = 0;
                
                // 检查是否达到最大传输次数
                if (data_count >= num_transfers) begin
                    break;
                end
            end
        end
    end
    
    // 处理最后不足16位的数据
    if (bit_count > 0 && bit_count < 16) begin
        $display("Warning: Last data has only %0d bits, padding with zeros", bit_count);
        current_data = current_data << (16 - bit_count);  // 左对齐补0
        test_data[data_count] = {16'b0, current_data};   // 高16位补0
        data_count = data_count + 1;
    end
    
    $fclose(file_handle);
    $display("Read %0d 32-bit data words (16-bit actual) from file %s", data_count, file_path);
    
    // 执行AXI写操作
    for (i = 0; i < num_transfers && i < data_count; i = i + 1) begin
        axi_slave_write(
            base_addr + i,  // 地址递增
            test_data[i],    // 数据（高16位为0，低16位有效）
            delay,           // 延迟
            mem_sel          // 存储器选择
        );
    end
end
endtask


// APB配置任务
task apb_config;
    input [1:0] mem_sel;
    input [1:0] matrix_mode;
    input mixed;
    input [2:0] precision;
    input start;
begin
    // apb_paddr = 0;
    apb_pwdata = {mem_sel, matrix_mode, mixed, precision, start};
    apb_psel = 1;
    apb_pwrite = 1;
    @(posedge clk);
    apb_penable = 1;
    @(posedge clk);
    apb_psel = 0;
    apb_penable = 0;
    
    $display("[APB-CFG] @%0t: mem_sel=%b matrix=%b mixed=%b prec=%b start=%b", 
            $time, mem_sel, matrix_mode, mixed, precision, start);
    repeat(5) @(posedge clk);
end
endtask


// 主测试流程
initial begin

    // 初始化系统
    $display("\n=== 系统初始化开始 ===");
    initialize();
    
    
    $display("=== 初始化完成 ===");


    // 测试1：从文件加载C矩阵数据
    $display("\n=== 测试1：从文件加载C矩阵数据 ===");
    apb_config(2'b10, 2'b10, 1'b0, 3'b010, 1'b1); 
    
    // 从文件读取数据并写入AXI总线
    axi_slave_write_from_file(
        32'h0000_0000,  // 基地址
        256,            // 传输次数
        2,              // 每次传输后的延迟
        2'b10,          // mem_sel选择C矩阵
        "D:/Desktop/fp16_m8n32k16/c_fp16_m8n32k16.mem.txt" //文件路径
    );
    
    // 等待加载完成
    wait(dut.u_fsm_controller.current_state == 4'd2); // 等待转为LOAD_A状态
    $display("C矩阵加载完成, 当前状态: LOAD_A");


    // 测试2：从文件加载A矩阵数据
    $display("\n=== 测试2：从文件加载A矩阵数据 ===");
    apb_config(2'b00, 2'b10, 1'b0, 3'b010, 1'b1);
            
    // 从文件读取数据并写入AXI总线
    axi_slave_write_from_file(
        32'h0000_0000,  // 基地址
        128,             // 传输次数
        2,              // 每次传输后的延迟
        2'b00,          // mem_sel选择A矩阵
        "D:/Desktop/fp16_m8n32k16/a_fp16_m8n32k16.mem.txt" // 文件路径
    );
    
    // 等待加载完成
    wait(dut.u_fsm_controller.current_state == 4'd3); // 等待转为LOAD_B状态
    $display("A矩阵加载完成, 当前状态: LOAD_B");
    

    // 测试3：从文件加载B矩阵数据
    $display("\n=== 测试3：从文件加载B矩阵数据 ===");
    apb_config(2'b01, 2'b10, 1'b0, 3'b010, 1'b1);
    
    // 从文件读取数据并写入AXI总线
    axi_slave_write_from_file(
        32'h0000_0000,  // 基地址
        512,             // 传输次数;
        2,              // 每次传输后的延迟
        2'b01,          // mem_sel选择B矩阵
        "D:/Desktop/fp16_m8n32k16/b_fp16_m8n32k16.mem.txt" // 文件路径
    );
    
    // 等待加载完成
    wait(dut.u_fsm_controller.current_state == 4'd4); // 等待转为COMPUTE状态
    $display("B矩阵加载完成, 当前状态: COMPUTE");
    
    
//==============================================================================
//第一分块输出
    wait(dut.u_fsm_controller.current_state == 4'd4); // 等待转为COMPUTE状态
    $display("当前状态: COMPUTE");

    $display("\n=== 启动AXI Master响应模拟 ===");
    fork
        axi_master_response(PE_ROW_MAX, PE_COL_MAX);//修改！！
    join_none

    wait(dut.u_fsm_controller.current_state == 4'd5); // 等待转为OUTPUT状态
    $display("计算完成, 当前状态: OUTPUT");

//==============================================================================
//第二分块输出
    wait(dut.u_fsm_controller.current_state == 4'd4); // 等待转为COMPUTE状态
    $display("当前状态: COMPUTE");
    
    $display("\n=== 启动AXI Master响应模拟 ===");
    fork
        axi_master_response(PE_ROW_MAX, PE_COL_MAX);//修改！！
    join_none
    
    wait(dut.u_fsm_controller.current_state == 4'd5); // 等待转为OUTPUT状态
    $display("计算完成, 当前状态: OUTPUT");

//==============================================================================
//第三分块输出
    wait(dut.u_fsm_controller.current_state == 4'd4); // 等待转为COMPUTE状态
    $display("当前状态: COMPUTE");
    
    $display("\n=== 启动AXI Master响应模拟 ===");
    fork
        axi_master_response(PE_ROW_MAX, PE_COL_MAX);//修改！！
    join_none
    
    wait(dut.u_fsm_controller.current_state == 4'd5); // 等待转为OUTPUT状态
    $display("计算完成, 当前状态: OUTPUT");

//==============================================================================
//第四分块输出
    wait(dut.u_fsm_controller.current_state == 4'd4); // 等待转为COMPUTE状态
    $display("当前状态: COMPUTE");
    
    $display("\n=== 启动AXI Master响应模拟 ===");
    fork
        axi_master_response(PE_ROW_MAX, PE_COL_MAX);//修改！！
    join_none
    
    wait(dut.u_fsm_controller.current_state == 4'd5); // 等待转为OUTPUT状态
    $display("计算完成, 当前状态: OUTPUT");
//==============================================================================

    // 等待master写完成
    wait(done);
    $display("AXI Master写完成");  

    // 结束测试
    #100;
    $display("\n=== 测试完成 ===");
    $finish;
end
                
endmodule