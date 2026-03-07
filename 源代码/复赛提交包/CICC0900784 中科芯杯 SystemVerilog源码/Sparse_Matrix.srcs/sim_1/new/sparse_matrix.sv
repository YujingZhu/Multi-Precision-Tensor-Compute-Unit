`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 朱妤婧 王欣彤
// 
// Create Date: 2025/07/15 19:11:38
// Design Name: TPU
// Module Name: TPU_Defense_Presentation
// Project Name: TPU_Defense_Presentation
// Target Devices: VCU118 XCVU9P-L2FLGA2104E
// Tool Versions: 2024.1 Vivado
// Description: 8×8脉动阵列 5级流水线矩阵乘法张量计算单元加速器
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

(*use_dsp ="no" *) module TOP_TPU_Sparse_Matrix #(
  parameter C_S_AXI_ADDR_WIDTH = 32,
  parameter C_S_AXI_DATA_WIDTH = 32,
  parameter C_M_AXI_DATA_WIDTH = 64,
  parameter C_M_AXI_ADDR_WIDTH = 32,
  parameter APB_DATA_WIDTH = 9,

  parameter MAX_K = 8,
  parameter C_ROW_MAX = 16,
  parameter C_COL_MAX = 16,
  parameter PE_ROW_MAX = 8,
  parameter PE_COL_MAX = 8,

  parameter IDLE     = 4'd0,
  parameter LOAD_C   = 4'd1,
  parameter LOAD_A   = 4'd2,
  parameter LOAD_B   = 4'd3,
  parameter COMPUTE  = 4'd4,
  parameter OUTPUT   = 4'd5
)(
  // AXI slave 接口
  input wire                          clk,
  input wire                          setn,
  input wire                          s_axi_awvalid,
  output wire                         s_axi_awready,
  input wire [C_S_AXI_ADDR_WIDTH-1:0] s_axi_awaddr,
  input wire                          s_axi_wvalid,
  output wire                         s_axi_wready,
  input wire [C_S_AXI_DATA_WIDTH-1:0] s_axi_wdata,
  output wire                         s_axi_bvalid,
  input wire                          s_axi_bready,
  output wire [1:0]                   s_axi_bresp,

  // AXI master 接口 (用于写回结果)
  output wire [C_M_AXI_ADDR_WIDTH-1:0] m_axi_awaddr,
  output wire                          m_axi_awvalid,
  input  wire                          m_axi_awready,
  output wire [C_M_AXI_DATA_WIDTH-1:0] m_axi_wdata,
  output wire                          m_axi_wvalid,
  input  wire                          m_axi_wready,
  input  wire [1:0]                    m_axi_bresp,
  input  wire                          m_axi_bvalid,
  output wire                          m_axi_bready,  
  output wire                          write_done,
  
  // APB接口
  input wire                          apb_psel,
  input wire                          apb_penable,
  input wire                          apb_pwrite,
  input wire [APB_DATA_WIDTH-1:0]     apb_pwdata

//   //以下为测试监控信号
//   // master 监控
//   output wire  [63:0]  d_out,
  
//   // fsm 监控
//   output [11:0] cycle_counter,
//   output        load_c_en,
//   output        load_a_en,
//   output        load_b_en,
//   output        out_en,
//   output        done,
//   output [3:0]  current_state,
//   output        pe_enable,
//   output        pe_load_c_en,
//   output reg [6:0]  compute_counter,//信号位宽增一位
//   output reg [2:0]  pe_counter,//新增PE计数器
//   output [31:0] s_addr,
//   output [11:0] beat_cnt//修改位数
);
 
  // slave 传输信号
  wire [31:0]                  a_row;
  wire [31:0]                  b_col;
  wire [31:0]                  c_element;
  
  // APB配置信号
  wire [2:0]  precision_mode;  // 精度模式
  wire        mixed_mode;      // 混合精度模式
  wire [5:0]  m, n, k;        // 矩阵维度
  wire [1:0]  matrix_mode;     // 矩阵模式
  wire        start;           // 启动信号
  wire [1:0]  mem_sel;         // 存储器选择

  // 数据加载信号
  wire        overflow;
  wire [3:0]  elements_per_word;

// FSM控制信号
  wire [11:0]  cycle_counter;
  wire         load_c_en;
  wire         load_a_en;
  wire         load_b_en;
  wire         out_en;
  wire         done;
  wire [3:0]   current_state;   // 当前状态
  wire         pe_enable;      // PE使能
  wire [2:0]   pe_counter;
  wire         pe_load_c_en;   // PE加载C使能
  wire [6:0]   compute_counter; // 计算计数器
  wire [31:0]  s_addr;           // 地址总线
  wire [11:0]  beat_cnt;                            
  
  // master 信号
  wire  [63:0]  d_out;           

(* ram_style = "block" *) reg [36:0] a_data00 [0:MAX_K-1];  
(* ram_style = "block" *) reg [36:0] a_data01 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data02 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data03 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data04 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data05 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data06 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data07 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data08 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data09 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data10 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data11 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data12 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data13 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data14 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data15 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data16 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data17 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data18 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data19 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data20 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data21 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data22 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data23 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data24 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data25 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data26 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data27 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data28 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data29 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data30 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] a_data31 [0:MAX_K-1];

(* ram_style = "block" *) reg [36:0] b_data00 [0:MAX_K-1];  
(* ram_style = "block" *) reg [36:0] b_data01 [0:MAX_K-1];  
(* ram_style = "block" *) reg [36:0] b_data02 [0:MAX_K-1];  
(* ram_style = "block" *) reg [36:0] b_data03 [0:MAX_K-1];  
(* ram_style = "block" *) reg [36:0] b_data04 [0:MAX_K-1];  
(* ram_style = "block" *) reg [36:0] b_data05 [0:MAX_K-1]; 
(* ram_style = "block" *) reg [36:0] b_data06 [0:MAX_K-1]; 
(* ram_style = "block" *) reg [36:0] b_data07 [0:MAX_K-1];  
(* ram_style = "block" *) reg [36:0] b_data08 [0:MAX_K-1];  
(* ram_style = "block" *) reg [36:0] b_data09 [0:MAX_K-1]; 
(* ram_style = "block" *) reg [36:0] b_data10 [0:MAX_K-1]; 
(* ram_style = "block" *) reg [36:0] b_data11 [0:MAX_K-1];  
(* ram_style = "block" *) reg [36:0] b_data12 [0:MAX_K-1];  
(* ram_style = "block" *) reg [36:0] b_data13 [0:MAX_K-1];  
(* ram_style = "block" *) reg [36:0] b_data14 [0:MAX_K-1];  
(* ram_style = "block" *) reg [36:0] b_data15 [0:MAX_K-1];  
(* ram_style = "block" *) reg [36:0] b_data16 [0:MAX_K-1]; 
(* ram_style = "block" *) reg [36:0] b_data17 [0:MAX_K-1];  
(* ram_style = "block" *) reg [36:0] b_data18 [0:MAX_K-1]; 
(* ram_style = "block" *) reg [36:0] b_data19 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] b_data20 [0:MAX_K-1]; 
(* ram_style = "block" *) reg [36:0] b_data21 [0:MAX_K-1];  
(* ram_style = "block" *) reg [36:0] b_data22 [0:MAX_K-1];  
(* ram_style = "block" *) reg [36:0] b_data23 [0:MAX_K-1];
(* ram_style = "block" *) reg [36:0] b_data24 [0:MAX_K-1]; 
(* ram_style = "block" *) reg [36:0] b_data25 [0:MAX_K-1]; 
(* ram_style = "block" *) reg [36:0] b_data26 [0:MAX_K-1];  
(* ram_style = "block" *) reg [36:0] b_data27 [0:MAX_K-1]; 
(* ram_style = "block" *) reg [36:0] b_data28 [0:MAX_K-1];  
(* ram_style = "block" *) reg [36:0] b_data29 [0:MAX_K-1]; 
(* ram_style = "block" *) reg [36:0] b_data30 [0:MAX_K-1]; 
(* ram_style = "block" *) reg [36:0] b_data31 [0:MAX_K-1]; 

reg [31:0] c_data [0:C_ROW_MAX-1][0:C_COL_MAX-1];

// APB控制器实例化
 apb_ctrl #(    
    .APB_DATA_WIDTH(APB_DATA_WIDTH)      
) u_apb_ctrl (
    // APB接口信号
    .apb_aclk     (clk),     
    .apb_aresetn  (setn),   
    .apb_psel     (apb_psel),      
    .apb_penable  (apb_penable),   
    .apb_pwrite   (apb_pwrite),   
    .apb_pwdata   (apb_pwdata),    
    
    // TPU控制信号
    .precision_mode (precision_mode),  
    .mixed_mode     (mixed_mode),    
    .m              (m),             
    .n              (n),              
    .k              (k),              
    .matrix_mode    (matrix_mode),    
    .start          (start),          
    .mem_sel        (mem_sel)         
);
  // AXI从控制器实例化
  axi_slave_ctrl #(
    .C_S_AXI_ADDR_WIDTH(C_S_AXI_ADDR_WIDTH),
    .C_S_AXI_DATA_WIDTH(C_S_AXI_DATA_WIDTH)
  ) u_axi_slave_ctrl (
    .s_axi_aclk(clk),
    .s_axi_aresetn(setn),
    .s_axi_awvalid(s_axi_awvalid),
    .s_axi_awready(s_axi_awready),
    .s_axi_awaddr(s_axi_awaddr),
    .s_axi_wvalid(s_axi_wvalid),
    .s_axi_wready(s_axi_wready),
    .s_axi_wdata(s_axi_wdata),
    .s_axi_bvalid(s_axi_bvalid),
    .s_axi_bready(s_axi_bready),
    .s_axi_bresp(s_axi_bresp),
    .mem_sel(mem_sel),
    .a_row(a_row),
    .b_col(b_col),
    .c_element(c_element),
    .s_addr(s_addr)
  );

  // 数据加载模块实例化
  data_load #(
    .MAX_K(MAX_K),
    .C_ROW_MAX(C_ROW_MAX),
    .C_COL_MAX(C_COL_MAX)
  ) u_data_load (
    .clk(clk),
    .rst_n(setn),
    .precision_mode(precision_mode),
    .s_axi_bready(s_axi_bready),
    .a_row(a_row),
    .b_col(b_col),
    .c_element(c_element),
    .elements_per_word(elements_per_word),
    .m(m),
    .n(n),
    .k(k),
    .matrix_mode(matrix_mode),//新增信号索引！
    .cycle_counter(cycle_counter),
    .load_c_en(load_c_en),
    .load_a_en(load_a_en),
    .load_b_en(load_b_en),
    // a_data 部分 (0-31)
    .a_data00(a_data00),
    .a_data01(a_data01),
    .a_data02(a_data02),
    .a_data03(a_data03),
    .a_data04(a_data04),
    .a_data05(a_data05),
    .a_data06(a_data06),
    .a_data07(a_data07),
    .a_data08(a_data08),
    .a_data09(a_data09),
    .a_data10(a_data10),
    .a_data11(a_data11),
    .a_data12(a_data12),
    .a_data13(a_data13),
    .a_data14(a_data14),
    .a_data15(a_data15),
    .a_data16(a_data16),
    .a_data17(a_data17),
    .a_data18(a_data18),
    .a_data19(a_data19),
    .a_data20(a_data20),
    .a_data21(a_data21),
    .a_data22(a_data22),
    .a_data23(a_data23),
    .a_data24(a_data24),
    .a_data25(a_data25),
    .a_data26(a_data26),
    .a_data27(a_data27),
    .a_data28(a_data28),
    .a_data29(a_data29),
    .a_data30(a_data30),
    .a_data31(a_data31),

    // b_data 部分 (0-31)
    .b_data00(b_data00),
    .b_data01(b_data01),
    .b_data02(b_data02),
    .b_data03(b_data03),
    .b_data04(b_data04),
    .b_data05(b_data05),
    .b_data06(b_data06),
    .b_data07(b_data07),
    .b_data08(b_data08),
    .b_data09(b_data09),
    .b_data10(b_data10),
    .b_data11(b_data11),
    .b_data12(b_data12),
    .b_data13(b_data13),
    .b_data14(b_data14),
    .b_data15(b_data15),
    .b_data16(b_data16),
    .b_data17(b_data17),
    .b_data18(b_data18),
    .b_data19(b_data19),
    .b_data20(b_data20),
    .b_data21(b_data21),
    .b_data22(b_data22),
    .b_data23(b_data23),
    .b_data24(b_data24),
    .b_data25(b_data25),
    .b_data26(b_data26),
    .b_data27(b_data27),
    .b_data28(b_data28),
    .b_data29(b_data29),
    .b_data30(b_data30),
    .b_data31(b_data31),
    .c_data(c_data)
  );

    // 状态机控制器实例化
    FSM_Controller #(
        .PE_ROW_MAX(PE_ROW_MAX),
        .PE_COL_MAX(PE_COL_MAX)
    ) u_fsm_controller (
        .clk(clk),
        .rst_n(setn),
        .m(m),
        .n(n),
        .k(k),
        .start(start),
        .s_addr(s_addr),
        .beat_cnt(beat_cnt),
        .current_state(current_state),
        .cycle_counter(cycle_counter),
        .pe_counter(pe_counter),
        .done(done),
        .pe_enable(pe_enable),
        .load_c_en(load_c_en),
        .load_a_en(load_a_en),
        .load_b_en(load_b_en),
        .pe_load_c_en(pe_load_c_en),
        .compute_counter(compute_counter),
        .out_en(out_en)
    );

    tensor_compute_unit #(
        .MAX_K(MAX_K),
        .C_ROW_MAX(C_ROW_MAX),
        .C_COL_MAX(C_COL_MAX),
        .PE_ROW_MAX(PE_ROW_MAX),
        .PE_COL_MAX(PE_COL_MAX),
        .IDLE(IDLE),
        .LOAD_C(LOAD_C),
        .LOAD_A(LOAD_A),
        .LOAD_B(LOAD_B),
        .COMPUTE(COMPUTE),
        .OUTPUT(OUTPUT)
    ) u_tensor_compute_unit (
        .clk(clk),
        .rst_n(setn),
        .precision_mode(precision_mode), 
        .overflow(overflow),
        .d_out(d_out),
        .mixed_mode     (mixed_mode), 
        .matrix_mode(matrix_mode),//新增信号索引！
        .state(current_state),
        .cycle_counter(cycle_counter),
        .compute_counter(compute_counter),
        .pe_counter(pe_counter),
        .pe_enable(pe_enable),
        .pe_load_c_en(pe_load_c_en),
        .a_data00(a_data00),
        .a_data01(a_data01),
        .a_data02(a_data02),
        .a_data03(a_data03),
        .a_data04(a_data04),
        .a_data05(a_data05),
        .a_data06(a_data06),
        .a_data07(a_data07),
        .a_data08(a_data08),
        .a_data09(a_data09),
        .a_data10(a_data10),
        .a_data11(a_data11),
        .a_data12(a_data12),
        .a_data13(a_data13),
        .a_data14(a_data14),
        .a_data15(a_data15),
        .a_data16(a_data16),
        .a_data17(a_data17),
        .a_data18(a_data18),
        .a_data19(a_data19),
        .a_data20(a_data20),
        .a_data21(a_data21),
        .a_data22(a_data22),
        .a_data23(a_data23),
        .a_data24(a_data24),
        .a_data25(a_data25),
        .a_data26(a_data26),
        .a_data27(a_data27),
        .a_data28(a_data28),
        .a_data29(a_data29),
        .a_data30(a_data30),
        .a_data31(a_data31),
        .b_data00(b_data00),
        .b_data01(b_data01),
        .b_data02(b_data02),
        .b_data03(b_data03),
        .b_data04(b_data04),
        .b_data05(b_data05),
        .b_data06(b_data06),
        .b_data07(b_data07),
        .b_data08(b_data08),
        .b_data09(b_data09),
        .b_data10(b_data10),
        .b_data11(b_data11),
        .b_data12(b_data12),
        .b_data13(b_data13),
        .b_data14(b_data14),
        .b_data15(b_data15),
        .b_data16(b_data16),
        .b_data17(b_data17),
        .b_data18(b_data18),
        .b_data19(b_data19),
        .b_data20(b_data20),
        .b_data21(b_data21),
        .b_data22(b_data22),
        .b_data23(b_data23),
        .b_data24(b_data24),
        .b_data25(b_data25),
        .b_data26(b_data26),
        .b_data27(b_data27),
        .b_data28(b_data28),
        .b_data29(b_data29),
        .b_data30(b_data30),
        .b_data31(b_data31),
        .c_data(c_data)
    );

    axi_master_ctrl #(
        .C_M_AXI_DATA_WIDTH(C_M_AXI_DATA_WIDTH),
        .C_M_AXI_ADDR_WIDTH(C_M_AXI_ADDR_WIDTH)
    )u_axi_master_ctrl(
        .m_axi_aclk(clk), // !!统一使用AXI时钟
        .m_axi_aresetn(setn), // !!统一复位

        // 写地址通道
        .m_axi_awaddr(m_axi_awaddr),
        .m_axi_awvalid(m_axi_awvalid),
        .m_axi_awready(m_axi_awready),

        // 写数据通道
        .m_axi_wdata(m_axi_wdata),
        .m_axi_wvalid(m_axi_wvalid),
        .m_axi_wready(m_axi_wready),

        // 写响应通道
        .m_axi_bresp(m_axi_bresp),
        .m_axi_bvalid(m_axi_bvalid),
        .m_axi_bready(m_axi_bready),

        // 控制接口
        .done(done),
        .write_done(write_done),
        .beat_cnt(beat_cnt),
        .out_en(out_en),
        .d_out(d_out)
    );

endmodule

(*use_dsp ="no" *)module axi_slave_ctrl #(
  parameter C_S_AXI_ADDR_WIDTH  = 32,
  parameter C_S_AXI_DATA_WIDTH  = 32
)(
    input wire                          s_axi_aclk,
    input wire                          s_axi_aresetn,

    input  wire                         s_axi_awvalid,
    output wire                         s_axi_awready,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]s_axi_awaddr,

    input  wire                         s_axi_wvalid,
    output wire                         s_axi_wready,
    input  wire [C_S_AXI_DATA_WIDTH-1:0]s_axi_wdata,

    output wire                         s_axi_bvalid,
    input  wire                         s_axi_bready,
    output reg [1:0]                    s_axi_bresp,
    
    input  wire [1:0]                   mem_sel,
    output reg [31:0]                   a_row,
    output reg [31:0]                   b_col,
    output reg [31:0]                   c_element,
    output reg [31:0]                   s_addr
);
    
localparam WR_IDLE  = 2'b00;
localparam WR_ADDR  = 2'b01;
localparam WR_WRITE = 2'b10;   
    
reg [1:0] wr_state;

always @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
    if (!s_axi_aresetn) begin
        wr_state     <= WR_IDLE;
        a_row        <= 0;
        b_col        <= 0;
        c_element    <= 0;
        s_addr         <= 0;
        s_axi_bresp  <= 2'b00;
    end else begin
        case(wr_state)
            WR_IDLE: begin
                s_addr <= 0;
                if (s_axi_awvalid) begin
                    wr_state <= WR_ADDR;
                end
            end
        
            WR_ADDR: begin
                if (s_axi_wvalid) begin
                    s_addr <= s_axi_awaddr;
                    wr_state <= WR_WRITE;
                end else if(!s_axi_wvalid)begin
                    s_addr <= 0;                    
                end
            end

            WR_WRITE: begin      
                case(mem_sel)
                    2'b00: a_row <= s_axi_wdata;
                    2'b01: b_col <= s_axi_wdata;
                    2'b10: c_element <= s_axi_wdata;
                    default: s_axi_bresp <= 2'b10; // 无效mem_sel
                endcase

                // 设置响应
                if (mem_sel inside {2'b00, 2'b01, 2'b10}) begin
                    s_axi_bresp <= 2'b00; // 正常响应
                end

                // 状态转移
                if (s_axi_bready) begin
                    if (wr_state == WR_ADDR)begin
                        wr_state <= WR_WRITE;
                    end else if (wr_state == WR_WRITE)begin
                        wr_state <= WR_IDLE;
                    end
                end
            end
            default:begin
                wr_state <= WR_IDLE;
            end
        endcase
    end
end

assign s_axi_awready = (wr_state == WR_IDLE);
assign s_axi_wready  = (wr_state == WR_ADDR);
assign s_axi_bvalid  = (wr_state == WR_WRITE);

endmodule

(*use_dsp ="no" *)module axi_master_ctrl #(
    parameter C_M_AXI_DATA_WIDTH = 64,
    parameter C_M_AXI_ADDR_WIDTH = 32
)(
    input  wire                          m_axi_aclk,
    input  wire                          m_axi_aresetn,

    // 写地址通道
    output reg [C_M_AXI_ADDR_WIDTH-1:0]  m_axi_awaddr,
    output wire                          m_axi_awvalid,
    input  wire                          m_axi_awready,

    // 写数据通道
    output reg [C_M_AXI_DATA_WIDTH-1:0]  m_axi_wdata,
    output wire                          m_axi_wvalid,
    input  wire                          m_axi_wready,

    // 写响应通道
    input  wire [1:0]                    m_axi_bresp,
    input  wire                          m_axi_bvalid,
    output wire                          m_axi_bready,

    // 控制接口
    // input  wire [5:0]                    m, n,
    input  wire                          done,
    output reg                           write_done,
    output reg [11:0]                    beat_cnt,
    input  wire                          out_en,
    input  wire [C_M_AXI_DATA_WIDTH-1:0] d_out
);

localparam WR_IDLE  = 2'b00;
localparam WR_ADDR  = 2'b01;
localparam WR_WRITE = 2'b10;   
    
reg [1:0]        state;

// 状态机时序控制
always @(posedge m_axi_aclk or negedge m_axi_aresetn) begin
    if (!m_axi_aresetn) begin
        state         <= WR_IDLE;
        beat_cnt      <= 0;
        write_done    <= 0;
        m_axi_awaddr  <= 0;
        m_axi_wdata   <= 0;
    end else begin       
        case(state)
            WR_IDLE:begin
                if (out_en) begin
                    state <= WR_ADDR;
                end else begin
                    beat_cnt <=0;
                end                                
            end
            WR_ADDR:begin
                if (m_axi_awready && out_en)begin
                    m_axi_awaddr <= m_axi_awaddr + 1; // 步进1
                    state <= WR_WRITE;
                end
            end
            WR_WRITE:begin
                if(m_axi_wready && out_en)begin
                    m_axi_wdata <= d_out;
                    beat_cnt <= beat_cnt + 1;
                    if (done) begin
                        state <= WR_IDLE;
                        write_done <= 1;
                    end else begin
                        state <= WR_ADDR;
                    end
                end
            end
            default:begin
                state <= WR_IDLE;
            end
        endcase
    end
end              

assign m_axi_awvalid = (state == WR_ADDR);
assign m_axi_wvalid  = (state == WR_WRITE);
assign m_axi_bready  = (state == WR_WRITE);

endmodule

(*use_dsp ="no" *)module apb_ctrl #(
  parameter APB_DATA_WIDTH = 9
)(
  input                          apb_aclk,
  input                          apb_aresetn,
  input                          apb_psel,
  input                          apb_penable,
  input                          apb_pwrite,
  input   [APB_DATA_WIDTH-1:0]   apb_pwdata,
  
  output reg     [2:0]           precision_mode,
  output reg                     mixed_mode,
  output reg     [5:0]           m,n,k,
  output reg     [1:0]           matrix_mode,
  output reg                     start,
  output reg     [1:0]           mem_sel
);

always @(posedge apb_aclk or negedge apb_aresetn) begin
    if (!apb_aresetn) begin
        {mem_sel, matrix_mode, mixed_mode, precision_mode} <= 8'b0;
        start <= 1'b0;
    end else if (apb_psel & apb_penable & apb_pwrite) begin
        start <= apb_pwdata[0];
        {mem_sel, matrix_mode, mixed_mode, precision_mode} <= apb_pwdata[8:1];
    end else begin
        start <= 1'b0; // 保持start为0，除非明确设置
    end
end

always @(*) begin
    case(matrix_mode)
        2'd0: {m, n, k} = {6'd16, 6'd16, 6'd16};
        2'd1: {m, n, k} = {6'd32, 6'd8, 6'd16};
        2'd2: {m, n, k} = {6'd8, 6'd32, 6'd16};
        default: {m, n, k} = {6'd16, 6'd16, 6'd16};
    endcase
end

endmodule
  
(*use_dsp ="no" *)module data_load#(
    parameter MAX_K = 8,
    parameter C_ROW_MAX = 16,
    parameter C_COL_MAX = 16
)(    
    input              clk,
    input              rst_n,
    // 配置接口
    input  [2:0]       precision_mode,
    // 数据接口
    input wire         s_axi_bready,
    input  [31:0]       a_row,
    input  [31:0]       b_col,
    input  [31:0]       c_element,
    output reg [3:0]   elements_per_word,
    // !!修改: 将 apb解码 m,n,k输入 不重复处理
    input wire [5:0]   m, n, k,  // 从配置模块获取
    input wire [1:0]   matrix_mode,
    // FSM控制信号
    input wire [11:0]  cycle_counter,
    input              load_c_en,    
    input              load_a_en,    
    input              load_b_en,      
    
    // 矩阵选择信号
    output reg [36:0] a_data00 [0:MAX_K-1],
    output reg [36:0] a_data01 [0:MAX_K-1],
    output reg [36:0] a_data02 [0:MAX_K-1],
    output reg [36:0] a_data03 [0:MAX_K-1],
    output reg [36:0] a_data04 [0:MAX_K-1],
    output reg [36:0] a_data05 [0:MAX_K-1],
    output reg [36:0] a_data06 [0:MAX_K-1],
    output reg [36:0] a_data07 [0:MAX_K-1],
    output reg [36:0] a_data08 [0:MAX_K-1],
    output reg [36:0] a_data09 [0:MAX_K-1],
    output reg [36:0] a_data10 [0:MAX_K-1],
    output reg [36:0] a_data11 [0:MAX_K-1],
    output reg [36:0] a_data12 [0:MAX_K-1],
    output reg [36:0] a_data13 [0:MAX_K-1],
    output reg [36:0] a_data14 [0:MAX_K-1],
    output reg [36:0] a_data15 [0:MAX_K-1],
    output reg [36:0] a_data16 [0:MAX_K-1],
    output reg [36:0] a_data17 [0:MAX_K-1],
    output reg [36:0] a_data18 [0:MAX_K-1],
    output reg [36:0] a_data19 [0:MAX_K-1],
    output reg [36:0] a_data20 [0:MAX_K-1],
    output reg [36:0] a_data21 [0:MAX_K-1],
    output reg [36:0] a_data22 [0:MAX_K-1],
    output reg [36:0] a_data23 [0:MAX_K-1],
    output reg [36:0] a_data24 [0:MAX_K-1],
    output reg [36:0] a_data25 [0:MAX_K-1],
    output reg [36:0] a_data26 [0:MAX_K-1],
    output reg [36:0] a_data27 [0:MAX_K-1],
    output reg [36:0] a_data28 [0:MAX_K-1],
    output reg [36:0] a_data29 [0:MAX_K-1],
    output reg [36:0] a_data30 [0:MAX_K-1],
    output reg [36:0] a_data31 [0:MAX_K-1],
    
    output reg [36:0] b_data00 [0:MAX_K-1],
    output reg [36:0] b_data01 [0:MAX_K-1],
    output reg [36:0] b_data02 [0:MAX_K-1],
    output reg [36:0] b_data03 [0:MAX_K-1],
    output reg [36:0] b_data04 [0:MAX_K-1],
    output reg [36:0] b_data05 [0:MAX_K-1],
    output reg [36:0] b_data06 [0:MAX_K-1],
    output reg [36:0] b_data07 [0:MAX_K-1],
    output reg [36:0] b_data08 [0:MAX_K-1],
    output reg [36:0] b_data09 [0:MAX_K-1],
    output reg [36:0] b_data10 [0:MAX_K-1],
    output reg [36:0] b_data11 [0:MAX_K-1],
    output reg [36:0] b_data12 [0:MAX_K-1],
    output reg [36:0] b_data13 [0:MAX_K-1],
    output reg [36:0] b_data14 [0:MAX_K-1],
    output reg [36:0] b_data15 [0:MAX_K-1],
    output reg [36:0] b_data16 [0:MAX_K-1],
    output reg [36:0] b_data17 [0:MAX_K-1],
    output reg [36:0] b_data18 [0:MAX_K-1],
    output reg [36:0] b_data19 [0:MAX_K-1],
    output reg [36:0] b_data20 [0:MAX_K-1],
    output reg [36:0] b_data21 [0:MAX_K-1],
    output reg [36:0] b_data22 [0:MAX_K-1],
    output reg [36:0] b_data23 [0:MAX_K-1],
    output reg [36:0] b_data24 [0:MAX_K-1],
    output reg [36:0] b_data25 [0:MAX_K-1],
    output reg [36:0] b_data26 [0:MAX_K-1],
    output reg [36:0] b_data27 [0:MAX_K-1],
    output reg [36:0] b_data28 [0:MAX_K-1],
    output reg [36:0] b_data29 [0:MAX_K-1],
    output reg [36:0] b_data30 [0:MAX_K-1],
    output reg [36:0] b_data31 [0:MAX_K-1],

    output reg [31:0] c_data [0:C_ROW_MAX-1][0:C_COL_MAX-1]

);   

    reg [3:0] ptr_a0;
    reg [3:0] ptr_a1;
    reg [3:0] ptr_a2;
    reg [3:0] ptr_a3;
    reg [3:0] ptr_a4;
    reg [3:0] ptr_a5;
    reg [3:0] ptr_a6;
    reg [3:0] ptr_a7;
    reg [3:0] ptr_a8;
    reg [3:0] ptr_a9;
    reg [3:0] ptr_a10;
    reg [3:0] ptr_a11;
    reg [3:0] ptr_a12;
    reg [3:0] ptr_a13;
    reg [3:0] ptr_a14;
    reg [3:0] ptr_a15;
    reg [3:0] ptr_a16;
    reg [3:0] ptr_a17;
    reg [3:0] ptr_a18;
    reg [3:0] ptr_a19;
    reg [3:0] ptr_a20;
    reg [3:0] ptr_a21;
    reg [3:0] ptr_a22;
    reg [3:0] ptr_a23;
    reg [3:0] ptr_a24;
    reg [3:0] ptr_a25;
    reg [3:0] ptr_a26;
    reg [3:0] ptr_a27;
    reg [3:0] ptr_a28;
    reg [3:0] ptr_a29;
    reg [3:0] ptr_a30;
    reg [3:0] ptr_a31;

    reg [3:0] ptr_b0;
    reg [3:0] ptr_b1;
    reg [3:0] ptr_b2;
    reg [3:0] ptr_b3;
    reg [3:0] ptr_b4;
    reg [3:0] ptr_b5;
    reg [3:0] ptr_b6;
    reg [3:0] ptr_b7;
    reg [3:0] ptr_b8;
    reg [3:0] ptr_b9;
    reg [3:0] ptr_b10;
    reg [3:0] ptr_b11;
    reg [3:0] ptr_b12;
    reg [3:0] ptr_b13;
    reg [3:0] ptr_b14;
    reg [3:0] ptr_b15;
    reg [3:0] ptr_b16;
    reg [3:0] ptr_b17;
    reg [3:0] ptr_b18;
    reg [3:0] ptr_b19;
    reg [3:0] ptr_b20;
    reg [3:0] ptr_b21;
    reg [3:0] ptr_b22;
    reg [3:0] ptr_b23;
    reg [3:0] ptr_b24;
    reg [3:0] ptr_b25;
    reg [3:0] ptr_b26;
    reg [3:0] ptr_b27;
    reg [3:0] ptr_b28;
    reg [3:0] ptr_b29;
    reg [3:0] ptr_b30;
    reg [3:0] ptr_b31;


// 保证索引不越界函数    
function automatic [31:0] safe_bit_select(
        input [31:0]     data_word,
        input [2:0]      precision
    );
        reg [15:0] fp16;
        reg        sign;
        reg [4:0]  exp5;
        reg [9:0]  frac10;
        reg [7:0]  exp8;
        reg [22:0] frac23;
        integer    leading_zeros;
        reg [23:0] denorm_shift;
        begin
            case(precision)
                // INT4模式
                3'b000: begin
                    return {{28{data_word[3]}}, data_word[3:0]};
                end
                // INT8模式
                3'b001: begin
                    return {{24{data_word[7]}}, data_word[7:0]};
                end
                // FP16模式（内联转换逻辑）
                3'b010: begin                   
                    fp16 = data_word[15:0];
                    sign = fp16[15];
                    exp5 = fp16[14:10];
                    frac10 = fp16[9:0];
                    
                    // FP16转FP32内联逻辑
                    if (exp5 == 5'b11111) begin // Inf/NaN
                        exp8 = 8'hFF;
                        frac23 = (frac10 == 0) ? 23'h0 : {1'b1, frac10, 12'h0};
                    end
                    else if (exp5 == 0) begin
                        if (frac10 == 0) begin // Zero
                            exp8 = 8'h00;
                            frac23 = 23'h0;
                        end else begin // Denormal
                            leading_zeros = 10;
                            for (integer i = 9; i >= 0; i = i - 1) begin
                                if (frac10[i]) begin
                                    leading_zeros = 9 - i;
                                    break;
                                end
                            end
                            denorm_shift = {1'b0, frac10, 13'h0} << (leading_zeros + 1);
                            frac23 = denorm_shift[23:1];
                            exp8 = 8'd127 - 8'd15 - leading_zeros;
                        end
                    end
                    else begin // Normal
                        exp8 = exp5 + 8'd112;
                        frac23 = {frac10, 13'h0};
                    end
                    return {sign, exp8, frac23};
                end
                // FP32模式
                3'b011: return data_word;
                // BF16 模式
                3'b100:begin
                    return {{16'b0}, data_word[15:0]};
                end
                // INT32模式
                3'b101:begin
                    return data_word;
                end
                default: return data_word;    
            endcase
        end
endfunction
    
    // 精度参数计算
    always @(*) begin
        case(precision_mode)

            3'b000: elements_per_word = 8;//4bits  int4
            3'b001: elements_per_word = 4;//8bits  int8
            3'b010: elements_per_word = 2;//16bits  fp16
            3'b011: elements_per_word = 1;//32bits  fp32

            3'b100: elements_per_word = 2;//16bits  bf16
            3'b101: elements_per_word = 1;//32bits  int32

            default: elements_per_word = 1;
        endcase
    end


    // C矩阵加载（带异步复位）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin  // 异步复位，低电平有效
            // 复位时将所有C矩阵元素清零
            for (int i = 0; i < C_ROW_MAX; i++) begin
                for (int j = 0; j < C_COL_MAX; j++) begin
                    c_data[i][j] <= 0;
                end
            end
        end
        else if (load_c_en && s_axi_bready && (cycle_counter < m*n)) begin  // 正常加载逻辑
                case(matrix_mode)
                2'b00: begin
                    automatic logic[4:0] row = cycle_counter / n;
                    automatic logic[4:0] col = cycle_counter % n;
                    c_data[row][col] <= safe_bit_select(c_element, precision_mode);
                end
                2'b01: begin
                    automatic logic[4:0] row = (cycle_counter / n > 15) ? cycle_counter / n - 16 : cycle_counter / n;
                    automatic logic[4:0] col = (cycle_counter / n > 15) ? cycle_counter % n + 8 : cycle_counter % n;
                    c_data[row][col] <= safe_bit_select(c_element, precision_mode);
                end
                2'b10: begin
                    automatic logic[4:0] row = (cycle_counter % n > 15) ? cycle_counter / n + 8 : cycle_counter / n;
                    automatic logic[4:0] col = (cycle_counter % n > 15) ? cycle_counter % n - 16 : cycle_counter % n;
                    c_data[row][col] <= safe_bit_select(c_element, precision_mode);
                end
                default: begin
                    automatic logic[4:0] row = cycle_counter / n;
                    automatic logic[4:0] col = cycle_counter % n;
                    c_data[row][col] <= safe_bit_select(c_element, precision_mode);
                end
                endcase
            end
        end
    
        // A矩阵加载
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin  // 异步复位，低电平有效
                ptr_a0 <= 0;
                ptr_a1 <= 0;
                ptr_a2 <= 0;
                ptr_a3 <= 0;
                ptr_a4 <= 0;
                ptr_a5 <= 0;
                ptr_a6 <= 0;
                ptr_a7 <= 0;
                ptr_a8 <= 0;
                ptr_a9 <= 0;
                ptr_a10 <= 0;
                ptr_a11 <= 0;
                ptr_a12 <= 0;
                ptr_a13 <= 0;
                ptr_a14 <= 0;
                ptr_a15 <= 0;
                ptr_a16 <= 0;
                ptr_a17 <= 0;
                ptr_a18 <= 0;
                ptr_a19 <= 0;
                ptr_a20 <= 0;
                ptr_a21 <= 0;
                ptr_a22 <= 0;
                ptr_a23 <= 0;
                ptr_a24 <= 0;
                ptr_a25 <= 0;
                ptr_a26 <= 0;
                ptr_a27 <= 0;
                ptr_a28 <= 0;
                ptr_a29 <= 0;
                ptr_a30 <= 0;
                ptr_a31 <= 0;
            // 复位时将所有寄存器清零
            for (int i = 0; i < MAX_K; i++) begin
                a_data00[i] <= 0;
                a_data01[i] <= 0;
                a_data02[i] <= 0;
                a_data03[i] <= 0;
                a_data04[i] <= 0;
                a_data05[i] <= 0;
                a_data06[i] <= 0;
                a_data07[i] <= 0;
                a_data08[i] <= 0;
                a_data09[i] <= 0;
                a_data10[i] <= 0;
                a_data11[i] <= 0;
                a_data12[i] <= 0;
                a_data13[i] <= 0;
                a_data14[i] <= 0;
                a_data15[i] <= 0;
                a_data16[i] <= 0;
                a_data17[i] <= 0;
                a_data18[i] <= 0;
                a_data19[i] <= 0;
                a_data20[i] <= 0;
                a_data21[i] <= 0;
                a_data22[i] <= 0;
                a_data23[i] <= 0;
                a_data24[i] <= 0;
                a_data25[i] <= 0;
                a_data26[i] <= 0;
                a_data27[i] <= 0;
                a_data28[i] <= 0;
                a_data29[i] <= 0;
                a_data30[i] <= 0;
                a_data31[i] <= 0;
            end
        end
        else if (load_a_en && s_axi_bready && (cycle_counter < m*k) && (a_row != 0)) begin
                    automatic logic[4:0] row = cycle_counter / k;
                    automatic logic[4:0] col = cycle_counter % k;
                            case(row)
                        5'b00000: begin 
                            a_data00[ptr_a0] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a0 = ptr_a0 + 1;
                        end
                        5'b00001: begin 
                            a_data01[ptr_a1] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a1 = ptr_a1 + 1;
                        end
                        5'b00010: begin 
                            a_data02[ptr_a2] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a2 = ptr_a2 + 1;
                        end
                        5'b00011: begin 
                            a_data03[ptr_a3] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a3 = ptr_a3 + 1;
                        end
                        5'b00100: begin 
                            a_data04[ptr_a4] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a4 = ptr_a4 + 1;
                        end
                        5'b00101: begin 
                            a_data05[ptr_a5] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a5 = ptr_a5 + 1;
                        end
                        5'b00110: begin 
                            a_data06[ptr_a6] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a6 = ptr_a6 + 1;
                        end
                        5'b00111: begin 
                            a_data07[ptr_a7] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a7 = ptr_a7 + 1;
                        end
                        5'b01000: begin 
                            a_data08[ptr_a8] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a8 = ptr_a8 + 1;
                        end
                        5'b01001: begin 
                            a_data09[ptr_a9] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a9 = ptr_a9 + 1;
                        end
                        5'b01010: begin 
                            a_data10[ptr_a10] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a10 = ptr_a10 + 1;
                        end
                        5'b01011: begin 
                            a_data11[ptr_a11] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a11 = ptr_a11 + 1;
                        end
                        5'b01100: begin 
                            a_data12[ptr_a12] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a12 = ptr_a12 + 1;
                        end
                        5'b01101: begin 
                            a_data13[ptr_a13] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a13 = ptr_a13 + 1;
                        end
                        5'b01110: begin 
                            a_data14[ptr_a14] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a14 = ptr_a14 + 1;
                        end
                        5'b01111: begin 
                            a_data15[ptr_a15] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a15 = ptr_a15 + 1;
                        end
                        5'b10000: begin 
                            a_data16[ptr_a16] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a16 = ptr_a16 + 1;
                        end
                        5'b10001: begin 
                            a_data17[ptr_a17] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a17 = ptr_a17 + 1;
                        end
                        5'b10010: begin 
                            a_data18[ptr_a18] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a18 = ptr_a18 + 1;
                        end
                        5'b10011: begin 
                            a_data19[ptr_a19] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a19 = ptr_a19 + 1;
                        end
                        5'b10100: begin 
                            a_data20[ptr_a20] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a20 = ptr_a20 + 1;
                        end
                        5'b10101: begin 
                            a_data21[ptr_a21] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a21 = ptr_a21 + 1;
                        end
                        5'b10110: begin 
                            a_data22[ptr_a22] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a22 = ptr_a22 + 1;
                        end
                        5'b10111: begin 
                            a_data23[ptr_a23] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a23 = ptr_a23 + 1;
                        end
                        5'b11000: begin 
                            a_data24[ptr_a24] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a24 = ptr_a24 + 1;
                        end
                        5'b11001: begin 
                            a_data25[ptr_a25] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a25 = ptr_a25 + 1;
                        end
                        5'b11010: begin 
                            a_data26[ptr_a26] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a26 = ptr_a26 + 1;
                        end
                        5'b11011: begin 
                            a_data27[ptr_a27] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a27 = ptr_a27 + 1;
                        end
                        5'b11100: begin 
                            a_data28[ptr_a28] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a28 = ptr_a28 + 1;
                        end
                        5'b11101: begin 
                            a_data29[ptr_a29] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a29 = ptr_a29 + 1;
                        end
                        5'b11110: begin 
                            a_data30[ptr_a30] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a30 = ptr_a30 + 1;
                        end
                        5'b11111: begin 
                            a_data31[ptr_a31] = {col,safe_bit_select(a_row, precision_mode)};
                            ptr_a31 = ptr_a31 + 1;
                        end
                        default:;
                    endcase   
                end
        end

        // B矩阵加载
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin  // 异步复位，低电平有效
                ptr_b0 <= 0;
                ptr_b1 <= 0;
                ptr_b2 <= 0;
                ptr_b3 <= 0;
                ptr_b4 <= 0;
                ptr_b5 <= 0;
                ptr_b6 <= 0;
                ptr_b7 <= 0;
                ptr_b8 <= 0;
                ptr_b9 <= 0;
                ptr_b10 <= 0;
                ptr_b11 <= 0;
                ptr_b12 <= 0;
                ptr_b13 <= 0;
                ptr_b14 <= 0;
                ptr_b15 <= 0;
                ptr_b16 <= 0;
                ptr_b17 <= 0;
                ptr_b18 <= 0;
                ptr_b19 <= 0;
                ptr_b20 <= 0;
                ptr_b21 <= 0;
                ptr_b22 <= 0;
                ptr_b23 <= 0;
                ptr_b24 <= 0;
                ptr_b25 <= 0;
                ptr_b26 <= 0;
                ptr_b27 <= 0;
                ptr_b28 <= 0;
                ptr_b29 <= 0;
                ptr_b30 <= 0;
                ptr_b31 <= 0;
            // 复位时将所有寄存器清零
            for (int i = 0; i < MAX_K; i++) begin
                b_data00[i] <= 0;
                b_data01[i] <= 0;
                b_data02[i] <= 0;
                b_data03[i] <= 0;
                b_data04[i] <= 0;
                b_data05[i] <= 0;
                b_data06[i] <= 0;
                b_data07[i] <= 0;
                b_data08[i] <= 0;
                b_data09[i] <= 0;
                b_data10[i] <= 0;
                b_data11[i] <= 0;
                b_data12[i] <= 0;
                b_data13[i] <= 0;
                b_data14[i] <= 0;
                b_data15[i] <= 0;
                b_data16[i] <= 0;
                b_data17[i] <= 0;
                b_data18[i] <= 0;
                b_data19[i] <= 0;
                b_data20[i] <= 0;
                b_data21[i] <= 0;
                b_data22[i] <= 0;
                b_data23[i] <= 0;
                b_data24[i] <= 0;
                b_data25[i] <= 0;
                b_data26[i] <= 0;
                b_data27[i] <= 0;
                b_data28[i] <= 0;
                b_data29[i] <= 0;
                b_data30[i] <= 0;
                b_data31[i] <= 0;
            end
        end
        else if (load_b_en && s_axi_bready && (cycle_counter < k*n) && (b_col !=0)) begin  // 正常加载逻辑
                    automatic logic[4:0] row = cycle_counter / n;
                    automatic logic[4:0] col = cycle_counter % n;
        case(col)
                        5'b00000: begin 
                            b_data00[ptr_b0] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b0 = ptr_b0 + 1;
                        end
                        5'b00001: begin 
                            b_data01[ptr_b1] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b1 = ptr_b1 + 1;
                        end
                        5'b00010: begin 
                            b_data02[ptr_b2] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b2 = ptr_b2 + 1;
                        end
                        5'b00011: begin 
                            b_data03[ptr_b3] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b3 = ptr_b3 + 1;
                        end
                        5'b00100: begin 
                            b_data04[ptr_b4] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b4 = ptr_b4 + 1;
                        end
                        5'b00101: begin 
                            b_data05[ptr_b5] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b5 = ptr_b5 + 1;
                        end
                        5'b00110: begin 
                            b_data06[ptr_b6] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b6 = ptr_b6 + 1;
                        end
                        5'b00111: begin 
                            b_data07[ptr_b7] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b7 = ptr_b7 + 1;
                        end
                        5'b01000: begin 
                            b_data08[ptr_b8] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b8 = ptr_b8 + 1;
                        end
                        5'b01001: begin 
                            b_data09[ptr_b9] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b9 = ptr_b9 + 1;
                        end
                        5'b01010: begin 
                            b_data10[ptr_b10] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b10 = ptr_b10 + 1;
                        end
                        5'b01011: begin 
                            b_data11[ptr_b11] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b11 = ptr_b11 + 1;
                        end
                        5'b01100: begin 
                            b_data12[ptr_b12] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b12 = ptr_b12 + 1;
                        end
                        5'b01101: begin 
                            b_data13[ptr_b13] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b13 = ptr_b13 + 1;
                        end
                        5'b01110: begin 
                            b_data14[ptr_b14] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b14 = ptr_b14 + 1;
                        end
                        5'b01111: begin 
                            b_data15[ptr_b15] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b15 = ptr_b15 + 1;
                        end
                        5'b10000: begin 
                            b_data16[ptr_b16] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b16 = ptr_b16 + 1;
                        end
                        5'b10001: begin 
                            b_data17[ptr_b17] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b17 = ptr_b17 + 1;
                        end
                        5'b10010: begin 
                            b_data18[ptr_b18] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b18 = ptr_b18 + 1;
                        end
                        5'b10011: begin 
                            b_data19[ptr_b19] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b19 = ptr_b19 + 1;
                        end
                        5'b10100: begin 
                            b_data20[ptr_b20] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b20 = ptr_b20 + 1;
                        end
                        5'b10101: begin 
                            b_data21[ptr_b21] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b21 = ptr_b21 + 1;
                        end
                        5'b10110: begin 
                            b_data22[ptr_b22] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b22 = ptr_b22 + 1;
                        end
                        5'b10111: begin 
                            b_data23[ptr_b23] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b23 = ptr_b23 + 1;
                        end
                        5'b11000: begin 
                            b_data24[ptr_b24] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b24 = ptr_b24 + 1;
                        end
                        5'b11001: begin 
                            b_data25[ptr_b25] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b25 = ptr_b25 + 1;
                        end
                        5'b11010: begin 
                            b_data26[ptr_b26] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b26 = ptr_b26 + 1;
                        end
                        5'b11011: begin 
                            b_data27[ptr_b27] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b27 = ptr_b27 + 1;
                        end
                        5'b11100: begin 
                            b_data28[ptr_b28] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b28 = ptr_b28 + 1;
                        end
                        5'b11101: begin 
                            b_data29[ptr_b29] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b29 = ptr_b29 + 1;
                        end
                        5'b11110: begin 
                            b_data30[ptr_b30] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b30 = ptr_b30 + 1;
                        end
                        5'b11111: begin 
                            b_data31[ptr_b31] = {row,safe_bit_select(b_col, precision_mode)};
                            ptr_b31 = ptr_b31 + 1;
                        end
                        default:;
                    endcase
                end
        end
    
endmodule

(*use_dsp ="no" *)module FSM_Controller#(
   parameter PE_ROW_MAX = 8,
   parameter PE_COL_MAX = 8
) (
    input              clk,
    input              rst_n,
    // 配置接口
    //m/n/k/start都是apb给的       
    input wire [5:0]   m, n, k,
    input              start,
    // input wire [3:0]   elements_per_word,
    //!!!来自AXI从控制器，指示当前加载的数据地址
    input wire [31:0]  s_addr,
    //!!!来自AXI主控制器，指示输出阶段已传输的数据量
    input wire [11:0]  beat_cnt,
    // 状态输出
    output reg [3:0]   current_state,
    output reg [11:0]  cycle_counter,
    output reg         done,
    output reg         pe_enable,
    // 加载使能
    output reg         load_c_en,
    output reg         load_a_en,
    output reg         load_b_en,
    output reg         pe_load_c_en,
    output reg [6:0]   compute_counter, // 保持7-bit
    output reg [2:0]   pe_counter,   // 保持3-bit,pe_counter是索引现在计算的是分块的哪一个块。 0 1 2 三个分区！
    // 输出使能
    output reg         out_en
);

    // 状态定义
    localparam IDLE     = 4'd0; 
    localparam LOAD_C   = 4'd1;
    localparam LOAD_A   = 4'd2;
    localparam LOAD_B   = 4'd3;
    localparam COMPUTE  = 4'd4;
    localparam OUTPUT   = 4'd5;

    // 内部信号
    reg start_d;
    wire start_posedge;
    assign start_posedge = start & ~start_d;

    // 控制状态转移与操作
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state      <= IDLE;
            done               <= 0;
            cycle_counter      <= 0;
            pe_enable          <= 0;
            pe_load_c_en       <= 0;
            compute_counter    <= 0;
            load_c_en          <= 0;
            load_a_en          <= 0;
            load_b_en          <= 0;
            start_d            <= 0;
            out_en             <= 0;
            pe_counter         <= 0;
        end else begin
            start_d <= start;  // 用于上升沿检测

        case (current_state)
            IDLE: begin
                done            <= 0;
                pe_enable       <= 0;
                compute_counter <= 0;
                cycle_counter   <= 0;
                pe_counter      <= 0;
                pe_load_c_en    <= 0;

                //检测到start信号的上升沿（start_posedge）
                if (start_posedge) begin
                    current_state <= LOAD_C;
                    load_c_en <= 1;
                end
            end

            LOAD_C: begin
                // m * n：C矩阵总元素数量（行×列）一共一个矩阵的数字总数
                // / elements_per_word：每个时钟周期能加载的元素数（4位的话那一个时钟周期就是处理8个数）
                // - 1：因为从0开始计数
                if (cycle_counter < (m * n) - 1) begin
                    cycle_counter <= (s_addr == 0) ? cycle_counter  : s_addr;  // 计数器递增（s_addr控制）
                    //！！如果s_addr为零那么cycle不递增，其实感觉可以保留自动更新的逻辑，也可以由外部控制？
                end else begin
                    current_state <= LOAD_A;
                    cycle_counter <= 0;
                    load_c_en <= 0;
                    load_a_en <= 1;
                end
            end

            LOAD_A: begin                    
                if (cycle_counter < (m * k) - 1) begin
                    cycle_counter <= (s_addr == 0) ? cycle_counter  : s_addr;  // 计数器递增（s_addr控制）
                end else begin
                    current_state <= LOAD_B;
                    cycle_counter <= 0;
                    load_a_en <= 0;
                    load_b_en <= 1;                        
                end
            end

            LOAD_B: begin
                if (cycle_counter < (k * n) - 1) begin
                    cycle_counter <= (s_addr == 0) ? cycle_counter : s_addr;  // 计数器递增（s_addr控制）
                end else begin
                    current_state <= COMPUTE;
                    cycle_counter <= 0;
                    load_b_en <= 0;
                end
            end

            COMPUTE: begin

                // integer propagation_delay;
                // propagation_delay = m + n ;//修改！！
                pe_enable <= 1; // 在计算期间打开

                if (cycle_counter == 0) begin
                    compute_counter <= 0;
                    pe_load_c_en    <= 1;
                end else begin
                    pe_load_c_en <= 0;
                end

                //更改compute_counter索引逻辑，要求符合分块计算加载逻辑！！
                if (compute_counter <= ( k + PE_ROW_MAX + PE_COL_MAX )) begin//修改！！！脉动行、列位置索引计数器！！！
                    compute_counter <= compute_counter + 1;
                    pe_enable <= 1;
                end else begin
                    compute_counter <= compute_counter;//加else，分块循环索引，循环四次
                end                    

                //每个分块矩阵计算完都输出再进行下一次计算下一个分块！！一直到计算完所有分块！！
                //5级流水线 + 4 
                //稀疏4：2   k + PE_ROW_MAX + PE_COL_MAX - 1
                if (cycle_counter >= ((k + PE_ROW_MAX + PE_COL_MAX - 1) + 4 )) begin//！！逻辑发群里了，详细跳转逻辑的讲解
                    current_state <= OUTPUT;
                    cycle_counter <= 0;
                    compute_counter <= 0;
                    pe_enable <= 0;//转换输出时使能降为零
                end else begin
                    cycle_counter <= cycle_counter + 1;
                end

            end

            OUTPUT: begin

                    //分块计算后输出的是每一个小的分块计算的d矩阵结果！！
                if (cycle_counter < (PE_ROW_MAX * PE_COL_MAX)) begin
                        out_en <= 1;  // 保持输出使能
                        cycle_counter <= beat_cnt - pe_counter*(PE_ROW_MAX * PE_COL_MAX);  // 输出计数器递增 -64!修改这里的逻辑
                        // 计数器递增（beat_cnt控制）;master控制！！！分块计算后需要对应修改beat_cnt逻辑！！
                        current_state <= OUTPUT;  // 保持输出状态
                end else begin
                        // 当前分块输出完成
                        out_en <= 0;  // 关闭输出使能
                        cycle_counter <= 0;  // 重置输出计数器

                        // 判断是否还有分块需要计算
                        if (pe_counter < 2'd3) begin//再改pe_counter逻辑！！分块更小！！
                            pe_counter <= pe_counter + 1;  // 分块计数器递增
                            done <= 0;  // 未完成所有分块的计算，不置1
                            current_state <= COMPUTE;  // 进入计算状态处理下一分块
                        end else begin
                            pe_counter <= 0;  // 重置分块计数器
                            done <= 1;  // 所有分块处理完成才把done置1
                            current_state <= IDLE;  // 返回空闲状态
                        end
                end

            end

            default: begin
                current_state <= IDLE;
            end

        endcase
        
        end
    end

endmodule


(*use_dsp ="no" *)module tensor_compute_unit #(
   parameter MAX_K = 8,
   parameter C_ROW_MAX = 16,
   parameter C_COL_MAX = 16,
   parameter PE_ROW_MAX = 8,
   parameter PE_COL_MAX = 8,
   parameter IDLE     = 4'd0,
   parameter LOAD_C   = 4'd1,
   parameter LOAD_A   = 4'd2,
   parameter LOAD_B   = 4'd3,
   parameter COMPUTE  = 4'd4,
   parameter OUTPUT   = 4'd5
)(
    input              clk,            // 时钟
    input              rst_n,          // 异步复位
    // 配置接口
    input  [2:0]       precision_mode, // 精度模式
    input              mixed_mode,     // 混合精度模式
    output reg         overflow,       // 溢出标志
    // 数据接口
    output reg  [63:0]  d_out,          // 输出数据对接master

    input  wire [1:0]  matrix_mode,//为了改进a，b总线连接需要新增信号！
    input  wire [3:0]  state,
    input wire  [11:0] cycle_counter,   // 改为12-bit输入
    input wire  [6:0]  compute_counter, 
    input wire  [2:0]  pe_counter,
    input  wire        pe_enable,
    input  wire        pe_load_c_en,

    //由储存数据传输过来，不用重新定义！！
    // 矩阵选择信号
    input reg [36:0] a_data00 [0:MAX_K-1],
    input reg [36:0] a_data01 [0:MAX_K-1],
    input reg [36:0] a_data02 [0:MAX_K-1],
    input reg [36:0] a_data03 [0:MAX_K-1],
    input reg [36:0] a_data04 [0:MAX_K-1],
    input reg [36:0] a_data05 [0:MAX_K-1],
    input reg [36:0] a_data06 [0:MAX_K-1],
    input reg [36:0] a_data07 [0:MAX_K-1],
    input reg [36:0] a_data08 [0:MAX_K-1],
    input reg [36:0] a_data09 [0:MAX_K-1],
    input reg [36:0] a_data10 [0:MAX_K-1],
    input reg [36:0] a_data11 [0:MAX_K-1],
    input reg [36:0] a_data12 [0:MAX_K-1],
    input reg [36:0] a_data13 [0:MAX_K-1],
    input reg [36:0] a_data14 [0:MAX_K-1],
    input reg [36:0] a_data15 [0:MAX_K-1],
    input reg [36:0] a_data16 [0:MAX_K-1],
    input reg [36:0] a_data17 [0:MAX_K-1],
    input reg [36:0] a_data18 [0:MAX_K-1],
    input reg [36:0] a_data19 [0:MAX_K-1],
    input reg [36:0] a_data20 [0:MAX_K-1],
    input reg [36:0] a_data21 [0:MAX_K-1],
    input reg [36:0] a_data22 [0:MAX_K-1],
    input reg [36:0] a_data23 [0:MAX_K-1],
    input reg [36:0] a_data24 [0:MAX_K-1],
    input reg [36:0] a_data25 [0:MAX_K-1],
    input reg [36:0] a_data26 [0:MAX_K-1],
    input reg [36:0] a_data27 [0:MAX_K-1],
    input reg [36:0] a_data28 [0:MAX_K-1],
    input reg [36:0] a_data29 [0:MAX_K-1],
    input reg [36:0] a_data30 [0:MAX_K-1],
    input reg [36:0] a_data31 [0:MAX_K-1],

    input reg [36:0] b_data00 [0:MAX_K-1],
    input reg [36:0] b_data01 [0:MAX_K-1],
    input reg [36:0] b_data02 [0:MAX_K-1],
    input reg [36:0] b_data03 [0:MAX_K-1],
    input reg [36:0] b_data04 [0:MAX_K-1],
    input reg [36:0] b_data05 [0:MAX_K-1],
    input reg [36:0] b_data06 [0:MAX_K-1],
    input reg [36:0] b_data07 [0:MAX_K-1],
    input reg [36:0] b_data08 [0:MAX_K-1],
    input reg [36:0] b_data09 [0:MAX_K-1],
    input reg [36:0] b_data10 [0:MAX_K-1],
    input reg [36:0] b_data11 [0:MAX_K-1],
    input reg [36:0] b_data12 [0:MAX_K-1],
    input reg [36:0] b_data13 [0:MAX_K-1],
    input reg [36:0] b_data14 [0:MAX_K-1],
    input reg [36:0] b_data15 [0:MAX_K-1],
    input reg [36:0] b_data16 [0:MAX_K-1],
    input reg [36:0] b_data17 [0:MAX_K-1],
    input reg [36:0] b_data18 [0:MAX_K-1],
    input reg [36:0] b_data19 [0:MAX_K-1],
    input reg [36:0] b_data20 [0:MAX_K-1],
    input reg [36:0] b_data21 [0:MAX_K-1],
    input reg [36:0] b_data22 [0:MAX_K-1],
    input reg [36:0] b_data23 [0:MAX_K-1],
    input reg [36:0] b_data24 [0:MAX_K-1],
    input reg [36:0] b_data25 [0:MAX_K-1],
    input reg [36:0] b_data26 [0:MAX_K-1],
    input reg [36:0] b_data27 [0:MAX_K-1],
    input reg [36:0] b_data28 [0:MAX_K-1],
    input reg [36:0] b_data29 [0:MAX_K-1],
    input reg [36:0] b_data30 [0:MAX_K-1],
    input reg [36:0] b_data31 [0:MAX_K-1],



    input reg [31:0] c_data [0:C_ROW_MAX-1][0:C_COL_MAX-1]

);
    
    wire [36:0] a_bus [1:PE_ROW_MAX][0:PE_COL_MAX];
    wire [36:0] b_bus [0:PE_ROW_MAX][1:PE_COL_MAX];
    reg  [63:0] d_out_buffer [1:PE_ROW_MAX][1:PE_COL_MAX];
    wire [1:PE_ROW_MAX][1:PE_COL_MAX] overflow_bus;

    reg [2:0] a_bus_index;    
    reg [2:0] b_bus_index;


    //8×8 PE阵列 新增ab总线行列定位！！
    always @(*) begin
        //初始化
        a_bus_index = 3'd0;
        b_bus_index = 3'd0;

        case (matrix_mode)
            2'b00: begin //{m,n,k}={16,16,16}
                    case (pe_counter)
                    2'b00: begin
                        a_bus_index = 3'd0;
                        b_bus_index = 3'd0;
                    end
                    2'b01: begin
                        a_bus_index = 3'd0;
                        b_bus_index = 3'd1;
                    end
                    2'b10: begin
                        a_bus_index = 3'd1;
                        b_bus_index = 3'd0;
                    end
                    2'b11: begin
                        a_bus_index = 3'd1;
                        b_bus_index = 3'd1;
                    end
                    default: begin
                        a_bus_index = 3'd0;
                        b_bus_index = 3'd0;
                    end
                endcase
            end
            2'b01: begin//{m,n,k}={32,8,16}
                b_bus_index = 3'd0;
                    case (pe_counter)
                        2'b00: begin
                        a_bus_index = 3'd0;
                        end
                        2'b01: begin
                        a_bus_index = 3'd1;
                        end
                        2'b10: begin
                        a_bus_index = 3'd2;
                        end
                        2'b11: begin
                        a_bus_index = 3'd3;
                        end
                        default: begin
                        a_bus_index = 3'd0;
                        end
                    endcase
            end
            2'b10: begin//{m,n,k}={8,32,16}
                a_bus_index = 3'd0;
                    case (pe_counter)
                        2'b00: begin
                        b_bus_index = 3'd0;
                        end
                        2'b01: begin
                        b_bus_index = 3'd1;
                        end
                        2'b10: begin
                        b_bus_index = 3'd2;
                        end
                        2'b11: begin
                        b_bus_index = 3'd3;
                        end
                        default: begin
                        b_bus_index = 3'd0;
                        end
                    endcase
            end
            default: begin
                a_bus_index = 3'd0;
                b_bus_index = 3'd0;
            end
        endcase
    end
    // A 总线寄存器定义
    reg [36:0] a_bus1;
    reg [36:0] a_bus2;
    reg [36:0] a_bus3;
    reg [36:0] a_bus4;
    reg [36:0] a_bus5;
    reg [36:0] a_bus6;
    reg [36:0] a_bus7;
    reg [36:0] a_bus8;

    // B 总线寄存器定义
    reg [36:0] b_bus1;
    reg [36:0] b_bus2;
    reg [36:0] b_bus3;
    reg [36:0] b_bus4;
    reg [36:0] b_bus5;
    reg [36:0] b_bus6;
    reg [36:0] b_bus7;
    reg [36:0] b_bus8; 

    reg [2:0] a_data_index_ptr[0:31];
    reg [2:0] b_data_index_ptr[0:31]; 

always @(*) begin
    if (!rst_n) begin
            a_bus1      <= 0;
            a_bus2      <= 0;
            a_bus3      <= 0;
            a_bus4      <= 0;
            a_bus5      <= 0;
            a_bus6      <= 0;
            a_bus7      <= 0;
            a_bus8      <= 0;
            b_bus1      <= 0;
            b_bus2      <= 0;
            b_bus3      <= 0;
            b_bus4      <= 0;
            b_bus5      <= 0;
            b_bus6      <= 0;
            b_bus7      <= 0;
            b_bus8      <= 0;
            
            for (int i = 0; i < 32; i++) begin
                a_data_index_ptr[i] <= 3'b0;
                b_data_index_ptr[i] <= 3'b0;  
            end
    end else begin
        
            // a_data指针更新逻辑
            case(a_bus_index)
                0: begin  // a_data00~a_data07
                    // a_bus1~a_bus8 赋值
                    if (((compute_counter-1) - 0) == a_data00[a_data_index_ptr[0]][36:32]) begin
                        a_bus1 = a_data00[a_data_index_ptr[0]];
                        a_data_index_ptr[0] = a_data_index_ptr[0] + 1;
                    end else begin
                        a_bus1 = 37'b0;
                        a_data_index_ptr[0] = a_data_index_ptr[0];
                    end
                    
                    if (((compute_counter-1) - 1) == a_data01[a_data_index_ptr[1]][36:32]) begin
                        a_bus2 = a_data01[a_data_index_ptr[1]];
                        a_data_index_ptr[1] = a_data_index_ptr[1] + 1;
                    end else begin
                        a_bus2 = 37'b0;
                        a_data_index_ptr[1] = a_data_index_ptr[1];
                    end
                    
                    if (((compute_counter-1) - 2) == a_data02[a_data_index_ptr[2]][36:32]) begin
                        a_bus3 = a_data02[a_data_index_ptr[2]];
                        a_data_index_ptr[2] = a_data_index_ptr[2] + 1;
                    end else begin
                        a_bus3 = 37'b0;
                        a_data_index_ptr[2] = a_data_index_ptr[2];
                    end
                    
                    if (((compute_counter-1) - 3) == a_data03[a_data_index_ptr[3]][36:32]) begin
                        a_bus4 = a_data03[a_data_index_ptr[3]];
                        a_data_index_ptr[3] = a_data_index_ptr[3] + 1;
                    end else begin
                        a_bus4 = 37'b0;
                        a_data_index_ptr[3] = a_data_index_ptr[3];
                    end
                    
                    if (((compute_counter-1) - 4) == a_data04[a_data_index_ptr[4]][36:32]) begin
                        a_bus5 = a_data04[a_data_index_ptr[4]];
                        a_data_index_ptr[4] = a_data_index_ptr[4] + 1;
                    end else begin
                        a_bus5 = 37'b0;
                        a_data_index_ptr[4] = a_data_index_ptr[4];
                    end
                    
                    if (((compute_counter-1) - 5) == a_data05[a_data_index_ptr[5]][36:32]) begin
                        a_bus6 = a_data05[a_data_index_ptr[5]];
                        a_data_index_ptr[5] = a_data_index_ptr[5] + 1;
                    end else begin
                        a_bus6 = 37'b0;
                        a_data_index_ptr[5] = a_data_index_ptr[5];
                    end
                    
                    if (((compute_counter-1) - 6) == a_data06[a_data_index_ptr[6]][36:32]) begin
                        a_bus7 = a_data06[a_data_index_ptr[6]];
                        a_data_index_ptr[6] = a_data_index_ptr[6] + 1;
                    end else begin
                        a_bus7 = 37'b0;
                        a_data_index_ptr[6] = a_data_index_ptr[6];
                    end
                    
                    if (((compute_counter-1) - 7) == a_data07[a_data_index_ptr[7]][36:32]) begin
                        a_bus8 = a_data07[a_data_index_ptr[7]];
                        a_data_index_ptr[7] = a_data_index_ptr[7] + 1;
                    end else begin
                        a_bus8 = 37'b0;
                        a_data_index_ptr[7] = a_data_index_ptr[7];
                    end
                end
                
                1: begin  // a_data08~a_data15
                    if (((compute_counter-1) - 0) == a_data08[a_data_index_ptr[8]][36:32]) begin
                        a_bus1 = a_data08[a_data_index_ptr[8]];
                        a_data_index_ptr[8] = a_data_index_ptr[8] + 1;
                    end else begin
                        a_bus1 = 37'b0;
                        a_data_index_ptr[8] = a_data_index_ptr[8];
                    end
                    
                    if (((compute_counter-1) - 1) == a_data09[a_data_index_ptr[9]][36:32]) begin
                        a_bus2 = a_data09[a_data_index_ptr[9]];
                        a_data_index_ptr[9] = a_data_index_ptr[9] + 1;
                    end else begin
                        a_bus2 = 37'b0;
                        a_data_index_ptr[9] = a_data_index_ptr[9];
                    end
                    
                    if (((compute_counter-1) - 2) == a_data10[a_data_index_ptr[10]][36:32]) begin
                        a_bus3 = a_data10[a_data_index_ptr[10]];
                        a_data_index_ptr[10] = a_data_index_ptr[10] + 1;
                    end else begin
                        a_bus3 = 37'b0;
                        a_data_index_ptr[10] = a_data_index_ptr[10];
                    end
                    
                    if (((compute_counter-1) - 3) == a_data11[a_data_index_ptr[11]][36:32]) begin
                        a_bus4 = a_data11[a_data_index_ptr[11]];
                        a_data_index_ptr[11] = a_data_index_ptr[11] + 1;
                    end else begin
                        a_bus4 = 37'b0;
                        a_data_index_ptr[11] = a_data_index_ptr[11];
                    end
                    
                    if (((compute_counter-1) - 4) == a_data12[a_data_index_ptr[12]][36:32]) begin
                        a_bus5 = a_data12[a_data_index_ptr[12]];
                        a_data_index_ptr[12] = a_data_index_ptr[12] + 1;
                    end else begin
                        a_bus5 = 37'b0;
                        a_data_index_ptr[12] = a_data_index_ptr[12];
                    end
                    
                    if (((compute_counter-1) - 5) == a_data13[a_data_index_ptr[13]][36:32]) begin
                        a_bus6 = a_data13[a_data_index_ptr[13]];
                        a_data_index_ptr[13] = a_data_index_ptr[13] + 1;
                    end else begin
                        a_bus6 = 37'b0;
                        a_data_index_ptr[13] = a_data_index_ptr[13];
                    end
                    
                    if (((compute_counter-1) - 6) == a_data14[a_data_index_ptr[14]][36:32]) begin
                        a_bus7 = a_data14[a_data_index_ptr[14]];
                        a_data_index_ptr[14] = a_data_index_ptr[14] + 1;
                    end else begin
                        a_bus7 = 37'b0;
                        a_data_index_ptr[14] = a_data_index_ptr[14];
                    end
                    
                    if (((compute_counter-1) - 7) == a_data15[a_data_index_ptr[15]][36:32]) begin
                        a_bus8 = a_data15[a_data_index_ptr[15]];
                        a_data_index_ptr[15] = a_data_index_ptr[15] + 1;
                    end else begin
                        a_bus8 = 37'b0;
                        a_data_index_ptr[15] = a_data_index_ptr[15];
                    end
                end
                
                2: begin  // a_data16~a_data23
                    if (((compute_counter-1) - 0) == a_data16[a_data_index_ptr[16]][36:32]) begin
                        a_bus1 = a_data16[a_data_index_ptr[16]];
                        a_data_index_ptr[16] = a_data_index_ptr[16] + 1;
                    end else begin
                        a_bus1 = 37'b0;
                        a_data_index_ptr[16] = a_data_index_ptr[16];
                    end
                    
                    if (((compute_counter-1) - 1) == a_data17[a_data_index_ptr[17]][36:32]) begin
                        a_bus2 = a_data17[a_data_index_ptr[17]];
                        a_data_index_ptr[17] = a_data_index_ptr[17] + 1;
                    end else begin
                        a_bus2 = 37'b0;
                        a_data_index_ptr[17] = a_data_index_ptr[17];
                    end
                    
                    if (((compute_counter-1) - 2) == a_data18[a_data_index_ptr[18]][36:32]) begin
                        a_bus3 = a_data18[a_data_index_ptr[18]];
                        a_data_index_ptr[18] = a_data_index_ptr[18] + 1;
                    end else begin
                        a_bus3 = 37'b0;
                        a_data_index_ptr[18] = a_data_index_ptr[18];
                    end
                    
                    if (((compute_counter-1) - 3) == a_data19[a_data_index_ptr[19]][36:32]) begin
                        a_bus4 = a_data19[a_data_index_ptr[19]];
                        a_data_index_ptr[19] = a_data_index_ptr[19] + 1;
                    end else begin
                        a_bus4 = 37'b0;
                        a_data_index_ptr[19] = a_data_index_ptr[19];
                    end
                    
                    if (((compute_counter-1) - 4) == a_data20[a_data_index_ptr[20]][36:32]) begin
                        a_bus5 = a_data20[a_data_index_ptr[20]];
                        a_data_index_ptr[20] = a_data_index_ptr[20] + 1;
                    end else begin
                        a_bus5 = 37'b0;
                        a_data_index_ptr[20] = a_data_index_ptr[20];
                    end
                    
                    if (((compute_counter-1) - 5) == a_data21[a_data_index_ptr[21]][36:32]) begin
                        a_bus6 = a_data21[a_data_index_ptr[21]];
                        a_data_index_ptr[21] = a_data_index_ptr[21] + 1;
                    end else begin
                        a_bus6 = 37'b0;
                        a_data_index_ptr[21] = a_data_index_ptr[21];
                    end
                    
                    if (((compute_counter-1) - 6) == a_data22[a_data_index_ptr[22]][36:32]) begin
                        a_bus7 = a_data22[a_data_index_ptr[22]];
                        a_data_index_ptr[22] = a_data_index_ptr[22] + 1;
                    end else begin
                        a_bus7 = 37'b0;
                        a_data_index_ptr[22] = a_data_index_ptr[22];
                    end
                    
                    if (((compute_counter-1) - 7) == a_data23[a_data_index_ptr[23]][36:32]) begin
                        a_bus8 = a_data23[a_data_index_ptr[23]];
                        a_data_index_ptr[23] = a_data_index_ptr[23] + 1;
                    end else begin
                        a_bus8 = 37'b0;
                        a_data_index_ptr[23] = a_data_index_ptr[23];
                    end
                end
                
                3: begin  // a_data24~a_data31
                    if (((compute_counter-1) - 0) == a_data24[a_data_index_ptr[24]][36:32]) begin
                        a_bus1 = a_data24[a_data_index_ptr[24]];
                        a_data_index_ptr[24] = a_data_index_ptr[24] + 1;
                    end else begin
                        a_bus1 = 37'b0;
                        a_data_index_ptr[24] = a_data_index_ptr[24];
                    end
                    
                    if (((compute_counter-1) - 1) == a_data25[a_data_index_ptr[25]][36:32]) begin
                        a_bus2 = a_data25[a_data_index_ptr[25]];
                        a_data_index_ptr[25] = a_data_index_ptr[25] + 1;
                    end else begin
                        a_bus2 = 37'b0;
                        a_data_index_ptr[25] = a_data_index_ptr[25];
                    end
                    
                    if (((compute_counter-1) - 2) == a_data26[a_data_index_ptr[26]][36:32]) begin
                        a_bus3 = a_data26[a_data_index_ptr[26]];
                        a_data_index_ptr[26] = a_data_index_ptr[26] + 1;
                    end else begin
                        a_bus3 = 37'b0;
                        a_data_index_ptr[26] = a_data_index_ptr[26];
                    end
                    
                    if (((compute_counter-1) - 3) == a_data27[a_data_index_ptr[27]][36:32]) begin
                        a_bus4 = a_data27[a_data_index_ptr[27]];
                        a_data_index_ptr[27] = a_data_index_ptr[27] + 1;
                    end else begin
                        a_bus4 = 37'b0;
                        a_data_index_ptr[27] = a_data_index_ptr[27];
                    end
                    
                    if (((compute_counter-1) - 4) == a_data28[a_data_index_ptr[28]][36:32]) begin
                        a_bus5 = a_data28[a_data_index_ptr[28]];
                        a_data_index_ptr[28] = a_data_index_ptr[28] + 1;
                    end else begin
                        a_bus5 = 37'b0;
                        a_data_index_ptr[28] = a_data_index_ptr[28];
                    end
                    
                    if (((compute_counter-1) - 5) == a_data29[a_data_index_ptr[29]][36:32]) begin
                        a_bus6 = a_data29[a_data_index_ptr[29]];
                        a_data_index_ptr[29] = a_data_index_ptr[29] + 1;
                    end else begin
                        a_bus6 = 37'b0;
                        a_data_index_ptr[29] = a_data_index_ptr[29];
                    end
                    
                    if (((compute_counter-1) - 6) == a_data30[a_data_index_ptr[30]][36:32]) begin
                        a_bus7 = a_data30[a_data_index_ptr[30]];
                        a_data_index_ptr[30] = a_data_index_ptr[30] + 1;
                    end else begin
                        a_bus7 = 37'b0;
                        a_data_index_ptr[30] = a_data_index_ptr[30];
                    end
                    
                    if (((compute_counter-1) - 7) == a_data31[a_data_index_ptr[31]][36:32]) begin
                        a_bus8 = a_data31[a_data_index_ptr[31]];
                        a_data_index_ptr[31] = a_data_index_ptr[31] + 1;
                    end else begin
                        a_bus8 = 37'b0;
                        a_data_index_ptr[31] = a_data_index_ptr[31];
                    end
                end
                
                default: begin
                    a_bus1 <= 37'b0;
                    a_bus2 <= 37'b0;
                    a_bus3 <= 37'b0;
                    a_bus4 <= 37'b0;
                    a_bus5 <= 37'b0;
                    a_bus6 <= 37'b0;
                    a_bus7 <= 37'b0;
                    a_bus8 <= 37'b0;

                    for (int i = 0; i < 32; i++) begin
                        a_data_index_ptr[i] <= a_data_index_ptr[i]; 
                    end

                end
            endcase
        
            // b_data指针更新逻辑（与a_data完全对称）
            case(b_bus_index)
                0: begin  // b_data00~b_data07
                    if (((compute_counter-1) - 0) == b_data00[b_data_index_ptr[0]][36:32]) begin
                        b_bus1 = b_data00[b_data_index_ptr[0]];
                        b_data_index_ptr[0] = b_data_index_ptr[0] + 1;
                    end else begin
                        b_bus1 = 37'b0;
                        b_data_index_ptr[0] = b_data_index_ptr[0];
                    end
                    
                    if (((compute_counter-1) - 1) == b_data01[b_data_index_ptr[1]][36:32]) begin
                        b_bus2 = b_data01[b_data_index_ptr[1]];
                        b_data_index_ptr[1] = b_data_index_ptr[1] + 1;
                    end else begin
                        b_bus2 = 37'b0;
                        b_data_index_ptr[1] = b_data_index_ptr[1];
                    end
                    
                    if (((compute_counter-1) - 2) == b_data02[b_data_index_ptr[2]][36:32]) begin
                        b_bus3 = b_data02[b_data_index_ptr[2]];
                        b_data_index_ptr[2] = b_data_index_ptr[2] + 1;
                    end else begin
                        b_bus3 = 37'b0;
                        b_data_index_ptr[2] = b_data_index_ptr[2];
                    end
                    
                    if (((compute_counter-1) - 3) == b_data03[b_data_index_ptr[3]][36:32]) begin
                        b_bus4 = b_data03[b_data_index_ptr[3]];
                        b_data_index_ptr[3] = b_data_index_ptr[3] + 1;
                    end else begin
                        b_bus4 = 37'b0;
                        b_data_index_ptr[3] = b_data_index_ptr[3];
                    end
                    
                    if (((compute_counter-1) - 4) == b_data04[b_data_index_ptr[4]][36:32]) begin
                        b_bus5 = b_data04[b_data_index_ptr[4]];
                        b_data_index_ptr[4] = b_data_index_ptr[4] + 1;
                    end else begin
                        b_bus5 = 37'b0;
                        b_data_index_ptr[4] = b_data_index_ptr[4];
                    end
                    
                    if (((compute_counter-1) - 5) == b_data05[b_data_index_ptr[5]][36:32]) begin
                        b_bus6 = b_data05[b_data_index_ptr[5]];
                        b_data_index_ptr[5] = b_data_index_ptr[5] + 1;
                    end else begin
                        b_bus6 = 37'b0;
                        b_data_index_ptr[5] = b_data_index_ptr[5];
                    end
                    
                    if (((compute_counter-1) - 6) == b_data06[b_data_index_ptr[6]][36:32]) begin
                        b_bus7 = b_data06[b_data_index_ptr[6]];
                        b_data_index_ptr[6] = b_data_index_ptr[6] + 1;
                    end else begin
                        b_bus7 = 37'b0;
                        b_data_index_ptr[6] = b_data_index_ptr[6];
                    end
                    
                    if (((compute_counter-1) - 7) == b_data07[b_data_index_ptr[7]][36:32]) begin
                        b_bus8 = b_data07[b_data_index_ptr[7]];
                        b_data_index_ptr[7] = b_data_index_ptr[7] + 1;
                    end else begin
                        b_bus8 = 37'b0;
                        b_data_index_ptr[7] = b_data_index_ptr[7];
                    end
                end
                
                1: begin  // b_data08~b_data15
                    if (((compute_counter-1) - 0) == b_data08[b_data_index_ptr[8]][36:32]) begin
                        b_bus1 = b_data08[b_data_index_ptr[8]];
                        b_data_index_ptr[8] = b_data_index_ptr[8] + 1;
                    end else begin
                        b_bus1 = 37'b0;
                        b_data_index_ptr[8] = b_data_index_ptr[8];
                    end
                    
                    if (((compute_counter-1) - 1) == b_data09[b_data_index_ptr[9]][36:32]) begin
                        b_bus2 = b_data09[b_data_index_ptr[9]];
                        b_data_index_ptr[9] = b_data_index_ptr[9] + 1;
                    end else begin
                        b_bus2 = 37'b0;
                        b_data_index_ptr[9] = b_data_index_ptr[9];
                    end
                    
                    if (((compute_counter-1) - 2) == b_data10[b_data_index_ptr[10]][36:32]) begin
                        b_bus3 = b_data10[b_data_index_ptr[10]];
                        b_data_index_ptr[10] = b_data_index_ptr[10] + 1;
                    end else begin
                        b_bus3 = 37'b0;
                        b_data_index_ptr[10] = b_data_index_ptr[10];
                    end
                    
                    if (((compute_counter-1) - 3) == b_data11[b_data_index_ptr[11]][36:32]) begin
                        b_bus4 = b_data11[b_data_index_ptr[11]];
                        b_data_index_ptr[11] = b_data_index_ptr[11] + 1;
                    end else begin
                        b_bus4 = 37'b0;
                        b_data_index_ptr[11] = b_data_index_ptr[11];
                    end
                    
                    if (((compute_counter-1) - 4) == b_data12[b_data_index_ptr[12]][36:32]) begin
                        b_bus5 = b_data12[b_data_index_ptr[12]];
                        b_data_index_ptr[12] = b_data_index_ptr[12] + 1;
                    end else begin
                        b_bus5 = 37'b0;
                        b_data_index_ptr[12] = b_data_index_ptr[12];
                    end
                    
                    if (((compute_counter-1) - 5) == b_data13[b_data_index_ptr[13]][36:32]) begin
                        b_bus6 = b_data13[b_data_index_ptr[13]];
                        b_data_index_ptr[13] = b_data_index_ptr[13] + 1;
                    end else begin
                        b_bus6 = 37'b0;
                        b_data_index_ptr[13] = b_data_index_ptr[13];
                    end
                    
                    if (((compute_counter-1) - 6) == b_data14[b_data_index_ptr[14]][36:32]) begin
                        b_bus7 = b_data14[b_data_index_ptr[14]];
                        b_data_index_ptr[14] = b_data_index_ptr[14] + 1;
                    end else begin
                        b_bus7 = 37'b0;
                        b_data_index_ptr[14] = b_data_index_ptr[14];
                    end
                    
                    if (((compute_counter-1) - 7) == b_data15[b_data_index_ptr[15]][36:32]) begin
                        b_bus8 = b_data15[b_data_index_ptr[15]];
                        b_data_index_ptr[15] = b_data_index_ptr[15] + 1;
                    end else begin
                        b_bus8 = 37'b0;
                        b_data_index_ptr[15] = b_data_index_ptr[15];
                    end
                end
                
                2: begin  // b_data16~b_data23
                    if (((compute_counter-1) - 0) == b_data16[b_data_index_ptr[16]][36:32]) begin
                        b_bus1 = b_data16[b_data_index_ptr[16]];
                        b_data_index_ptr[16] = b_data_index_ptr[16] + 1;
                    end else begin
                        b_bus1 = 37'b0;
                        b_data_index_ptr[16] = b_data_index_ptr[16];
                    end
                    
                    if (((compute_counter-1) - 1) == b_data17[b_data_index_ptr[17]][36:32]) begin
                        b_bus2 = b_data17[b_data_index_ptr[17]];
                        b_data_index_ptr[17] = b_data_index_ptr[17] + 1;
                    end else begin
                        b_bus2 = 37'b0;
                        b_data_index_ptr[17] = b_data_index_ptr[17];
                    end
                    
                    if (((compute_counter-1) - 2) == b_data18[b_data_index_ptr[18]][36:32]) begin
                        b_bus3 = b_data18[b_data_index_ptr[18]];
                        b_data_index_ptr[18] = b_data_index_ptr[18] + 1;
                    end else begin
                        b_bus3 = 37'b0;
                        b_data_index_ptr[18] = b_data_index_ptr[18];
                    end
                    
                    if (((compute_counter-1) - 3) == b_data19[b_data_index_ptr[19]][36:32]) begin
                        b_bus4 = b_data19[b_data_index_ptr[19]];
                        b_data_index_ptr[19] = b_data_index_ptr[19] + 1;
                    end else begin
                        b_bus4 = 37'b0;
                        b_data_index_ptr[19] = b_data_index_ptr[19];
                    end
                    
                    if (((compute_counter-1) - 4) == b_data20[b_data_index_ptr[20]][36:32]) begin
                        b_bus5 = b_data20[b_data_index_ptr[20]];
                        b_data_index_ptr[20] = b_data_index_ptr[20] + 1;
                    end else begin
                        b_bus5 = 37'b0;
                        b_data_index_ptr[20] = b_data_index_ptr[20];
                    end
                    
                    if (((compute_counter-1) - 5) == b_data21[b_data_index_ptr[21]][36:32]) begin
                        b_bus6 = b_data21[b_data_index_ptr[21]];
                        b_data_index_ptr[21] = b_data_index_ptr[21] + 1;
                    end else begin
                        b_bus6 = 37'b0;
                        b_data_index_ptr[21] = b_data_index_ptr[21];
                    end
                    
                    if (((compute_counter-1) - 6) == b_data22[b_data_index_ptr[22]][36:32]) begin
                        b_bus7 = b_data22[b_data_index_ptr[22]];
                        b_data_index_ptr[22] = b_data_index_ptr[22] + 1;
                    end else begin
                        b_bus7 = 37'b0;
                        b_data_index_ptr[22] = b_data_index_ptr[22];
                    end
                    
                    if (((compute_counter-1) - 7) == b_data23[b_data_index_ptr[23]][36:32]) begin
                        b_bus8 = b_data23[b_data_index_ptr[23]];
                        b_data_index_ptr[23] = b_data_index_ptr[23] + 1;
                    end else begin
                        b_bus8 = 37'b0;
                        b_data_index_ptr[23] = b_data_index_ptr[23];
                    end
                end
                
                3: begin  // b_data24~b_data31
                    if (((compute_counter-1) - 0) == b_data24[b_data_index_ptr[24]][36:32]) begin
                        b_bus1 = b_data24[b_data_index_ptr[24]];
                        b_data_index_ptr[24] = b_data_index_ptr[24] + 1;
                    end else begin
                        b_bus1 = 37'b0;
                        b_data_index_ptr[24] = b_data_index_ptr[24];
                    end
                    
                    if (((compute_counter-1) - 1) == b_data25[b_data_index_ptr[25]][36:32]) begin
                        b_bus2 = b_data25[b_data_index_ptr[25]];
                        b_data_index_ptr[25] = b_data_index_ptr[25] + 1;
                    end else begin
                        b_bus2 = 37'b0;
                        b_data_index_ptr[25] = b_data_index_ptr[25];
                    end
                    
                    if (((compute_counter-1) - 2) == b_data26[b_data_index_ptr[26]][36:32]) begin
                        b_bus3 = b_data26[b_data_index_ptr[26]];
                        b_data_index_ptr[26] = b_data_index_ptr[26] + 1;
                    end else begin
                        b_bus3 = 37'b0;
                        b_data_index_ptr[26] = b_data_index_ptr[26];
                    end
                    
                    if (((compute_counter-1) - 3) == b_data27[b_data_index_ptr[27]][36:32]) begin
                        b_bus4 = b_data27[b_data_index_ptr[27]];
                        b_data_index_ptr[27] = b_data_index_ptr[27] + 1;
                    end else begin
                        b_bus4 = 37'b0;
                        b_data_index_ptr[27] = b_data_index_ptr[27];
                    end
                    
                    if (((compute_counter-1) - 4) == b_data28[b_data_index_ptr[28]][36:32]) begin
                        b_bus5 = b_data28[b_data_index_ptr[28]];
                        b_data_index_ptr[28] = b_data_index_ptr[28] + 1;
                    end else begin
                        b_bus5 = 37'b0;
                        b_data_index_ptr[28] = b_data_index_ptr[28];
                    end
                    
                    if (((compute_counter-1) - 5) == b_data29[b_data_index_ptr[29]][36:32]) begin
                        b_bus6 = b_data29[b_data_index_ptr[29]];
                        b_data_index_ptr[29] = b_data_index_ptr[29] + 1;
                    end else begin
                        b_bus6 = 37'b0;
                        b_data_index_ptr[29] = b_data_index_ptr[29];
                    end
                    
                    if (((compute_counter-1) - 6) == b_data30[b_data_index_ptr[30]][36:32]) begin
                        b_bus7 = b_data30[b_data_index_ptr[30]];
                        b_data_index_ptr[30] = b_data_index_ptr[30] + 1;
                    end else begin
                        b_bus7 = 37'b0;
                        b_data_index_ptr[30] = b_data_index_ptr[30];
                    end
                    
                    if (((compute_counter-1) - 7) == b_data31[b_data_index_ptr[31]][36:32]) begin
                        b_bus8 = b_data31[b_data_index_ptr[31]];
                        b_data_index_ptr[31] = b_data_index_ptr[31] + 1;
                    end else begin
                        b_bus8 = 37'b0;
                        b_data_index_ptr[31] = b_data_index_ptr[31];
                    end
                end
                
                default: begin
                    b_bus1 <= 37'b0;
                    b_bus2 <= 37'b0;
                    b_bus3 <= 37'b0;
                    b_bus4 <= 37'b0;
                    b_bus5 <= 37'b0;
                    b_bus6 <= 37'b0;
                    b_bus7 <= 37'b0;
                    b_bus8 <= 37'b0;

                    for (int i = 0; i < 32; i++) begin
                        b_data_index_ptr[i] <= b_data_index_ptr[i];  
                    end
                end
            endcase
        end
    end

// 8×8 PE阵列 - a_bus定义
assign a_bus[1][0]  =  a_bus1;
assign a_bus[2][0]  =  a_bus2;
assign a_bus[3][0]  =  a_bus3;
assign a_bus[4][0]  =  a_bus4;
assign a_bus[5][0]  =  a_bus5;
assign a_bus[6][0]  =  a_bus6;
assign a_bus[7][0]  =  a_bus7;
assign a_bus[8][0]  =  a_bus8;

// 8×8 PE阵列 - b_bus定义
assign b_bus[0][1]  =  b_bus1;
assign b_bus[0][2]  =  b_bus2;
assign b_bus[0][3]  =  b_bus3;
assign b_bus[0][4]  =  b_bus4;
assign b_bus[0][5]  =  b_bus5;
assign b_bus[0][6]  =  b_bus6;
assign b_bus[0][7]  =  b_bus7;
assign b_bus[0][8]  =  b_bus8;

reg [2:0] c_row_index, c_col_index;

always @* begin
    //初始化
   c_row_index = 3'd0;
   c_col_index = 3'd0;

        case (pe_counter)
            2'b00: begin
                c_row_index= 3'd0;
                c_col_index = 3'd0;
            end
            2'b01: begin
                c_row_index = 3'd0;
                c_col_index = 3'd1;
            end
            2'b10: begin
                c_row_index = 3'd1;
                c_col_index = 3'd0;
            end
            2'b11: begin
                c_row_index = 3'd1;
                c_col_index = 3'd1;
            end
            default: begin
                c_row_index = 3'd0;
                c_col_index = 3'd0;
            end
        endcase
end

generate
    for (genvar i = 1; i <= PE_ROW_MAX; i++) begin: pe_row
        for (genvar j = 1; j <= PE_COL_MAX; j++) begin: pe_col
            ProcessingElement pe (
                .clk(clk),
                .rst_n(rst_n),
                .pe_enable(pe_enable),
                .precision_mode(precision_mode),
                .mixed_mode(mixed_mode),
                .pe_load_c_en(pe_load_c_en),
                .a_in(a_bus[i][j-1]),   
                .b_in(b_bus[i-1][j]),  
                .c_in(c_data[ i-1 + (c_row_index * PE_ROW_MAX) ][ j-1 + (c_col_index * PE_COL_MAX) ]), //修改c矩阵载入数据!!
                .a_out(a_bus[i][j]),    // 向右传递A
                .b_out(b_bus[i][j]),    // 向下传递B
                .d_out(d_out_buffer[i][j]),   //改变信号类型！！reg，要存着这个信号
                .overflow(overflow_bus[i][j])
            );
        end
    end
endgenerate

// 溢出信号聚合逻辑
always @(posedge clk) begin
    if(state == COMPUTE) begin
        overflow <= |(overflow_bus); // 所有PE的溢出信号或操作
    end else begin
        overflow <= 0;
    end
end

always @(posedge clk) begin
    if(state == OUTPUT) begin
        // 计算输出矩阵坐标
        automatic int row = cycle_counter / PE_COL_MAX;
        automatic int col = cycle_counter % PE_COL_MAX;
        
        // 边界保护
        if(row < PE_ROW_MAX && col < PE_COL_MAX) begin
            d_out <= d_out_buffer[row+1][col+1]; // PE阵列从(1,1)开始
        end else begin
            d_out <= 32'hDEADBEEF; // 错误标记
        end
    end else begin
        d_out <= 0;
    end
end

endmodule


(*use_dsp ="no" *)module ProcessingElement (
    input              clk,           
    input              rst_n,          
    input              pe_enable,
    input  [2:0]       precision_mode, 
    input              mixed_mode,
    input              pe_load_c_en,      
    input  [36:0]      a_in,          
    input  [36:0]      b_in,           
    input  [31:0]      c_in,       
    output reg [36:0]  a_out,         
    output reg [36:0]  b_out,         
    output reg  [63:0] d_out,          
    output reg         overflow
);

// 累加器寄存器
reg signed [63:0]  int_acc;     // 整数累加器
reg [63:0]         fp_acc;      // 浮点累加器
reg [31:0]         bf16_acc;    // BF16累加器

// 流水线寄存器定义
// Stage1 -> Stage2
reg        pe_enable_s2;

// Stage2 -> Stage3
reg        pe_enable_s3;
reg [31:0] a_preprocessed;
reg [31:0] b_preprocessed;
// reg [63:0] a_fp_preprocessed;
// reg [63:0] b_fp_preprocessed;

// Stage3 -> Stage4
reg        pe_enable_s4;
reg [63:0] int_product;
reg        int_overflow;

reg [31:0] fp32_product;
// reg [63:0] fp64_product;
reg [15:0] bf16_product;

// Stage4 -> Stage5
reg [63:0] int_result;
reg [31:0] fp_result;
reg [31:0] bf16_result;
reg        overflow_int;

// 初始化所有寄存器

initial begin
    // 累加器寄存器
    int_acc = 0;
    fp_acc = 0;
    bf16_acc = 0;
    
    // Stage1 -> Stage2 寄存器
    pe_enable_s2 = 0;
    
    // Stage2 -> Stage3 寄存器
    pe_enable_s3 = 0;
    a_preprocessed = 0;
    b_preprocessed = 0;
    
    // Stage3 -> Stage4 寄存器
    pe_enable_s4 = 0;
    int_product = 0;
    fp32_product = 0;
    bf16_product = 0;
    
    // Stage4 -> Stage5 寄存器
    int_result = 0;
    fp_result = 0;
    bf16_result = 0;
    
    // 输出寄存器
    a_out = 0;
    b_out = 0;
    d_out = 0;
    overflow = 0;
end

// Stage1: 加载阶段 (加载C值)
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        int_acc <= 0;
        fp_acc <= 0;
        bf16_acc <= 0;
    end else if (pe_load_c_en) begin
        //重新加载的时候清零!!
        int_acc <= 0;
        fp_acc <= 0;
        bf16_acc <= 0;

        case(precision_mode)
            3'b000, 3'b001: begin
                case(mixed_mode)//整数下混合精度int4/int8混int32
                    0: int_acc <= preprocess(c_in);  // 将预处理后的C值直接加载到整数累加器
                    1: int_acc <= {$signed({{32{c_in[31]}},c_in})};
                    default: int_acc <= 0;
                endcase
            end
            3'b100: bf16_acc <= preprocess(c_in);         // BF16模式
            default: fp_acc <= preprocess(c_in);          // FP32模式
        endcase

    end else begin
        int_acc <= int_acc;
        fp_acc <= fp_acc;
        bf16_acc <= bf16_acc;
    end
end

// Stage2: 脉动数据传输 ＋ 输入乘加数据预处理阶段
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        a_out <= 0;
        b_out <= 0;
        pe_enable_s2 <= 0;
        a_preprocessed <= 0;
        b_preprocessed <= 0;
    end else begin

        // 脉动数据传递 ＋ 数据处理
        // 乘.累加数据预处理
        if (a_in)begin
            a_out <= preprocess(a_in);
        end else begin
            a_out <= 0;
        end

        if (b_in) begin
            b_out <= preprocess(b_in);
        end else begin
            b_out <= 0;
        end

        // 乘.累加数据预处理
        if (a_in)begin
            a_preprocessed <= preprocess(a_in);
        end else begin
            a_preprocessed <= 0;
        end

        if (b_in)begin
            b_preprocessed <= preprocess(b_in);
        end else begin
            b_out <= 0;
            b_preprocessed <= 0;
        end

        // 传递控制信号
        pe_enable_s2 <= pe_enable;
    end
end

// Stage3: 乘法阶段
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pe_enable_s3 <= 0;
        int_product <= 0;
        fp32_product <= 0;
        // fp64_product <= 0;
        bf16_product <= 0;
    end else begin
        // 传递控制信号
        pe_enable_s3 <= pe_enable_s2;
        
        // 执行乘法运算
        if (pe_enable_s2) begin
            if(a_preprocessed == 0 || b_preprocessed == 0) begin
                int_product <= 0;
                fp32_product <= 0;
                // fp64_product <= 0;
                bf16_product <= 0;
                int_overflow <= 0; // 标记溢出
            end else begin
                // 执行乘法运算
                case(precision_mode)
                    3'b000: {int_product, int_overflow}  <= int4_mul(a_preprocessed[3:0], b_preprocessed[3:0]);
                    3'b001: {int_product, int_overflow}  <= int8_mul(a_preprocessed[7:0], b_preprocessed[7:0]);
                    3'b100: bf16_product <= bf16_mult(a_preprocessed[15:0], b_preprocessed[15:0]);
                    default: begin
                    // FP32
                    fp32_product <= fp32_mult(a_preprocessed, b_preprocessed);

                    // //FP64
                    // a_fp_preprocessed = fp32_to_fp64(a_preprocessed);
                    // b_fp_preprocessed = fp32_to_fp64(b_preprocessed);
                    // fp64_product <= fp64_mult(a_fp_preprocessed, b_fp_preprocessed);
                    end
                endcase
            end 
        end else begin
            int_product <= 0;
            fp32_product <= 0;
            // fp64_product <= 0;
            bf16_product <= 0;
            int_overflow <= 0; // 标记溢出
        end
    end
end

// Stage4: 累加阶段
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pe_enable_s4 <= 0;
        int_result <= 0;
        fp_result <= 0;
        bf16_result <= 0;
        overflow <= 0;
    end else begin
        // 传递控制信号
        pe_enable_s4 <= pe_enable_s3;
        
        if (pe_enable_s3) begin
            case(precision_mode)
                // 整数模式累加
                3'b000, 3'b001: begin
                    int_result = int_acc + int_product;
                    overflow_int <= int_overflow | 
                                   ((int_acc[63] == int_product[63]) && 
                                    (int_acc[63] != int_result[63]));
                    int_acc <= int_result;
                end
                
                // BF16模式累加
                3'b100: begin
                    bf16_result = {16'b0, bf16_add(bf16_acc[15:0], bf16_product)};
                    overflow_int <= (bf16_result[14:7] == 8'hFF) || (bf16_result[14:7] == 8'h00);
                    bf16_acc <= bf16_result;
                end
                
                // 浮点模式累加
                default: begin
                    // FP32
                    fp_result = fp32_add(fp_acc, fp32_product);

                    // //FP64
                    // fp_result = fp64_add(fp_acc, fp64_product);
                    overflow_int <= (bf16_result[14:7] == 8'hFF) || (bf16_result[14:7] == 8'h00);
                    fp_acc <=  fp_result;
                end
            endcase
        end
    end
end

// Stage5: 输出阶段
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        d_out <= 0;
        overflow <= 0;
        // 更新累加器
        int_acc <= 0;
        fp_acc <= 0;
        bf16_acc <= 0;
    end else begin      
        // 输出结果
        if (pe_enable_s4) begin
            case(precision_mode)
                3'b000, 3'b001: d_out <= int_result;
                3'b100: d_out <= {32'b0, bf16_result};
                default: d_out <= fp_result;
            endcase

            overflow <= overflow_int;
        end
    end
end

// 输入数据预处理（返回符号扩展后的37bit有符号数）
function automatic signed [36:0] preprocess(input [36:0] data);
    case(precision_mode)
        3'b000:  preprocess = {data[36:32],{{28{data[3]}}, data[3:0]}};    // INT4
        3'b001:  preprocess = {data[36:32],{{24{data[7]}}, data[7:0]}};    // INT8
        3'b100:  preprocess = {data[36:32],{{16{data[15]}}, data[15:0]}};  // BF16
        default: preprocess = data;                         // FP32 INT32直接传递
    endcase
endfunction

function automatic logic [64:0] int4_mul(input logic [3:0] a_reg, input logic [3:0] b_reg);

    logic signed [63:0] a_signed, b_signed;
    logic signed [63:0] product;
    logic overflow;

    // 符号扩展
    a_signed = {{60{a_reg[3]}}, a_reg};
    b_signed = {{60{b_reg[3]}}, b_reg};

    product = a_signed * b_signed;

    // 溢出检测（以 32-bit 为限）
    overflow = (product > 64'sh7FFF_FFFF || product < -64'sh8000_0000);

    // 返回拼接值
    return {product, overflow};

endfunction

function automatic logic [64:0] int8_mul(input logic [7:0] a_reg, input logic [7:0] b_reg);

    logic signed [63:0] a_signed;
    logic signed [63:0] b_signed;
    logic signed [63:0] product;
    logic overflow;

    // 符号扩展（8-bit → 64-bit）
    a_signed = {{56{a_reg[7]}}, a_reg};
    b_signed = {{56{b_reg[7]}}, b_reg};

    product = a_signed * b_signed;

    // 溢出检测（以 32-bit 为限）
    overflow = (product > 64'sh7FFF_FFFF || product < -64'sh8000_0000);

    // 返回拼接值：高位为 product，最低位为 overflow
    return {product, overflow};

endfunction

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


function automatic [15:0] bf16_add;
    input [15:0] a, b;
    reg sign_a, sign_b;
    reg [7:0] exp_a, exp_b;
    reg [6:0] frac_a, frac_b;
        
    reg hidden_a, hidden_b;
        
    reg a_is_zero, b_is_zero;
    reg a_is_inf, b_is_inf;
    reg a_is_nan, b_is_nan;
        
    reg result_is_nan, result_is_inf, result_is_zero;
        
    reg sign_s;
    reg [7:0] exp_diff, exp_s, exp_ss;
    reg [6:0] mantissa_ss;
    reg [7:0] mantissa_a, mantissa_b;
    reg [8:0] mantissa_s;

    begin
        sign_a = a[15];
        sign_b = b[15];
        exp_a = a[14:7];
        exp_b = b[14:7];
        frac_a = a[6:0];
        frac_b = b[6:0];
            
        hidden_a = |exp_a;  // 0 为非规格数
        hidden_b = |exp_b;
            
        a_is_zero = (~hidden_a) & (~|frac_a);
        b_is_zero = (~hidden_b) & (~|frac_b);
        a_is_inf = (&exp_a) & (~|frac_a);
        b_is_inf = (&exp_b) & (~|frac_b);
        a_is_nan = (&exp_a) & (|frac_a);
        b_is_nan = (&exp_b) & (|frac_b);
            
        result_is_nan = a_is_nan | b_is_nan | (a_is_inf & b_is_zero) | (b_is_inf & a_is_zero);
        result_is_inf = (a_is_inf & ~b_is_zero) | (b_is_inf & ~a_is_zero);
        result_is_zero = a_is_zero & b_is_zero;
            
        mantissa_a = {hidden_a, frac_a};
        mantissa_b = {hidden_b, frac_b};

        if (exp_a >= exp_b) begin
            exp_diff = exp_a - exp_b;
            mantissa_b = mantissa_b >> exp_diff;
            exp_s = exp_a;
        end else begin
            exp_diff = exp_b - exp_a;
            mantissa_a = mantissa_a >> exp_diff;
            exp_s = exp_b;
        end

        if (sign_a ~^ sign_b) begin
            mantissa_s = mantissa_a + mantissa_b;
            sign_s = sign_a;
        end else begin
            if (mantissa_a > mantissa_b) begin
                mantissa_s = mantissa_a - mantissa_b;
                sign_s = sign_a;
            end else begin
                mantissa_s = mantissa_b - mantissa_a;
                sign_s = sign_b;
            end            
        end

        if (mantissa_s[8]) begin
            exp_s = exp_s + 1'b1;
            mantissa_s = mantissa_s >> 1;
        end

        case({result_is_nan, result_is_inf, result_is_zero})
            3'b100: begin  // NaN
                exp_ss = 8'hFF;
                mantissa_ss = 7'h40;
            end
            3'b010: begin  // Inf
                exp_ss = 8'hFF;
                mantissa_ss = 7'h00;
            end
            3'b001: begin  // Zero
                exp_ss = 8'h00;
                mantissa_ss = 7'h00;
            end
            default: begin  // Normal case
                exp_ss = exp_s;
                mantissa_ss = mantissa_s[6:0];
            end
        endcase
         
        bf16_add = {sign_s, exp_ss, mantissa_ss};
    end
endfunction  

function automatic [15:0] bf16_mult;
        input [15:0] a, b;
        reg sign_a, sign_b;
        reg [7:0] exp_a, exp_b;
        reg [6:0] frac_a, frac_b;
        
        reg hidden_a, hidden_b;
        
        reg a_is_zero, b_is_zero;
        reg a_is_inf, b_is_inf;
        reg a_is_nan, b_is_nan;
        
        reg result_sign;
        reg [7:0] full_mant_a, full_mant_b;
        reg [15:0] frac_mul;
        reg normalize_shift;
        reg [15:0] normalized_frac;
        reg [6:0] rounded_frac;
        reg [9:0] exp_sum;
        reg underflow, overflow;
        reg result_is_nan, result_is_inf, result_is_zero;
        reg [7:0] final_exp;
        reg [6:0] final_frac;

        begin
            sign_a = a[15];
            sign_b = b[15];
            exp_a = a[14:7];
            exp_b = b[14:7];
            frac_a = a[6:0];
            frac_b = b[6:0];
            
            hidden_a = |exp_a;  // 0 为非规格数
            hidden_b = |exp_b;
            
            a_is_zero = (~hidden_a) & (~|frac_a);
            b_is_zero = (~hidden_b) & (~|frac_b);
            a_is_inf = (&exp_a) & (~|frac_a);
            b_is_inf = (&exp_b) & (~|frac_b);
            a_is_nan = (&exp_a) & (|frac_a);
            b_is_nan = (&exp_b) & (|frac_b);
            
            result_sign = sign_a ^ sign_b;
            
            full_mant_a = {hidden_a, frac_a};
            full_mant_b = {hidden_b, frac_b};
            frac_mul = full_mant_a * full_mant_b;  // 8x8 -> 16 bits
            
            normalize_shift = frac_mul[15];
            normalized_frac = normalize_shift ? frac_mul : (frac_mul << 1);
            
            rounded_frac = normalized_frac[14:8] + 
                          (normalized_frac[7] & (|normalized_frac[6:0] | normalized_frac[8]));
            
            exp_sum = {2'b0, exp_a} + {2'b0, exp_b} - 10'd127 + normalize_shift;
            underflow = exp_sum[9] | (~|exp_sum[8:0]);  // exp <= 0
            overflow = ~(exp_sum[9]) & (exp_sum[8] | (&exp_sum[7:0]));  // exp >= 255
            
            result_is_nan = a_is_nan | b_is_nan | (a_is_inf & b_is_zero) | (b_is_inf & a_is_zero);
            result_is_inf = overflow | (a_is_inf & ~b_is_zero) | (b_is_inf & ~a_is_zero);
            result_is_zero = underflow | a_is_zero | b_is_zero;
            
            case ({result_is_nan, result_is_inf, result_is_zero})
                3'b100: begin  // NaN
                    final_exp = 8'hFF;
                    final_frac = 7'h40;
                end
                3'b010: begin  // Inf
                    final_exp = 8'hFF;
                    final_frac = 7'h00;
                end
                3'b001: begin  // Zero
                    final_exp = 8'h00;
                    final_frac = 7'h00;
                end
                default: begin  // Normal case
                    final_exp = exp_sum[7:0];
                    final_frac = rounded_frac;
                end
            endcase
            
            bf16_mult = {result_sign, final_exp, final_frac};
        end
endfunction

endmodule
