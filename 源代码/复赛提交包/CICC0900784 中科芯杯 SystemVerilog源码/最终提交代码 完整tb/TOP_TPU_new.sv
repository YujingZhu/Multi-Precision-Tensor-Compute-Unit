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

(*use_dsp ="no" *) module TOP_TPU #(
  parameter C_S_AXI_ADDR_WIDTH = 32,
  parameter C_S_AXI_DATA_WIDTH = 32,
  parameter C_M_AXI_DATA_WIDTH = 64,
  parameter C_M_AXI_ADDR_WIDTH = 32,
  parameter APB_DATA_WIDTH = 9,

  parameter MAX_M = 32,
  parameter MAX_N = 32,
  parameter MAX_K = 16,
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
  input wire [APB_DATA_WIDTH-1:0]     apb_pwdata,

  //以下为测试监控信号
  // master 监控
  output wire  [63:0]  d_out,
  
  // fsm 监控
  output [11:0] cycle_counter,
  output        load_c_en,
  output        load_a_en,
  output        load_b_en,
  output        out_en,
  output        done,
  output [3:0]  current_state,
  output        pe_enable,
  output        pe_load_c_en,
  output reg [6:0]  compute_counter,//信号位宽增一位
  output reg [2:0]  pe_counter,//新增PE计数器
  output [31:0] s_addr,
  output [11:0] beat_cnt//修改位数
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

// // FSM控制信号
//   wire [11:0]  cycle_counter;
//   wire         load_c_en;
//   wire         load_a_en;
//   wire         load_b_en;
//   wire         out_en;
//   wire         done;
//   wire [3:0]   current_state;   // 当前状态
//   wire         pe_enable;      // PE使能
//   wire         pe_load_c_en;   // PE加载C使能
//   wire [6:0]   compute_counter; // 计算计数器
//   wire [31:0]  s_addr;           // 地址总线
//   wire [15:0]  beat_cnt;                            
  
//   // master 信号
//   wire  [63:0]  d_out;           

(* ram_style = "block" *) reg [31:0] a_data00 [0:MAX_K-1];  
(* ram_style = "block" *) reg [31:0] a_data01 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data02 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data03 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data04 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data05 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data06 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data07 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data08 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data09 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data10 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data11 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data12 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data13 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data14 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data15 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data16 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data17 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data18 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data19 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data20 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data21 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data22 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data23 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data24 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data25 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data26 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data27 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data28 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data29 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data30 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] a_data31 [0:MAX_K-1];

(* ram_style = "block" *) reg [31:0] b_data00 [0:MAX_K-1];  
(* ram_style = "block" *) reg [31:0] b_data01 [0:MAX_K-1];  
(* ram_style = "block" *) reg [31:0] b_data02 [0:MAX_K-1];  
(* ram_style = "block" *) reg [31:0] b_data03 [0:MAX_K-1];  
(* ram_style = "block" *) reg [31:0] b_data04 [0:MAX_K-1];  
(* ram_style = "block" *) reg [31:0] b_data05 [0:MAX_K-1]; 
(* ram_style = "block" *) reg [31:0] b_data06 [0:MAX_K-1]; 
(* ram_style = "block" *) reg [31:0] b_data07 [0:MAX_K-1];  
(* ram_style = "block" *) reg [31:0] b_data08 [0:MAX_K-1];  
(* ram_style = "block" *) reg [31:0] b_data09 [0:MAX_K-1]; 
(* ram_style = "block" *) reg [31:0] b_data10 [0:MAX_K-1]; 
(* ram_style = "block" *) reg [31:0] b_data11 [0:MAX_K-1];  
(* ram_style = "block" *) reg [31:0] b_data12 [0:MAX_K-1];  
(* ram_style = "block" *) reg [31:0] b_data13 [0:MAX_K-1];  
(* ram_style = "block" *) reg [31:0] b_data14 [0:MAX_K-1];  
(* ram_style = "block" *) reg [31:0] b_data15 [0:MAX_K-1];  
(* ram_style = "block" *) reg [31:0] b_data16 [0:MAX_K-1]; 
(* ram_style = "block" *) reg [31:0] b_data17 [0:MAX_K-1];  
(* ram_style = "block" *) reg [31:0] b_data18 [0:MAX_K-1]; 
(* ram_style = "block" *) reg [31:0] b_data19 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] b_data20 [0:MAX_K-1]; 
(* ram_style = "block" *) reg [31:0] b_data21 [0:MAX_K-1];  
(* ram_style = "block" *) reg [31:0] b_data22 [0:MAX_K-1];  
(* ram_style = "block" *) reg [31:0] b_data23 [0:MAX_K-1];
(* ram_style = "block" *) reg [31:0] b_data24 [0:MAX_K-1]; 
(* ram_style = "block" *) reg [31:0] b_data25 [0:MAX_K-1]; 
(* ram_style = "block" *) reg [31:0] b_data26 [0:MAX_K-1];  
(* ram_style = "block" *) reg [31:0] b_data27 [0:MAX_K-1]; 
(* ram_style = "block" *) reg [31:0] b_data28 [0:MAX_K-1];  
(* ram_style = "block" *) reg [31:0] b_data29 [0:MAX_K-1]; 
(* ram_style = "block" *) reg [31:0] b_data30 [0:MAX_K-1]; 
(* ram_style = "block" *) reg [31:0] b_data31 [0:MAX_K-1]; 

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
    .MAX_M(MAX_M),
    .MAX_N(MAX_N),
    .MAX_K(MAX_K),
    .C_ROW_MAX(C_ROW_MAX),
    .C_COL_MAX(C_COL_MAX)
  ) u_data_load (
    .clk(clk),
    .rst_n(setn),
    .precision_mode(precision_mode),
    // .mixed_mode(mixed_mode),
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
        // .elements_per_word(elements_per_word),
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
        .MAX_M(MAX_M),
        .MAX_N(MAX_N),
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
        // .m(m),
        // .n(n),
        // .k(k),
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
        // .m(m),
        // .n(n),
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
parameter MAX_M = 32,
parameter MAX_N = 32,
parameter MAX_K = 16,
parameter C_ROW_MAX = 16,
parameter C_COL_MAX = 16
)(    
    input              clk,
    input              rst_n,
    // 配置接口
    input  [2:0]       precision_mode,
    // input              mixed_mode,
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
    output reg [31:0] a_data00 [0:MAX_K-1],
    output reg [31:0] a_data01 [0:MAX_K-1],
    output reg [31:0] a_data02 [0:MAX_K-1],
    output reg [31:0] a_data03 [0:MAX_K-1],
    output reg [31:0] a_data04 [0:MAX_K-1],
    output reg [31:0] a_data05 [0:MAX_K-1],
    output reg [31:0] a_data06 [0:MAX_K-1],
    output reg [31:0] a_data07 [0:MAX_K-1],
    output reg [31:0] a_data08 [0:MAX_K-1],
    output reg [31:0] a_data09 [0:MAX_K-1],
    output reg [31:0] a_data10 [0:MAX_K-1],
    output reg [31:0] a_data11 [0:MAX_K-1],
    output reg [31:0] a_data12 [0:MAX_K-1],
    output reg [31:0] a_data13 [0:MAX_K-1],
    output reg [31:0] a_data14 [0:MAX_K-1],
    output reg [31:0] a_data15 [0:MAX_K-1],
    output reg [31:0] a_data16 [0:MAX_K-1],
    output reg [31:0] a_data17 [0:MAX_K-1],
    output reg [31:0] a_data18 [0:MAX_K-1],
    output reg [31:0] a_data19 [0:MAX_K-1],
    output reg [31:0] a_data20 [0:MAX_K-1],
    output reg [31:0] a_data21 [0:MAX_K-1],
    output reg [31:0] a_data22 [0:MAX_K-1],
    output reg [31:0] a_data23 [0:MAX_K-1],
    output reg [31:0] a_data24 [0:MAX_K-1],
    output reg [31:0] a_data25 [0:MAX_K-1],
    output reg [31:0] a_data26 [0:MAX_K-1],
    output reg [31:0] a_data27 [0:MAX_K-1],
    output reg [31:0] a_data28 [0:MAX_K-1],
    output reg [31:0] a_data29 [0:MAX_K-1],
    output reg [31:0] a_data30 [0:MAX_K-1],
    output reg [31:0] a_data31 [0:MAX_K-1],
    
    output reg [31:0] b_data00 [0:MAX_K-1],
    output reg [31:0] b_data01 [0:MAX_K-1],
    output reg [31:0] b_data02 [0:MAX_K-1],
    output reg [31:0] b_data03 [0:MAX_K-1],
    output reg [31:0] b_data04 [0:MAX_K-1],
    output reg [31:0] b_data05 [0:MAX_K-1],
    output reg [31:0] b_data06 [0:MAX_K-1],
    output reg [31:0] b_data07 [0:MAX_K-1],
    output reg [31:0] b_data08 [0:MAX_K-1],
    output reg [31:0] b_data09 [0:MAX_K-1],
    output reg [31:0] b_data10 [0:MAX_K-1],
    output reg [31:0] b_data11 [0:MAX_K-1],
    output reg [31:0] b_data12 [0:MAX_K-1],
    output reg [31:0] b_data13 [0:MAX_K-1],
    output reg [31:0] b_data14 [0:MAX_K-1],
    output reg [31:0] b_data15 [0:MAX_K-1],
    output reg [31:0] b_data16 [0:MAX_K-1],
    output reg [31:0] b_data17 [0:MAX_K-1],
    output reg [31:0] b_data18 [0:MAX_K-1],
    output reg [31:0] b_data19 [0:MAX_K-1],
    output reg [31:0] b_data20 [0:MAX_K-1],
    output reg [31:0] b_data21 [0:MAX_K-1],
    output reg [31:0] b_data22 [0:MAX_K-1],
    output reg [31:0] b_data23 [0:MAX_K-1],
    output reg [31:0] b_data24 [0:MAX_K-1],
    output reg [31:0] b_data25 [0:MAX_K-1],
    output reg [31:0] b_data26 [0:MAX_K-1],
    output reg [31:0] b_data27 [0:MAX_K-1],
    output reg [31:0] b_data28 [0:MAX_K-1],
    output reg [31:0] b_data29 [0:MAX_K-1],
    output reg [31:0] b_data30 [0:MAX_K-1],
    output reg [31:0] b_data31 [0:MAX_K-1],

    output reg [31:0] c_data [0:C_ROW_MAX-1][0:C_COL_MAX-1]

);   

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

            3'b000: elements_per_word = 8;//4bits 32位每个时钟周期处理8个数 int4
            3'b001: elements_per_word = 4;//8bits 32位每个时钟周期处理4个数 int8
            3'b010: elements_per_word = 2;//16bits 32位每个时钟周期处理2个数 fp16
            3'b011: elements_per_word = 1;//32bits 32位每个时钟周期处理1个数 fp32

            3'b100: elements_per_word = 2;//16bits 32位每个时钟周期处理2个数 bf16
            3'b101: elements_per_word = 1;//32bits 32位每个时钟周期处理1个数 int32

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
        else if (load_a_en && s_axi_bready && (cycle_counter < m*k)) begin
                    automatic logic[4:0] row = cycle_counter / k;
                    automatic logic[4:0] col = cycle_counter % k;
                    case(row)
                        5'b00000: a_data00[col] <= safe_bit_select(a_row, precision_mode);
                        5'b00001: a_data01[col] <= safe_bit_select(a_row, precision_mode);
                        5'b00010: a_data02[col] <= safe_bit_select(a_row, precision_mode);
                        5'b00011: a_data03[col] <= safe_bit_select(a_row, precision_mode);
                        5'b00100: a_data04[col] <= safe_bit_select(a_row, precision_mode);
                        5'b00101: a_data05[col] <= safe_bit_select(a_row, precision_mode);
                        5'b00110: a_data06[col] <= safe_bit_select(a_row, precision_mode);
                        5'b00111: a_data07[col] <= safe_bit_select(a_row, precision_mode);
                        5'b01000: a_data08[col] <= safe_bit_select(a_row, precision_mode);
                        5'b01001: a_data09[col] <= safe_bit_select(a_row, precision_mode);
                        5'b01010: a_data10[col] <= safe_bit_select(a_row, precision_mode);
                        5'b01011: a_data11[col] <= safe_bit_select(a_row, precision_mode);
                        5'b01100: a_data12[col] <= safe_bit_select(a_row, precision_mode);
                        5'b01101: a_data13[col] <= safe_bit_select(a_row, precision_mode);
                        5'b01110: a_data14[col] <= safe_bit_select(a_row, precision_mode);
                        5'b01111: a_data15[col] <= safe_bit_select(a_row, precision_mode);
                        5'b10000: a_data16[col] <= safe_bit_select(a_row, precision_mode);
                        5'b10001: a_data17[col] <= safe_bit_select(a_row, precision_mode);
                        5'b10010: a_data18[col] <= safe_bit_select(a_row, precision_mode);
                        5'b10011: a_data19[col] <= safe_bit_select(a_row, precision_mode);
                        5'b10100: a_data20[col] <= safe_bit_select(a_row, precision_mode);
                        5'b10101: a_data21[col] <= safe_bit_select(a_row, precision_mode);
                        5'b10110: a_data22[col] <= safe_bit_select(a_row, precision_mode);
                        5'b10111: a_data23[col] <= safe_bit_select(a_row, precision_mode);
                        5'b11000: a_data24[col] <= safe_bit_select(a_row, precision_mode);
                        5'b11001: a_data25[col] <= safe_bit_select(a_row, precision_mode);
                        5'b11010: a_data26[col] <= safe_bit_select(a_row, precision_mode);
                        5'b11011: a_data27[col] <= safe_bit_select(a_row, precision_mode);
                        5'b11100: a_data28[col] <= safe_bit_select(a_row, precision_mode);
                        5'b11101: a_data29[col] <= safe_bit_select(a_row, precision_mode);
                        5'b11110: a_data30[col] <= safe_bit_select(a_row, precision_mode);
                        5'b11111: a_data31[col] <= safe_bit_select(a_row, precision_mode);
                        default:;
                    endcase    
                end
        end

        // B矩阵加载
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin  // 异步复位，低电平有效
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
        else if (load_b_en && s_axi_bready && (cycle_counter < k*n)) begin  // 正常加载逻辑
                    automatic logic[4:0] row = cycle_counter / n;
                    automatic logic[4:0] col = cycle_counter % n;
                    case(col)
                        5'b00000: b_data00[row] <= safe_bit_select(b_col, precision_mode);
                        5'b00001: b_data01[row] <= safe_bit_select(b_col, precision_mode);
                        5'b00010: b_data02[row] <= safe_bit_select(b_col, precision_mode);
                        5'b00011: b_data03[row] <= safe_bit_select(b_col, precision_mode);
                        5'b00100: b_data04[row] <= safe_bit_select(b_col, precision_mode);
                        5'b00101: b_data05[row] <= safe_bit_select(b_col, precision_mode);
                        5'b00110: b_data06[row] <= safe_bit_select(b_col, precision_mode);
                        5'b00111: b_data07[row] <= safe_bit_select(b_col, precision_mode);
                        5'b01000: b_data08[row] <= safe_bit_select(b_col, precision_mode);
                        5'b01001: b_data09[row] <= safe_bit_select(b_col, precision_mode);
                        5'b01010: b_data10[row] <= safe_bit_select(b_col, precision_mode);
                        5'b01011: b_data11[row] <= safe_bit_select(b_col, precision_mode);
                        5'b01100: b_data12[row] <= safe_bit_select(b_col, precision_mode);
                        5'b01101: b_data13[row] <= safe_bit_select(b_col, precision_mode);
                        5'b01110: b_data14[row] <= safe_bit_select(b_col, precision_mode);
                        5'b01111: b_data15[row] <= safe_bit_select(b_col, precision_mode);
                        5'b10000: b_data16[row] <= safe_bit_select(b_col, precision_mode);
                        5'b10001: b_data17[row] <= safe_bit_select(b_col, precision_mode);
                        5'b10010: b_data18[row] <= safe_bit_select(b_col, precision_mode);
                        5'b10011: b_data19[row] <= safe_bit_select(b_col, precision_mode);
                        5'b10100: b_data20[row] <= safe_bit_select(b_col, precision_mode);
                        5'b10101: b_data21[row] <= safe_bit_select(b_col, precision_mode);
                        5'b10110: b_data22[row] <= safe_bit_select(b_col, precision_mode);
                        5'b10111: b_data23[row] <= safe_bit_select(b_col, precision_mode);
                        5'b11000: b_data24[row] <= safe_bit_select(b_col, precision_mode);
                        5'b11001: b_data25[row] <= safe_bit_select(b_col, precision_mode);
                        5'b11010: b_data26[row] <= safe_bit_select(b_col, precision_mode);
                        5'b11011: b_data27[row] <= safe_bit_select(b_col, precision_mode);
                        5'b11100: b_data28[row] <= safe_bit_select(b_col, precision_mode);
                        5'b11101: b_data29[row] <= safe_bit_select(b_col, precision_mode);
                        5'b11110: b_data30[row] <= safe_bit_select(b_col, precision_mode);
                        5'b11111: b_data31[row] <= safe_bit_select(b_col, precision_mode);
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
                if (compute_counter <= k + PE_ROW_MAX + PE_COL_MAX ) begin//修改！！！脉动行、列位置索引计数器！！！
                    compute_counter <= compute_counter + 1;
                    pe_enable <= 1;
                end else begin
                    compute_counter <= compute_counter;//加else，分块循环索引，循环四次
                end                    

                //每个分块矩阵计算完都输出再进行下一次计算下一个分块！！一直到计算完所有分块！！
                //5级流水线 + 4 
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
   parameter MAX_M = 32,
   parameter MAX_N = 32,
   parameter MAX_K = 16,
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
    // input  wire [5:0]  m, n, k,
    input  wire [3:0]  state,
    input wire  [11:0] cycle_counter,   // 改为12-bit输入
    input wire  [6:0]  compute_counter, 
    input wire  [2:0]  pe_counter,
    input  wire        pe_enable,
    input  wire        pe_load_c_en,

    //由储存数据传输过来，不用重新定义！！
    // 矩阵选择信号
    input reg [31:0] a_data00 [0:MAX_K-1],
    input reg [31:0] a_data01 [0:MAX_K-1],
    input reg [31:0] a_data02 [0:MAX_K-1],
    input reg [31:0] a_data03 [0:MAX_K-1],
    input reg [31:0] a_data04 [0:MAX_K-1],
    input reg [31:0] a_data05 [0:MAX_K-1],
    input reg [31:0] a_data06 [0:MAX_K-1],
    input reg [31:0] a_data07 [0:MAX_K-1],
    input reg [31:0] a_data08 [0:MAX_K-1],
    input reg [31:0] a_data09 [0:MAX_K-1],
    input reg [31:0] a_data10 [0:MAX_K-1],
    input reg [31:0] a_data11 [0:MAX_K-1],
    input reg [31:0] a_data12 [0:MAX_K-1],
    input reg [31:0] a_data13 [0:MAX_K-1],
    input reg [31:0] a_data14 [0:MAX_K-1],
    input reg [31:0] a_data15 [0:MAX_K-1],
    input reg [31:0] a_data16 [0:MAX_K-1],
    input reg [31:0] a_data17 [0:MAX_K-1],
    input reg [31:0] a_data18 [0:MAX_K-1],
    input reg [31:0] a_data19 [0:MAX_K-1],
    input reg [31:0] a_data20 [0:MAX_K-1],
    input reg [31:0] a_data21 [0:MAX_K-1],
    input reg [31:0] a_data22 [0:MAX_K-1],
    input reg [31:0] a_data23 [0:MAX_K-1],
    input reg [31:0] a_data24 [0:MAX_K-1],
    input reg [31:0] a_data25 [0:MAX_K-1],
    input reg [31:0] a_data26 [0:MAX_K-1],
    input reg [31:0] a_data27 [0:MAX_K-1],
    input reg [31:0] a_data28 [0:MAX_K-1],
    input reg [31:0] a_data29 [0:MAX_K-1],
    input reg [31:0] a_data30 [0:MAX_K-1],
    input reg [31:0] a_data31 [0:MAX_K-1],
    
    input reg [31:0] b_data00 [0:MAX_K-1],
    input reg [31:0] b_data01 [0:MAX_K-1],
    input reg [31:0] b_data02 [0:MAX_K-1],
    input reg [31:0] b_data03 [0:MAX_K-1],
    input reg [31:0] b_data04 [0:MAX_K-1],
    input reg [31:0] b_data05 [0:MAX_K-1],
    input reg [31:0] b_data06 [0:MAX_K-1],
    input reg [31:0] b_data07 [0:MAX_K-1],
    input reg [31:0] b_data08 [0:MAX_K-1],
    input reg [31:0] b_data09 [0:MAX_K-1],
    input reg [31:0] b_data10 [0:MAX_K-1],
    input reg [31:0] b_data11 [0:MAX_K-1],
    input reg [31:0] b_data12 [0:MAX_K-1],
    input reg [31:0] b_data13 [0:MAX_K-1],
    input reg [31:0] b_data14 [0:MAX_K-1],
    input reg [31:0] b_data15 [0:MAX_K-1],
    input reg [31:0] b_data16 [0:MAX_K-1],
    input reg [31:0] b_data17 [0:MAX_K-1],
    input reg [31:0] b_data18 [0:MAX_K-1],
    input reg [31:0] b_data19 [0:MAX_K-1],
    input reg [31:0] b_data20 [0:MAX_K-1],
    input reg [31:0] b_data21 [0:MAX_K-1],
    input reg [31:0] b_data22 [0:MAX_K-1],
    input reg [31:0] b_data23 [0:MAX_K-1],
    input reg [31:0] b_data24 [0:MAX_K-1],
    input reg [31:0] b_data25 [0:MAX_K-1],
    input reg [31:0] b_data26 [0:MAX_K-1],
    input reg [31:0] b_data27 [0:MAX_K-1],
    input reg [31:0] b_data28 [0:MAX_K-1],
    input reg [31:0] b_data29 [0:MAX_K-1],
    input reg [31:0] b_data30 [0:MAX_K-1],
    input reg [31:0] b_data31 [0:MAX_K-1],

    input reg [31:0] c_data [0:C_ROW_MAX-1][0:C_COL_MAX-1]

);
    
    wire [31:0] a_bus [1:PE_ROW_MAX][0:PE_COL_MAX];
    wire [31:0] b_bus [0:PE_ROW_MAX][1:PE_COL_MAX];
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

    //8×8 PE阵列
    assign a_bus[1][0]  =  ( a_bus_index == 0 ) ? (((compute_counter-1) >= 0)  ? a_data00 [(compute_counter-1)-0]  : 32'd0 ) :
                           ( a_bus_index == 1 ) ? (((compute_counter-1) >= 0)  ? a_data08 [(compute_counter-1)-0]  : 32'd0 ) :
                           ( a_bus_index == 2 ) ? (((compute_counter-1) >= 0)  ? a_data16 [(compute_counter-1)-0]  : 32'd0 ) :
                           ( a_bus_index == 3 ) ? (((compute_counter-1) >= 0)  ? a_data24 [(compute_counter-1)-0]  : 32'd0 ) :
                           32'd0 ;

    assign a_bus[2][0]  =  ( a_bus_index == 0 ) ? (((compute_counter-1) >= 1)  ? a_data01 [(compute_counter-1)-1]  : 32'd0 ) :
                           ( a_bus_index == 1 ) ? (((compute_counter-1) >= 1)  ? a_data09 [(compute_counter-1)-1]  : 32'd0 ) :
                           ( a_bus_index == 2 ) ? (((compute_counter-1) >= 1)  ? a_data17 [(compute_counter-1)-1]  : 32'd0 ) :
                           ( a_bus_index == 3 ) ? (((compute_counter-1) >= 1)  ? a_data25 [(compute_counter-1)-1]  : 32'd0 ) :
                           32'd0 ;

    assign a_bus[3][0]  =  ( a_bus_index == 0 ) ? (((compute_counter-1) >= 2)  ? a_data02 [(compute_counter-1)-2]  : 32'd0 ) :
                           ( a_bus_index == 1 ) ? (((compute_counter-1) >= 2)  ? a_data10 [(compute_counter-1)-2]  : 32'd0 ) :
                           ( a_bus_index == 2 ) ? (((compute_counter-1) >= 2)  ? a_data18 [(compute_counter-1)-2]  : 32'd0 ) :
                           ( a_bus_index == 3 ) ? (((compute_counter-1) >= 2)  ? a_data26 [(compute_counter-1)-2]  : 32'd0 ) :
                            32'd0 ;

    assign a_bus[4][0]  =  ( a_bus_index == 0 ) ? (((compute_counter-1) >= 3)  ? a_data03 [(compute_counter-1)-3]  : 32'd0 ) :
                           ( a_bus_index == 1 ) ? (((compute_counter-1) >= 3)  ? a_data11 [(compute_counter-1)-3]  : 32'd0 ) :
                           ( a_bus_index == 2 ) ? (((compute_counter-1) >= 3)  ? a_data19 [(compute_counter-1)-3]  : 32'd0 ) :
                           ( a_bus_index == 3 ) ? (((compute_counter-1) >= 3)  ? a_data27 [(compute_counter-1)-3]  : 32'd0 ) :
                            32'd0 ;                            

    assign a_bus[5][0]  =  ( a_bus_index == 0 ) ? (((compute_counter-1) >= 4)  ? a_data04 [(compute_counter-1)-4]  : 32'd0 ) :
                           ( a_bus_index == 1 ) ? (((compute_counter-1) >= 4)  ? a_data12 [(compute_counter-1)-4]  : 32'd0 ) :
                           ( a_bus_index == 2 ) ? (((compute_counter-1) >= 4)  ? a_data20 [(compute_counter-1)-4]  : 32'd0 ) :
                           ( a_bus_index == 3 ) ? (((compute_counter-1) >= 4)  ? a_data28 [(compute_counter-1)-4]  : 32'd0 ) :
                            32'd0 ;

    assign a_bus[6][0]  =  ( a_bus_index == 0 ) ? (((compute_counter-1) >= 5)  ? a_data05 [(compute_counter-1)-5]  : 32'd0 ) :
                           ( a_bus_index == 1 ) ? (((compute_counter-1) >= 5)  ? a_data13 [(compute_counter-1)-5]  : 32'd0 ) :
                           ( a_bus_index == 2 ) ? (((compute_counter-1) >= 5)  ? a_data21 [(compute_counter-1)-5]  : 32'd0 ) :
                           ( a_bus_index == 3 ) ? (((compute_counter-1) >= 5)  ? a_data29 [(compute_counter-1)-5]  : 32'd0 ) :
                            32'd0 ; 

    assign a_bus[7][0]  =  ( a_bus_index == 0 ) ? (((compute_counter-1) >= 6)  ? a_data06 [(compute_counter-1)-6]  : 32'd0 ) :
                           ( a_bus_index == 1 ) ? (((compute_counter-1) >= 6)  ? a_data14 [(compute_counter-1)-6]  : 32'd0 ) :
                           ( a_bus_index == 2 ) ? (((compute_counter-1) >= 6)  ? a_data22 [(compute_counter-1)-6]  : 32'd0 ) :
                           ( a_bus_index == 3 ) ? (((compute_counter-1) >= 6)  ? a_data30 [(compute_counter-1)-6]  : 32'd0 ) :
                            32'd0 ; 

    assign a_bus[8][0]  =  ( a_bus_index == 0 ) ? (((compute_counter-1) >= 7)  ? a_data07 [(compute_counter-1)-7]  : 32'd0 ) :
                           ( a_bus_index == 1 ) ? (((compute_counter-1) >= 7)  ? a_data15 [(compute_counter-1)-7]  : 32'd0 ) :
                           ( a_bus_index == 2 ) ? (((compute_counter-1) >= 7)  ? a_data23 [(compute_counter-1)-7]  : 32'd0 ) :
                           ( a_bus_index == 3 ) ? (((compute_counter-1) >= 7)  ? a_data31 [(compute_counter-1)-7]  : 32'd0 ) :
                            32'd0 ; 

    //8×8 PE阵列
    assign b_bus[0][1]  =  ( b_bus_index == 0 ) ? (((compute_counter-1) >= 0)  ? b_data00 [(compute_counter-1)-0]  : 32'd0 ) :
                           ( b_bus_index == 1 ) ? (((compute_counter-1) >= 0)  ? b_data08 [(compute_counter-1)-0]  : 32'd0 ) :
                           ( b_bus_index == 2 ) ? (((compute_counter-1) >= 0)  ? b_data16 [(compute_counter-1)-0]  : 32'd0 ) :
                           ( b_bus_index == 3 ) ? (((compute_counter-1) >= 0)  ? b_data24 [(compute_counter-1)-0]  : 32'd0 ) :
                           32'd0 ;

    assign b_bus[0][2]  =  ( b_bus_index == 0 ) ? (((compute_counter-1) >= 1)  ? b_data01 [(compute_counter-1)-1]  : 32'd0 ) :
                           ( b_bus_index == 1 ) ? (((compute_counter-1) >= 1)  ? b_data09 [(compute_counter-1)-1]  : 32'd0 ) :
                           ( b_bus_index == 2 ) ? (((compute_counter-1) >= 1)  ? b_data17 [(compute_counter-1)-1]  : 32'd0 ) :
                           ( b_bus_index == 3 ) ? (((compute_counter-1) >= 1)  ? b_data25 [(compute_counter-1)-1]  : 32'd0 ) :
                           32'd0 ;

    assign b_bus[0][3]  =  ( b_bus_index == 0 ) ? (((compute_counter-1) >= 2)  ? b_data02 [(compute_counter-1)-2]  : 32'd0 ) :
                           ( b_bus_index == 1 ) ? (((compute_counter-1) >= 2)  ? b_data10 [(compute_counter-1)-2]  : 32'd0 ) :
                           ( b_bus_index == 2 ) ? (((compute_counter-1) >= 2)  ? b_data18 [(compute_counter-1)-2]  : 32'd0 ) :
                           ( b_bus_index == 3 ) ? (((compute_counter-1) >= 2)  ? b_data26 [(compute_counter-1)-2]  : 32'd0 ) :
                            32'd0 ;

    assign b_bus[0][4]  =  ( b_bus_index == 0 ) ? (((compute_counter-1) >= 3)  ? b_data03 [(compute_counter-1)-3]  : 32'd0 ) :
                           ( b_bus_index == 1 ) ? (((compute_counter-1) >= 3)  ? b_data11 [(compute_counter-1)-3]  : 32'd0 ) :
                           ( b_bus_index == 2 ) ? (((compute_counter-1) >= 3)  ? b_data19 [(compute_counter-1)-3]  : 32'd0 ) :
                           ( b_bus_index == 3 ) ? (((compute_counter-1) >= 3)  ? b_data27 [(compute_counter-1)-3]  : 32'd0 ) :
                            32'd0 ;

    assign b_bus[0][5]  =  ( b_bus_index == 0 ) ? (((compute_counter-1) >= 4)  ? b_data04 [(compute_counter-1)-4]  : 32'd0 ) :
                           ( b_bus_index == 1 ) ? (((compute_counter-1) >= 4)  ? b_data12 [(compute_counter-1)-4]  : 32'd0 ) :
                           ( b_bus_index == 2 ) ? (((compute_counter-1) >= 4)  ? b_data20 [(compute_counter-1)-4]  : 32'd0 ) :
                           ( b_bus_index == 3 ) ? (((compute_counter-1) >= 4)  ? b_data28 [(compute_counter-1)-4]  : 32'd0 ) :
                            32'd0 ;

    assign b_bus[0][6]  =  ( b_bus_index == 0 ) ? (((compute_counter-1) >= 5)  ? b_data05 [(compute_counter-1)-5]  : 32'd0 ) :
                           ( b_bus_index == 1 ) ? (((compute_counter-1) >= 5)  ? b_data13 [(compute_counter-1)-5]  : 32'd0 ) :
                           ( b_bus_index == 2 ) ? (((compute_counter-1) >= 5)  ? b_data21 [(compute_counter-1)-5]  : 32'd0 ) :
                           ( b_bus_index == 3 ) ? (((compute_counter-1) >= 5)  ? b_data29 [(compute_counter-1)-5]  : 32'd0 ) :
                            32'd0 ;

    assign b_bus[0][7]  =  ( b_bus_index == 0 ) ? (((compute_counter-1) >= 6)  ? b_data06 [(compute_counter-1)-6]  : 32'd0 ) :
                           ( b_bus_index == 1 ) ? (((compute_counter-1) >= 6)  ? b_data14 [(compute_counter-1)-6]  : 32'd0 ) :
                           ( b_bus_index == 2 ) ? (((compute_counter-1) >= 6)  ? b_data22 [(compute_counter-1)-6]  : 32'd0 ) :
                           ( b_bus_index == 3 ) ? (((compute_counter-1) >= 6)  ? b_data30 [(compute_counter-1)-6]  : 32'd0 ) :
                            32'd0 ; 

    assign b_bus[0][8]  =  ( b_bus_index == 0 ) ? (((compute_counter-1) >= 7)  ? b_data07 [(compute_counter-1)-7]  : 32'd0 ) :
                           ( b_bus_index == 1 ) ? (((compute_counter-1) >= 7)  ? b_data15 [(compute_counter-1)-7]  : 32'd0 ) :
                           ( b_bus_index == 2 ) ? (((compute_counter-1) >= 7)  ? b_data23 [(compute_counter-1)-7]  : 32'd0 ) :
                           ( b_bus_index == 3 ) ? (((compute_counter-1) >= 7)  ? b_data31 [(compute_counter-1)-7]  : 32'd0 ) :
                            32'd0 ;


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
                .mixed_mode(mixed_mode),//现在还没有多精度计算，等用到了再把注释删掉 ~.~
                .pe_load_c_en(pe_load_c_en),
                .a_in(a_bus[i][j-1]),   // 从左获取A数据！！！修改，原来的流动方向错了
                .b_in(b_bus[i-1][j]),   // 从上获取B数据 ！！！修改，原来的流动方向错了
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

// FP16转FP32函数
function automatic [31:0] fp16_to_fp32;
    input  [15:0] fp16; 

    reg        sign;
    reg [4:0]  exp5;
    reg [9:0]  frac10;

    reg        is_zero;
    reg        is_denorm;
    reg        is_inf_nan;

    reg [7:0]  exp8;
    reg [22:0] frac23;

    integer    leading_zeros;
    reg [23:0] denorm_shift;

    begin
        sign     = fp16[15];
        exp5     = fp16[14:10];
        frac10   = fp16[9:0];

        is_zero    = (exp5 == 0) && (frac10 == 0);
        is_denorm  = (exp5 == 0) && (frac10 != 0);
        is_inf_nan = (exp5 == 5'b11111);

        exp8   = 8'h00;
        frac23 = 23'h0;

        if (is_inf_nan) begin
            exp8   = 8'hFF;
            frac23 = (frac10 == 0) ? 23'h0 : {1'b1, frac10, 12'h0};
        end
        else if (is_zero) begin
            exp8   = 8'h00;
            frac23 = 23'h0;
        end
        else if (is_denorm) begin
            leading_zeros = 10;
            for (integer i = 9; i >= 0; i = i - 1) begin
                if (frac10[i]) begin
                    leading_zeros = 9 - i;
                    break;
                end
            end
            denorm_shift = {1'b0, frac10, 13'h0} << (leading_zeros + 1);
            frac23       = denorm_shift[23:1];
            exp8 = 8'd127 - 8'd15 - leading_zeros;
        end
        else begin
            exp8   = exp5 + 8'd112;
            frac23 = {frac10, 13'h0};
        end

        fp16_to_fp32 = {sign, exp8, frac23};
    end
endfunction

// 符号扩展函数
function automatic [31:0] sign_extend4(input [3:0] data);
    return {{28{data[3]}}, data};
endfunction

function automatic [31:0] sign_extend8(input [7:0] data);
    return {{24{data[7]}}, data};
endfunction

endmodule


(*use_dsp ="no" *)module ProcessingElement (
    input              clk,           
    input              rst_n,          
    input              pe_enable,
    input  [2:0]       precision_mode, 
    input              mixed_mode,
    input              pe_load_c_en,      
    input  [31:0]      a_in,          
    input  [31:0]      b_in,           
    input  [31:0]      c_in,       
    output reg [31:0]  a_out,         
    output reg [31:0]  b_out,         
    output reg  [63:0] d_out,          
    output reg         overflow
);

// 累加器寄存器
reg signed [63:0]  int_acc;     // 整数累加器
reg [63:0]         fp_acc;      // 浮点累加器
reg [31:0]         bf16_acc;    // BF16累加器

// 流水线寄存器定义
// Stage1 -> Stage2
reg [2:0]  precision_mode_s2;
reg        mixed_mode_s2;
reg        pe_enable_s2;

// Stage2 -> Stage3
reg [2:0]  precision_mode_s3;
reg        mixed_mode_s3;
reg        pe_enable_s3;
reg [31:0] a_preprocessed;
reg [31:0] b_preprocessed;
// reg [63:0] a_fp_preprocessed;
// reg [63:0] b_fp_preprocessed;

// Stage3 -> Stage4
reg [2:0]  precision_mode_s4;
reg        pe_enable_s4;
reg [63:0] int_product;
reg        int_overflow;

reg [31:0] fp32_product;
// reg [63:0] fp64_product;
reg [15:0] bf16_product;

// Stage4 -> Stage5
reg [2:0]  precision_mode_s5;
reg        pe_enable_s5;
reg [63:0] int_result;
reg [63:0] fp_result;
reg [31:0] bf16_result;
reg        overflow_int;

// 初始化所有寄存器

initial begin
    // 累加器寄存器
    int_acc = 0;
    fp_acc = 0;
    bf16_acc = 0;
    
    // Stage1 -> Stage2 寄存器
    precision_mode_s2 = 0;
    mixed_mode_s2 = 0;
    pe_enable_s2 = 0;
    
    // Stage2 -> Stage3 寄存器
    precision_mode_s3 = 0;
    mixed_mode_s3 = 0;
    pe_enable_s3 = 0;
    a_preprocessed = 0;
    b_preprocessed = 0;
    
    // Stage3 -> Stage4 寄存器
    precision_mode_s4 = 0;
    pe_enable_s4 = 0;
    int_product = 0;
    fp32_product = 0;
    bf16_product = 0;
    
    // Stage4 -> Stage5 寄存器
    precision_mode_s5 = 0;
    pe_enable_s5 = 0;
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
        precision_mode_s2 <= 0;
        mixed_mode_s2 <= 0;
        pe_enable_s2 <= 0;
        a_preprocessed <= 0;
        b_preprocessed <= 0;
    end else begin

        // 脉动数据传递 ＋ 数据处理
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
            b_preprocessed <= 0;
        end

        // 传递控制信号
        precision_mode_s2 <= precision_mode;
        mixed_mode_s2 <= mixed_mode;
        pe_enable_s2 <= pe_enable;
    end
end

// Stage3: 乘法阶段
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        precision_mode_s3 <= 0;
        pe_enable_s3 <= 0;
        int_product <= 0;
        fp32_product <= 0;
        // fp64_product <= 0;
        bf16_product <= 0;
    end else begin
        // 传递控制信号
        precision_mode_s3 <= precision_mode_s2;
        pe_enable_s3 <= pe_enable_s2;
        
        // 执行乘法运算
        if (pe_enable_s2) begin
            case(precision_mode_s2)
                3'b000: {int_product, int_overflow}  <= int4_mul(a_preprocessed[3:0], b_preprocessed[3:0]);
                3'b001: {int_product, int_overflow}  <= int8_mul(a_preprocessed[7:0], b_preprocessed[7:0]);
                3'b100: bf16_product <= bf16_mult(a_preprocessed[15:0], b_preprocessed[15:0]);
                default: begin
                // FP32
                fp32_product <= fp32_mult(a_preprocessed, b_preprocessed);

                //FP64
                // a_fp_preprocessed = fp32_to_fp64(a_preprocessed); 
                // b_fp_preprocessed = fp32_to_fp64(b_preprocessed);
                // fp64_product <= fp64_mult(a_fp_preprocessed, b_fp_preprocessed);
                end
            endcase
        end
    end
end

// Stage4: 累加阶段
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        precision_mode_s4 <= 0;
        pe_enable_s4 <= 0;
        int_result <= 0;
        fp_result <= 0;
        bf16_result <= 0;
        overflow <= 0;
    end else begin
        // 传递控制信号
        precision_mode_s4 <= precision_mode_s3;
        pe_enable_s4 <= pe_enable_s3;
        
        if (pe_enable_s3) begin
            case(precision_mode_s3)
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
        // 传递控制信号
        precision_mode_s5 <= precision_mode_s4;
        pe_enable_s5 <= pe_enable_s4;
        
        // 输出结果
        if (pe_enable_s4) begin
            case(precision_mode_s4)
                3'b000, 3'b001: d_out <= int_result;
                3'b100: d_out <= {32'b0, bf16_result};
                default: d_out <= fp_result;
            endcase

            overflow <= overflow_int;
        end
    end
end

// 输入数据预处理（返回符号扩展后的32bit有符号数）
function automatic signed [31:0] preprocess(input [31:0] data);
    case(precision_mode)
        3'b000: return $signed({{28{data[3]}}, data[3:0]});    // INT4
        3'b001: return $signed({{24{data[7]}}, data[7:0]});    // INT8
        3'b100: return $signed({{16{data[15]}}, data[15:0]});  // BF16
        default: return $signed(data);                         // FP32 INT32直接传递
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
    reg signed [9:0] exp_large, exp_small;
    reg [23:0] mant_large, mant_small;
    reg sign_large, sign_small;
    reg [7:0] exp_diff;
    reg [66:0] mant_small_ext, mant_large_ext;
    
    // 4. 加减运算
    reg signed [66:0] mant_sum_ext;
    reg result_sign;
    
    // 5. 归一化与舍入
    reg signed [9:0] exp_out;
    reg [23:0] mant_out;
    reg [66:0] mant_tmp;
    reg [4:0] norm_count;
    reg norm_vld;
    
    // 6. 舍入位
    reg guard, round, sticky, lsb, round_up;
    reg [22:0] final_frac;
    reg [7:0] final_exp;
    reg final_sign;
    
    // 7. 优先编码器临时变量
    reg [31:0] penc_input;
    integer i;
    
    begin
        // === 1. 解码阶段 ===
        sign_a = a[31];
        sign_b = b[31];
        exp_a = a[30:23];
        exp_b = b[30:23];
        frac_a = a[22:0];
        frac_b = b[22:0];
        
        mant_a = (exp_a == 0) ? {1'b0, frac_a} : {1'b1, frac_a};
        mant_b = (exp_b == 0) ? {1'b0, frac_b} : {1'b1, frac_b};
        
        // === 2. 特殊值检测 ===
        a_is_nan = (exp_a == 8'hFF) && (frac_a != 0);
        b_is_nan = (exp_b == 8'hFF) && (frac_b != 0);
        a_is_inf = (exp_a == 8'hFF) && (frac_a == 0);
        b_is_inf = (exp_b == 8'hFF) && (frac_b == 0);
        a_is_zero = (exp_a == 0) && (frac_a == 0);
        b_is_zero = (exp_b == 0) && (frac_b == 0);
        
        is_nan = a_is_nan || b_is_nan;
        is_inf = a_is_inf || b_is_inf;
        both_inf_opposite_sign = a_is_inf && b_is_inf && (sign_a != sign_b);
        
        // === 3. 对齐尾数 ===
        a_bigger = (exp_a > exp_b) || ((exp_a == exp_b) && (mant_a >= mant_b));
        exp_large = a_bigger ? {2'b0, exp_a} : {2'b0, exp_b};
        exp_small = a_bigger ? {2'b0, exp_b} : {2'b0, exp_a};
        mant_large = a_bigger ? mant_a : mant_b;
        mant_small = a_bigger ? mant_b : mant_a;
        sign_large = a_bigger ? sign_a : sign_b;
        sign_small = a_bigger ? sign_b : sign_a;
        
        exp_diff = exp_large - exp_small;
        mant_small_ext = {2'd0, mant_small, 42'd0} >> exp_diff;
        mant_large_ext = {2'd0, mant_large, 42'd0};
        
        // === 4. 加减运算 ===
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
        
        // === 5. 归一化处理 ===
        mant_tmp = mant_sum_ext;
        
        // 检查溢出
        if (mant_tmp[66]) begin
            mant_out = mant_tmp[66:43];  // 取高24位
            exp_out = exp_large + 1;
        end 
        // 规格化处理
        else if (mant_tmp[65]) begin
            mant_out = mant_tmp[65:42];
            exp_out = exp_large;
        end
        else begin
            // 用if-else实现优先编码器
            penc_input = {11'b0, mant_tmp[64:42]};
            norm_count = 0;
            norm_vld = 0;
            
            for (i = 31; i >= 0; i = i - 1) begin
                if (penc_input[i] && !norm_vld) begin
                    norm_count = i;
                    norm_vld = 1;
                end
            end
            
            mant_out = mant_tmp[64:42] << (5'd23 - norm_count);
            exp_out = exp_large + norm_count - 5'd23;
        end
        
        // === 6. 舍入处理 ===
        guard = mant_tmp[41];
        round = mant_tmp[40];
        sticky = |mant_tmp[39:0];
        lsb = mant_out[0];
        round_up = guard && (round || sticky || lsb);
        
        if (round_up) begin
            mant_out = mant_out + 1;
            if (mant_out[23]) begin  // 舍入后溢出
                mant_out = mant_out >> 1;
                exp_out = exp_out + 1;
            end
        end
        
        // === 7. 溢出检查 ===
        if (exp_out > 254) begin
            final_exp = 8'hFF;
            final_frac = 23'b0;
        end
        else if (exp_out < 1) begin
            if (exp_out < -23) begin
                final_exp = 0;
                final_frac = 0;
            end else begin
                final_exp = 0;
                final_frac = mant_out[22:0] >> (1 - exp_out);
            end
        end
        else begin
            final_exp = exp_out[7:0];
            final_frac = mant_out[22:0];
        end
        
        // === 8. 结果组装 ===
        if (is_nan || both_inf_opposite_sign) begin
            fp32_add = 32'h7FC00000; // NaN
        end
        else if (is_inf) begin
            fp32_add = {result_sign, 8'hFF, 23'b0}; // Inf
        end
        else if (a_is_zero && b_is_zero) begin
            fp32_add = {result_sign, 31'b0}; // 带符号的零
        end
        else begin
            fp32_add = {result_sign, final_exp, final_frac};
        end
    end
endfunction

function [31:0] fp32_mult;
    input  [31:0] a, b;
    
    reg         a_sign, b_sign, product_sign;
    reg  [7:0]  a_exp, b_exp;
    reg [22:0]  a_frac, b_frac;
    reg         a_is_nan, b_is_nan, a_is_inf, b_is_inf, a_is_zero, b_is_zero;
    reg         is_inf_zero;
    reg [23:0]  A_sig, B_sig;
    reg signed [9:0] exp_sum;
    reg [47:0]  sig_product;
    reg [23:0]  normalized;
    reg         guard_bit, round_bit, sticky_bit;
    reg [22:0]  final_frac;
    reg [7:0]   final_exp;
    
    begin
        a_sign = a[31]; b_sign = b[31];
        a_exp  = a[30:23]; b_exp = b[30:23];
        a_frac = a[22:0]; b_frac = b[22:0];
        
        a_is_nan  = (a_exp == 8'hFF) && (a_frac != 0);
        b_is_nan  = (b_exp == 8'hFF) && (b_frac != 0);
        a_is_inf  = (a_exp == 8'hFF) && (a_frac == 0);
        b_is_inf  = (b_exp == 8'hFF) && (b_frac == 0);
        a_is_zero = (a_exp == 0) && (a_frac == 0);
        b_is_zero = (b_exp == 0) && (b_frac == 0);
        is_inf_zero = (a_is_inf && b_is_zero) || (b_is_inf && a_is_zero);   // Inf×0=NaN
        
        product_sign = a_sign ^ b_sign;                             // 符号位乘法结果
        
        A_sig = (a_exp != 0) ? {1'b1, a_frac} : {1'b0, a_frac};     // 规格数尾数位补1
        B_sig = (b_exp != 0) ? {1'b1, b_frac} : {1'b0, b_frac};     // 非规格数尾数补0
        exp_sum = a_exp + b_exp - 8'd127;                           // 指数位乘法结果 去偏置+补偿
        sig_product = A_sig * B_sig;                                // 尾数位乘法结果 
        
        if (sig_product[47]) begin          // 位47
            normalized = sig_product[47:24];
            exp_sum = exp_sum + 1;
            guard_bit = sig_product[23];
            round_bit = sig_product[22];
            sticky_bit = |sig_product[21:0];
        end
        else if (sig_product[46]) begin    // 位46
            normalized = sig_product[46:23];
            exp_sum = exp_sum ;
            guard_bit = sig_product[22];
            round_bit = sig_product[21];
            sticky_bit = |sig_product[20:0];
        end
        else if (sig_product[45]) begin    // 位45
            normalized = sig_product[45:22];
            exp_sum = exp_sum - 1;
            guard_bit = sig_product[21];
            round_bit = sig_product[20];
            sticky_bit = |sig_product[19:0];
        end
        else if (sig_product[44]) begin    // 位44
            normalized = sig_product[44:21];
            exp_sum = exp_sum -2 ;
            guard_bit = sig_product[20];
            round_bit = sig_product[19];
            sticky_bit = |sig_product[18:0];
        end
        else if (sig_product[43]) begin    // 位43
            normalized = sig_product[43:20];
            exp_sum = exp_sum -3;
            guard_bit = sig_product[19];
            round_bit = sig_product[18];
            sticky_bit = |sig_product[17:0];
        end
        else if (sig_product[42]) begin    // 位42
            normalized = sig_product[42:19];
            exp_sum = exp_sum -4;
            guard_bit = sig_product[18];
            round_bit = sig_product[17];
            sticky_bit = |sig_product[16:0];
        end
        else if (sig_product[41]) begin    // 位41
            normalized = sig_product[41:18];
            exp_sum = exp_sum -5;
            guard_bit = sig_product[17];
            round_bit = sig_product[16];
            sticky_bit = |sig_product[15:0];
        end
        else if (sig_product[40]) begin    // 位40
            normalized = sig_product[40:17];
            exp_sum = exp_sum -6;
            guard_bit = sig_product[16];
            round_bit = sig_product[15];
            sticky_bit = |sig_product[14:0];
        end
        else if (sig_product[39]) begin    // 位39
            normalized = sig_product[39:16];
            exp_sum = exp_sum -7;
            guard_bit = sig_product[15];
            round_bit = sig_product[14];
            sticky_bit = |sig_product[13:0];
        end
        else if (sig_product[38]) begin    // 位38
            normalized = sig_product[38:15];
            exp_sum = exp_sum -8;
            guard_bit = sig_product[14];
            round_bit = sig_product[13];
            sticky_bit = |sig_product[12:0];
        end
        else if (sig_product[37]) begin    // 位37
            normalized = sig_product[37:14];
            exp_sum = exp_sum -9;
            guard_bit = sig_product[13];
            round_bit = sig_product[12];
            sticky_bit = |sig_product[11:0];
        end
        else if (sig_product[36]) begin    // 位36
            normalized = sig_product[36:13];
            exp_sum = exp_sum -10;
            guard_bit = sig_product[12];
            round_bit = sig_product[11];
            sticky_bit = |sig_product[10:0];
        end
        else if (sig_product[35]) begin    // 位35
            normalized = sig_product[35:12];
            exp_sum = exp_sum -11;
            guard_bit = sig_product[11];
            round_bit = sig_product[10];
            sticky_bit = |sig_product[9:0];
        end
        else if (sig_product[34]) begin    // 位34
            normalized = sig_product[34:11];
            exp_sum = exp_sum -12;
            guard_bit = sig_product[10];
            round_bit = sig_product[9];
            sticky_bit = |sig_product[8:0];
        end
        else if (sig_product[33]) begin    // 位33
            normalized = sig_product[33:10];
            exp_sum = exp_sum -13;
            guard_bit = sig_product[9];
            round_bit = sig_product[8];
            sticky_bit = |sig_product[7:0];
        end
        else if (sig_product[32]) begin    // 位32
            normalized = sig_product[32:9];
            exp_sum = exp_sum -14;
            guard_bit = sig_product[8];
            round_bit = sig_product[7];
            sticky_bit = |sig_product[6:0];
        end
        else if (sig_product[31]) begin    // 位31
            normalized = sig_product[31:8];
            exp_sum = exp_sum -15;
            guard_bit = sig_product[7];
            round_bit = sig_product[6];
            sticky_bit = |sig_product[5:0];
        end
        else if (sig_product[30]) begin    // 位30
            normalized = sig_product[30:7];
            exp_sum = exp_sum -16;
            guard_bit = sig_product[6];
            round_bit = sig_product[5];
            sticky_bit = |sig_product[4:0];
        end
        else if (sig_product[29]) begin    // 位29
            normalized = sig_product[29:6];
            exp_sum = exp_sum -17;
            guard_bit = sig_product[5];
            round_bit = sig_product[4];
            sticky_bit = |sig_product[3:0];
        end
        else if (sig_product[28]) begin    // 位28
            normalized = sig_product[28:5];
            exp_sum = exp_sum -18;
            guard_bit = sig_product[4];
            round_bit = sig_product[3];
            sticky_bit = |sig_product[2:0];
        end
        else if (sig_product[27]) begin    // 位27
            normalized = sig_product[27:4];
            exp_sum = exp_sum -19;
            guard_bit = sig_product[3];
            round_bit = sig_product[2];
            sticky_bit = |sig_product[1:0];
        end
        else if (sig_product[26]) begin    // 位26
            normalized = sig_product[26:3];
            exp_sum = exp_sum -20;
            guard_bit = sig_product[2];
            round_bit = sig_product[1];
            sticky_bit = sig_product[0];
        end
        else if (sig_product[25]) begin    // 位25
            normalized = sig_product[25:2];
            exp_sum = exp_sum -21;
            guard_bit = sig_product[1];
            round_bit = sig_product[0];
            sticky_bit = 1'b0;
        end
        else if (sig_product[24]) begin    // 位24
            normalized = sig_product[24:1];
            exp_sum = exp_sum -22;
            guard_bit = sig_product[0];
            round_bit = 1'b0;
            sticky_bit = 1'b0;
        end
        else begin                        // 所有位为0
            normalized = 24'b0;
            guard_bit = 1'b0;
            round_bit = 1'b0;
            sticky_bit = 1'b0;
        end
        
        // 舍入处理 (向偶舍入)
        if (guard_bit && (round_bit || sticky_bit || normalized[0])) begin
            normalized = normalized + 1;
            if (normalized[23]) begin      // 检查尾数溢出
                normalized = normalized >> 1;
                exp_sum = exp_sum + 1;
            end
        end
        
        // 结果组装
        if (a_is_nan || b_is_nan || is_inf_zero) begin
            fp32_mult = 32'h7FC00000; // 标准NaN编码
        end
        else if (a_is_inf || b_is_inf) begin
            fp32_mult = {product_sign, 8'hFF, 23'b0}; // Inf
        end
        else if (a_is_zero || b_is_zero) begin
            fp32_mult = {product_sign, 31'b0}; // 带符号的零
        end
        else if (exp_sum[9] || exp_sum[7:0] == 8'hFF) begin // 指数上溢
            fp32_mult = {product_sign, 8'hFF, 23'b0}; // Inf
        end
        else if (exp_sum < 0) begin // 指数下溢
            fp32_mult = {product_sign, 31'b0}; // 刷新为零
        end
        else begin
            final_exp = exp_sum[7:0];
            final_frac = normalized[22:0];
            fp32_mult = {product_sign, final_exp, final_frac};
        end
    end
endfunction

function [63:0] fp64_mult;
    input  [63:0] a, b;
    
    reg         a_sign, b_sign, product_sign;
    reg  [10:0] a_exp, b_exp;
    reg [51:0]  a_frac, b_frac;
    reg         a_is_nan, b_is_nan, a_is_inf, b_is_inf, a_is_zero, b_is_zero;
    reg         is_inf_zero;
    reg [52:0]  A_sig, B_sig;
    reg signed [11:0] exp_sum;
    reg [105:0] sig_product;
    reg [52:0]  normalized;
    reg         guard_bit, round_bit, sticky_bit;
    reg [51:0]  final_frac;
    reg [10:0]  final_exp;
    
    begin
        // 提取符号、指数、尾数
        a_sign = a[63]; b_sign = b[63];
        a_exp  = a[62:52]; b_exp = b[62:52];
        a_frac = a[51:0]; b_frac = b[51:0];
        
        // 特殊值检测
        a_is_nan  = (a_exp == 11'h7FF) && (a_frac != 0);
        b_is_nan  = (b_exp == 11'h7FF) && (b_frac != 0);
        a_is_inf  = (a_exp == 11'h7FF) && (a_frac == 0);
        b_is_inf  = (b_exp == 11'h7FF) && (b_frac == 0);
        a_is_zero = (a_exp == 0) && (a_frac == 0);
        b_is_zero = (b_exp == 0) && (b_frac == 0);
        is_inf_zero = (a_is_inf && b_is_zero) || (b_is_inf && a_is_zero);   // Inf×0=NaN
        
        // 通用处理
        product_sign = a_sign ^ b_sign;                             // 符号位乘法结果
        
        // 非特殊值处理
        A_sig = (a_exp != 0) ? {1'b1, a_frac} : {1'b0, a_frac};     // 规格数尾数位补1
        B_sig = (b_exp != 0) ? {1'b1, b_frac} : {1'b0, b_frac};     // 非规格数尾数补0
        exp_sum = a_exp + b_exp - 11'd1023;                         // 指数位乘法结果 去偏置+补偿
        sig_product = A_sig * B_sig;                                // 尾数位乘法结果 
        
        // 完整的规格化处理（106位优先级编码器）
        if (sig_product[105]) begin          // 位105
            normalized = sig_product[105:53];
            exp_sum = exp_sum + 1;
            guard_bit = sig_product[52];
            round_bit = sig_product[51];
            sticky_bit = |sig_product[50:0];
        end
        else if (sig_product[104]) begin    // 位104
            normalized = sig_product[104:52];
            exp_sum = exp_sum;
            guard_bit = sig_product[51];
            round_bit = sig_product[50];
            sticky_bit = |sig_product[49:0];
        end
        else if (sig_product[103]) begin    // 位103
            normalized = sig_product[103:51];
            exp_sum = exp_sum - 1;
            guard_bit = sig_product[50];
            round_bit = sig_product[49];
            sticky_bit = |sig_product[48:0];
        end
        else if (sig_product[102]) begin    // 位102
            normalized = sig_product[102:50];
            exp_sum = exp_sum - 2;
            guard_bit = sig_product[49];
            round_bit = sig_product[48];
            sticky_bit = |sig_product[47:0];
        end
        else if (sig_product[101]) begin    // 位101
            normalized = sig_product[101:49];
            exp_sum = exp_sum - 3;
            guard_bit = sig_product[48];
            round_bit = sig_product[47];
            sticky_bit = |sig_product[46:0];
        end
        else if (sig_product[100]) begin    // 位100
            normalized = sig_product[100:48];
            exp_sum = exp_sum - 4;
            guard_bit = sig_product[47];
            round_bit = sig_product[46];
            sticky_bit = |sig_product[45:0];
        end
        else if (sig_product[99]) begin     // 位99
            normalized = sig_product[99:47];
            exp_sum = exp_sum - 5;
            guard_bit = sig_product[46];
            round_bit = sig_product[45];
            sticky_bit = |sig_product[44:0];
        end
        else if (sig_product[98]) begin     // 位98
            normalized = sig_product[98:46];
            exp_sum = exp_sum - 6;
            guard_bit = sig_product[45];
            round_bit = sig_product[44];
            sticky_bit = |sig_product[43:0];
        end
        else if (sig_product[97]) begin     // 位97
            normalized = sig_product[97:45];
            exp_sum = exp_sum - 7;
            guard_bit = sig_product[44];
            round_bit = sig_product[43];
            sticky_bit = |sig_product[42:0];
        end
        else if (sig_product[96]) begin     // 位96
            normalized = sig_product[96:44];
            exp_sum = exp_sum - 8;
            guard_bit = sig_product[43];
            round_bit = sig_product[42];
            sticky_bit = |sig_product[41:0];
        end
        else if (sig_product[95]) begin     // 位95
            normalized = sig_product[95:43];
            exp_sum = exp_sum - 9;
            guard_bit = sig_product[42];
            round_bit = sig_product[41];
            sticky_bit = |sig_product[40:0];
        end
        else if (sig_product[94]) begin     // 位94
            normalized = sig_product[94:42];
            exp_sum = exp_sum - 10;
            guard_bit = sig_product[41];
            round_bit = sig_product[40];
            sticky_bit = |sig_product[39:0];
        end
        else if (sig_product[93]) begin     // 位93
            normalized = sig_product[93:41];
            exp_sum = exp_sum - 11;
            guard_bit = sig_product[40];
            round_bit = sig_product[39];
            sticky_bit = |sig_product[38:0];
        end
        else if (sig_product[92]) begin     // 位92
            normalized = sig_product[92:40];
            exp_sum = exp_sum - 12;
            guard_bit = sig_product[39];
            round_bit = sig_product[38];
            sticky_bit = |sig_product[37:0];
        end
        else if (sig_product[91]) begin     // 位91
            normalized = sig_product[91:39];
            exp_sum = exp_sum - 13;
            guard_bit = sig_product[38];
            round_bit = sig_product[37];
            sticky_bit = |sig_product[36:0];
        end
        else if (sig_product[90]) begin     // 位90
            normalized = sig_product[90:38];
            exp_sum = exp_sum - 14;
            guard_bit = sig_product[37];
            round_bit = sig_product[36];
            sticky_bit = |sig_product[35:0];
        end
        else if (sig_product[89]) begin     // 位89
            normalized = sig_product[89:37];
            exp_sum = exp_sum - 15;
            guard_bit = sig_product[36];
            round_bit = sig_product[35];
            sticky_bit = |sig_product[34:0];
        end
        else if (sig_product[88]) begin     // 位88
            normalized = sig_product[88:36];
            exp_sum = exp_sum - 16;
            guard_bit = sig_product[35];
            round_bit = sig_product[34];
            sticky_bit = |sig_product[33:0];
        end
        else if (sig_product[87]) begin     // 位87
            normalized = sig_product[87:35];
            exp_sum = exp_sum - 17;
            guard_bit = sig_product[34];
            round_bit = sig_product[33];
            sticky_bit = |sig_product[32:0];
        end
        else if (sig_product[86]) begin     // 位86
            normalized = sig_product[86:34];
            exp_sum = exp_sum - 18;
            guard_bit = sig_product[33];
            round_bit = sig_product[32];
            sticky_bit = |sig_product[31:0];
        end
        else if (sig_product[85]) begin     // 位85
            normalized = sig_product[85:33];
            exp_sum = exp_sum - 19;
            guard_bit = sig_product[32];
            round_bit = sig_product[31];
            sticky_bit = |sig_product[30:0];
        end
        else if (sig_product[84]) begin     // 位84
            normalized = sig_product[84:32];
            exp_sum = exp_sum - 20;
            guard_bit = sig_product[31];
            round_bit = sig_product[30];
            sticky_bit = |sig_product[29:0];
        end
        else if (sig_product[83]) begin     // 位83
            normalized = sig_product[83:31];
            exp_sum = exp_sum - 21;
            guard_bit = sig_product[30];
            round_bit = sig_product[29];
            sticky_bit = |sig_product[28:0];
        end
        else if (sig_product[82]) begin     // 位82
            normalized = sig_product[82:30];
            exp_sum = exp_sum - 22;
            guard_bit = sig_product[29];
            round_bit = sig_product[28];
            sticky_bit = |sig_product[27:0];
        end
        else if (sig_product[81]) begin     // 位81
            normalized = sig_product[81:29];
            exp_sum = exp_sum - 23;
            guard_bit = sig_product[28];
            round_bit = sig_product[27];
            sticky_bit = |sig_product[26:0];
        end
        else if (sig_product[80]) begin     // 位80
            normalized = sig_product[80:28];
            exp_sum = exp_sum - 24;
            guard_bit = sig_product[27];
            round_bit = sig_product[26];
            sticky_bit = |sig_product[25:0];
        end
        else if (sig_product[79]) begin     // 位79
            normalized = sig_product[79:27];
            exp_sum = exp_sum - 25;
            guard_bit = sig_product[26];
            round_bit = sig_product[25];
            sticky_bit = |sig_product[24:0];
        end
        else if (sig_product[78]) begin     // 位78
            normalized = sig_product[78:26];
            exp_sum = exp_sum - 26;
            guard_bit = sig_product[25];
            round_bit = sig_product[24];
            sticky_bit = |sig_product[23:0];
        end
        else if (sig_product[77]) begin     // 位77
            normalized = sig_product[77:25];
            exp_sum = exp_sum - 27;
            guard_bit = sig_product[24];
            round_bit = sig_product[23];
            sticky_bit = |sig_product[22:0];
        end
        else if (sig_product[76]) begin     // 位76
            normalized = sig_product[76:24];
            exp_sum = exp_sum - 28;
            guard_bit = sig_product[23];
            round_bit = sig_product[22];
            sticky_bit = |sig_product[21:0];
        end
        else if (sig_product[75]) begin     // 位75
            normalized = sig_product[75:23];
            exp_sum = exp_sum - 29;
            guard_bit = sig_product[22];
            round_bit = sig_product[21];
            sticky_bit = |sig_product[20:0];
        end
        else if (sig_product[74]) begin     // 位74
            normalized = sig_product[74:22];
            exp_sum = exp_sum - 30;
            guard_bit = sig_product[21];
            round_bit = sig_product[20];
            sticky_bit = |sig_product[19:0];
        end
        else if (sig_product[73]) begin     // 位73
            normalized = sig_product[73:21];
            exp_sum = exp_sum - 31;
            guard_bit = sig_product[20];
            round_bit = sig_product[19];
            sticky_bit = |sig_product[18:0];
        end
        else if (sig_product[72]) begin     // 位72
            normalized = sig_product[72:20];
            exp_sum = exp_sum - 32;
            guard_bit = sig_product[19];
            round_bit = sig_product[18];
            sticky_bit = |sig_product[17:0];
        end
        else if (sig_product[71]) begin     // 位71
            normalized = sig_product[71:19];
            exp_sum = exp_sum - 33;
            guard_bit = sig_product[18];
            round_bit = sig_product[17];
            sticky_bit = |sig_product[16:0];
        end
        else if (sig_product[70]) begin     // 位70
            normalized = sig_product[70:18];
            exp_sum = exp_sum - 34;
            guard_bit = sig_product[17];
            round_bit = sig_product[16];
            sticky_bit = |sig_product[15:0];
        end
        else if (sig_product[69]) begin     // 位69
            normalized = sig_product[69:17];
            exp_sum = exp_sum - 35;
            guard_bit = sig_product[16];
            round_bit = sig_product[15];
            sticky_bit = |sig_product[14:0];
        end
        else if (sig_product[68]) begin     // 位68
            normalized = sig_product[68:16];
            exp_sum = exp_sum - 36;
            guard_bit = sig_product[15];
            round_bit = sig_product[14];
            sticky_bit = |sig_product[13:0];
        end
        else if (sig_product[67]) begin     // 位67
            normalized = sig_product[67:15];
            exp_sum = exp_sum - 37;
            guard_bit = sig_product[14];
            round_bit = sig_product[13];
            sticky_bit = |sig_product[12:0];
        end
        else if (sig_product[66]) begin     // 位66
            normalized = sig_product[66:14];
            exp_sum = exp_sum - 38;
            guard_bit = sig_product[13];
            round_bit = sig_product[12];
            sticky_bit = |sig_product[11:0];
        end
        else if (sig_product[65]) begin     // 位65
            normalized = sig_product[65:13];
            exp_sum = exp_sum - 39;
            guard_bit = sig_product[12];
            round_bit = sig_product[11];
            sticky_bit = |sig_product[10:0];
        end
        else if (sig_product[64]) begin     // 位64
            normalized = sig_product[64:12];
            exp_sum = exp_sum - 40;
            guard_bit = sig_product[11];
            round_bit = sig_product[10];
            sticky_bit = |sig_product[9:0];
        end
        else if (sig_product[63]) begin     // 位63
            normalized = sig_product[63:11];
            exp_sum = exp_sum - 41;
            guard_bit = sig_product[10];
            round_bit = sig_product[9];
            sticky_bit = |sig_product[8:0];
        end
        else if (sig_product[62]) begin     // 位62
            normalized = sig_product[62:10];
            exp_sum = exp_sum - 42;
            guard_bit = sig_product[9];
            round_bit = sig_product[8];
            sticky_bit = |sig_product[7:0];
        end
        else if (sig_product[61]) begin     // 位61
            normalized = sig_product[61:9];
            exp_sum = exp_sum - 43;
            guard_bit = sig_product[8];
            round_bit = sig_product[7];
            sticky_bit = |sig_product[6:0];
        end
        else if (sig_product[60]) begin     // 位60
            normalized = sig_product[60:8];
            exp_sum = exp_sum - 44;
            guard_bit = sig_product[7];
            round_bit = sig_product[6];
            sticky_bit = |sig_product[5:0];
        end
        else if (sig_product[59]) begin     // 位59
            normalized = sig_product[59:7];
            exp_sum = exp_sum - 45;
            guard_bit = sig_product[6];
            round_bit = sig_product[5];
            sticky_bit = |sig_product[4:0];
        end
        else if (sig_product[58]) begin     // 位58
            normalized = sig_product[58:6];
            exp_sum = exp_sum - 46;
            guard_bit = sig_product[5];
            round_bit = sig_product[4];
            sticky_bit = |sig_product[3:0];
        end
        else if (sig_product[57]) begin     // 位57
            normalized = sig_product[57:5];
            exp_sum = exp_sum - 47;
            guard_bit = sig_product[4];
            round_bit = sig_product[3];
            sticky_bit = |sig_product[2:0];
        end
        else if (sig_product[56]) begin     // 位56
            normalized = sig_product[56:4];
            exp_sum = exp_sum - 48;
            guard_bit = sig_product[3];
            round_bit = sig_product[2];
            sticky_bit = |sig_product[1:0];
        end
        else if (sig_product[55]) begin     // 位55
            normalized = sig_product[55:3];
            exp_sum = exp_sum - 49;
            guard_bit = sig_product[2];
            round_bit = sig_product[1];
            sticky_bit = sig_product[0];
        end
        else if (sig_product[54]) begin     // 位54
            normalized = sig_product[54:2];
            exp_sum = exp_sum - 50;
            guard_bit = sig_product[1];
            round_bit = sig_product[0];
            sticky_bit = 1'b0;
        end
        else if (sig_product[53]) begin     // 位53
            normalized = sig_product[53:1];
            exp_sum = exp_sum - 51;
            guard_bit = sig_product[0];
            round_bit = 1'b0;
            sticky_bit = 1'b0;
        end
        else begin                        // 所有位为0
            normalized = 53'b0;
            guard_bit = 1'b0;
            round_bit = 1'b0;
            sticky_bit = 1'b0;
        end
        
        // 舍入处理 (向偶舍入)
        if (guard_bit && (round_bit || sticky_bit || normalized[0])) begin
            normalized = normalized + 1;
            if (normalized[52]) begin      // 检查尾数溢出
                normalized = normalized >> 1;
                exp_sum = exp_sum + 1;
            end
        end
        
        // 结果组装
        if (a_is_nan || b_is_nan || is_inf_zero) begin
            fp64_mult = 64'h7FF8000000000000; // 标准NaN编码
        end
        else if (a_is_inf || b_is_inf) begin
            fp64_mult = {product_sign, 11'h7FF, 52'b0}; // Inf
        end
        else if (a_is_zero || b_is_zero) begin
            fp64_mult = {product_sign, 63'b0}; // 带符号的零
        end
        else if (exp_sum[11] || exp_sum[10:0] == 11'h7FF) begin // 指数上溢
            fp64_mult = {product_sign, 11'h7FF, 52'b0}; // Inf
        end
        else if (exp_sum < -1022) begin // 指数下溢（非规格化数）
            fp64_mult = {product_sign, 63'b0}; // 刷新为零
        end
        else begin
            final_exp = exp_sum[10:0];
            final_frac = normalized[51:0];
            fp64_mult = {product_sign, final_exp, final_frac};
        end
    end
endfunction


function [63:0] fp64_add;
    input [63:0] a, b;
    
    // 1. 提取符号位、指数位、尾数
    reg sign_a, sign_b;
    reg [10:0] exp_a, exp_b;
    reg [51:0] frac_a, frac_b;
    reg [52:0] mant_a, mant_b;
    
    // 2. 特殊值处理
    reg a_is_nan, b_is_nan, a_is_inf, b_is_inf, a_is_zero, b_is_zero;
    reg is_nan, is_inf, both_inf_opposite_sign;
    
    // 3. 对齐处理
    reg a_bigger;
    reg signed [12:0] exp_large, exp_small;
    reg [52:0] mant_large, mant_small;
    reg sign_large, sign_small;
    reg [11:0] exp_diff;
    reg [115:0] mant_small_ext, mant_large_ext;
    
    // 4. 加减运算
    reg signed [115:0] mant_sum_ext;
    reg result_sign;
    
    // 5. 归一化与舍入
    reg signed [12:0] exp_out;
    reg [52:0] mant_out;
    reg [115:0] mant_tmp;
    reg [6:0] norm_count;
    reg norm_vld;
    
    // 6. 舍入位
    reg guard, round, sticky, lsb, round_up;
    reg [51:0] final_frac;
    reg [10:0] final_exp;
    reg final_sign;
    
    // 7. 优先编码器临时变量
    reg [63:0] penc_input;
    integer i;
    
    begin
        // === 1. 解码阶段 ===
        sign_a = a[63];
        sign_b = b[63];
        exp_a = a[62:52];
        exp_b = b[62:52];
        frac_a = a[51:0];
        frac_b = b[51:0];
        
        mant_a = (exp_a == 0) ? {1'b0, frac_a} : {1'b1, frac_a};
        mant_b = (exp_b == 0) ? {1'b0, frac_b} : {1'b1, frac_b};
        
        // === 2. 特殊值检测 ===
        a_is_nan = (exp_a == 11'h7FF) && (frac_a != 0);
        b_is_nan = (exp_b == 11'h7FF) && (frac_b != 0);
        a_is_inf = (exp_a == 11'h7FF) && (frac_a == 0);
        b_is_inf = (exp_b == 11'h7FF) && (frac_b == 0);
        a_is_zero = (exp_a == 0) && (frac_a == 0);
        b_is_zero = (exp_b == 0) && (frac_b == 0);
        
        is_nan = a_is_nan || b_is_nan;
        is_inf = a_is_inf || b_is_inf;
        both_inf_opposite_sign = a_is_inf && b_is_inf && (sign_a != sign_b);
        
        // === 3. 对齐尾数 ===
        a_bigger = (exp_a > exp_b) || ((exp_a == exp_b) && (mant_a >= mant_b));
        exp_large = a_bigger ? {2'b0, exp_a} : {2'b0, exp_b};
        exp_small = a_bigger ? {2'b0, exp_b} : {2'b0, exp_a};
        mant_large = a_bigger ? mant_a : mant_b;
        mant_small = a_bigger ? mant_b : mant_a;
        sign_large = a_bigger ? sign_a : sign_b;
        sign_small = a_bigger ? sign_b : sign_a;
        
        exp_diff = exp_large - exp_small;
        mant_small_ext = {2'd0, mant_small, 62'd0} >> exp_diff;
        mant_large_ext = {2'd0, mant_large, 62'd0};
        
        // === 4. 加减运算 ===
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
        
        // === 5. 归一化处理 ===
        mant_tmp = mant_sum_ext;
        
        // 检查溢出
        if (mant_tmp[115]) begin
            mant_out = mant_tmp[115:63];  // 取高53位
            exp_out = exp_large + 1;
        end 
        // 规格化处理
        else if (mant_tmp[114]) begin
            mant_out = mant_tmp[114:62];
            exp_out = exp_large;
        end
        else begin
            // 用if-else实现优先编码器
            penc_input = {11'b0, mant_tmp[113:62]};
            norm_count = 0;
            norm_vld = 0;
            
            for (i = 63; i >= 0; i = i - 1) begin
                if (penc_input[i] && !norm_vld) begin
                    norm_count = i;
                    norm_vld = 1;
                end
            end
            
            mant_out = mant_tmp[113:62] << (7'd52 - norm_count);
            exp_out = exp_large + norm_count - 7'd52;
        end
        
        // === 6. 舍入处理 ===
        guard = mant_tmp[61];
        round = mant_tmp[60];
        sticky = |mant_tmp[59:0];
        lsb = mant_out[0];
        round_up = guard && (round || sticky || lsb);
        
        if (round_up) begin
            mant_out = mant_out + 1;
            if (mant_out[52]) begin  // 舍入后溢出
                mant_out = mant_out >> 1;
                exp_out = exp_out + 1;
            end
        end
        
        // === 7. 溢出检查 ===
        if (exp_out > 2046) begin
            final_exp = 11'h7FF;
            final_frac = 52'b0;
        end
        else if (exp_out < 1) begin
            if (exp_out < -52) begin
                final_exp = 0;
                final_frac = 0;
            end else begin
                final_exp = 0;
                final_frac = mant_out[51:0] >> (1 - exp_out);
            end
        end
        else begin
            final_exp = exp_out[10:0];
            final_frac = mant_out[51:0];
        end
        
        // === 8. 结果组装 ===
        if (is_nan || both_inf_opposite_sign) begin
            fp64_add = 64'h7FF8000000000000; // NaN
        end
        else if (is_inf) begin
            fp64_add = {result_sign, 11'h7FF, 52'b0}; // Inf
        end
        else if (a_is_zero && b_is_zero) begin
            fp64_add = {result_sign, 63'b0}; // 带符号的零
        end
        else begin
            fp64_add = {result_sign, final_exp, final_frac};
        end
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
