# Multi-Precision Tensor Compute Unit — TPU Design

> **第九届（2025）全国大学生集成电路创新创业大赛 · 中科芯杯 · 华南分赛区二等奖**

基于 FPGA 的多精度张量计算单元（TPU）设计，支持多种数值精度的矩阵乘累加（GEMM）运算 **D = A × B + C**，并实现稀疏矩阵加速。

## 项目信息

| 项目 | 内容 |
|------|------|
| 赛题 | 中科芯杯——多精度张量计算单元设计 |
| 参赛单位 | 华南理工大学 |
| 指导老师 | 姚恩义 |
| 参赛队员 | 朱妤婧、陈锦洋、王欣彤 |
| 队伍编号 | CICC0900784 |
| 目标平台 | Xilinx VCU118 (XCVU9P-L2FLGA2104E) |
| 开发工具 | Vivado 2024.1 |
| 设计语言 | SystemVerilog |

## 核心特性

### 多精度支持

| 精度模式 | 输入数据位宽 | 累加器位宽 | 说明 |
|----------|------------|-----------|------|
| FP32 | 32-bit | 32-bit | IEEE 754 单精度浮点 |
| FP16 | 16-bit | 16-bit | IEEE 754 半精度浮点 |
| BF16 | 16-bit | 16-bit | Brain Floating Point |
| INT8 | 8-bit | 8/32-bit | 8位定点整数 |
| INT4 | 4-bit | 4/32-bit | 4位定点整数 |

- 通过 APB 接口的 `precision_mode` 寄存器动态切换精度模式
- 支持整数混合精度模式（INT4/INT8 输入 + INT32 累加）

### 矩阵维度

支持多种 M×N×K 矩阵配置：

- `m8n32k16` — 8×32×16
- `m16n16k16` — 16×16×16
- `m32n8k16` — 32×8×16

最大维度：M=32, N=32, K=16

### 稀疏矩阵加速

实现了三种稀疏计算模式，跳过零元素运算以提升计算效率：

| 模块 | 说明 |
|------|------|
| `TOP_TPU_Sparse_Matrix` | 基础稀疏矩阵加速 |
| `Pruning_Sparse_Matrix` | 结构化剪枝（Structured Pruning）稀疏加速 |
| `Tile_Wise_Sparse_Matrix` | 分块（Tile-wise）稀疏加速 |

## 架构设计

```
                    APB Interface
                         │
                    ┌────────────┐
                    │  apb_ctrl  │  精度/维度/模式配置
                    └─────┬──────┘
                          │
    AXI Slave             │           AXI Master
    (数据输入)             │           (结果输出)
        │            ┌────────────┐        │
        ├───────────►│  data_load  │        │
        │            └─────┬──────┘        │
        │                  │               │
        │           ┌──────▼──────┐        │
        │           │  FSM 控制器  │        │
        │           └──────┬──────┘        │
        │                  │               │
        │        ┌─────────▼─────────┐     │
        │        │  8×8 PE 脉动阵列  │     │
        │        │  (5级流水线 MAC)   │─────┤
        │        └───────────────────┘     │
        │                                  │
        └──────────────────────────────────┘
```

- **PE 阵列**：8×8 Processing Element 脉动阵列，每个 PE 包含 5 级流水线乘累加单元
- **总线接口**：AXI Slave 接收矩阵数据，AXI Master 写回计算结果，APB 配置控制参数
- **FSM 控制**：IDLE → LOAD_C → LOAD_A → LOAD_B → COMPUTE → OUTPUT 六状态机
- **时钟约束**：7ns 周期（~143 MHz）
- **存储**：使用 Block RAM 存储矩阵数据

## 目录结构

```
.
├── README.md
├── 获奖证书/
│   └── 集创赛 获奖证书.jpg
│
├── 答辩材料/
│   ├── CICC0900784 中科芯杯 分赛区决赛PPT汇报.pptx    # 答辩PPT
│   ├── CICC0900784 中科芯杯 分赛区决赛技术文档.docx    # 技术文档 (Word)
│   ├── CICC0900784 中科芯杯 分赛区决赛技术文档.pdf     # 技术文档 (PDF)
│   ├── CICC0900784 中科芯杯 快速预览页.docx            # 快速预览页
│   ├── CICC0900784 中科芯杯 快速预览页.pdf
│   ├── CICC0900784 中科芯杯 海报展示.png               # 展示海报
│   ├── 32×32＋16×16＋8×8分块PE阵列仿真测试结果.docx    # 仿真结果
│   └── 复活赛.docx
│
├── 源代码/
│   ├── 复赛提交包/
│   │   ├── CICC0900784 中科芯杯 SystemVerilog源码.zip  # 提交压缩包
│   │   └── CICC0900784 中科芯杯 SystemVerilog源码/     # Vivado 工程
│   │       ├── Sparse_Matrix.srcs/                     # 稀疏矩阵模块
│   │       │   ├── sources_1/new/                      # RTL 源码
│   │       │   │   ├── sparse_matrix.sv                #   基础稀疏矩阵 TPU
│   │       │   │   ├── sparse_matrix_4×4.sv            #   4×4 PE 阵列版本
│   │       │   │   ├── Pruning.sv                      #   结构化剪枝版本
│   │       │   │   └── Tile_wise.sv                    #   分块稀疏版本
│   │       │   ├── sim_1/new/                          # 仿真 Testbench
│   │       │   ├── constrs_1/new/                      # 时序约束 (7ns)
│   │       │   └── utils_1/                            # 综合检查点 (.dcp)
│   │       ├── TPU_Defense_Presentation.srcs/          # 答辩演示版本
│   │       └── 最终提交代码 完整tb/                     # 最终提交版本
│   │           ├── TOP_TPU_new.sv                      #   顶层模块
│   │           └── tb/                                 #   完整 Testbench 集
│   ├── 独立模块/
│   │   ├── fp32.sv                  # FP32 PE 测试模块
│   │   ├── fp32_add.sv              # FP32 浮点加法器
│   │   ├── fp32_mul.sv              # FP32 浮点乘法器
│   │   ├── tb_fp32_add.sv           # FP32 加法器 Testbench
│   │   ├── tb_fp32_mul.sv           # FP32 乘法器 Testbench
│   │   ├── compute测试通过.sv        # 计算单元验证
│   │   ├── fp32 PE测试.sv           # PE 单元验证
│   │   ├── tb_int4_m8n32k16.sv      # INT4 矩阵测试
│   │   └── hex.py                   # 测试数据生成脚本
│   └── Testbench/
│       ├── data_load_tb/            # 数据加载模块 Testbench
│       └── for_pre_tb/              # 答辩演示 Testbench (全精度覆盖)
│
└── 测试数据/
    ├── testcase/                    # 完整测试向量集
    │   ├── fp16/                    #   FP16 测试数据
    │   ├── fp32/                    #   FP32 测试数据 (含稀疏矩阵/剪枝/分块)
    │   ├── int4/                    #   INT4 测试数据
    │   ├── int4_int32/              #   INT4输入+INT32累加 测试数据
    │   ├── int8/                    #   INT8 测试数据
    │   ├── int8_int32/              #   INT8输入+INT32累加 测试数据
    │   └── FP32sparse_matrix/       #   FP32 稀疏矩阵专用测试数据
    ├── bf16_m8n32k16/               # BF16 测试数据
    ├── fp16_m8n32k16/               # FP16 测试数据 (含期望结果)
    ├── fp32/                        # FP32 测试数据 (含 bin/dec 格式)
    ├── INT4sparse_matrix/           # INT4 稀疏矩阵测试数据
    └── *.mem                        # 基础测试向量
```

## 仿真运行

### 环境要求

- Xilinx Vivado 2024.1 或更高版本
- 目标器件：XCVU9P-L2FLGA2104E (VCU118)

### 运行步骤

1. 在 Vivado 中打开工程（`源代码/复赛提交包/` 下的 `.srcs` 目录）
2. 将对应精度的测试数据（`测试数据/testcase/`）加载到仿真路径
3. 选择对应的 Testbench 运行行为仿真
4. 验证输出矩阵 D 与期望结果匹配

### 测试数据格式

测试数据文件命名规则：`{矩阵}_{精度}_{维度}.mem.txt`

- `a_` — 输入矩阵 A
- `b_` — 输入矩阵 B
- `c_` — 偏置矩阵 C
- `d_` — 期望输出矩阵 D

## 获奖证书

<p align="center">
  <img src="获奖证书/集创赛 获奖证书.jpg" width="600">
</p>

## License

本项目为竞赛作品，仅供学习交流使用。
