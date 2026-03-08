# Multi-Precision Tensor Compute Unit — FPGA-Based TPU Design

> **2nd Prize, South China Regional Final — 9th National Undergraduate Integrated Circuit Innovation & Entrepreneurship Competition (CICC 2025), Zhongke Xin Cup**

**[中文版 (README.md)](README.md)**

An FPGA-based multi-precision Tensor Processing Unit (TPU) targeting AI inference acceleration through heterogeneous computing. The design features an 8×8 systolic array with 5-stage pipelined MACs, supporting **7 data precisions** (INT4/INT8/INT32/FP16/FP32/FP64/BF16) and **3 mixed-precision modes**. Sparse matrix acceleration via Bitmap encoding, structured pruning, and tile-wise sparsity achieves significant compute and power savings. Synthesized at **214.6 MHz** on Xilinx VCU118 with a peak throughput of **27.47 GOPS**.

## Core GEMM Operation

For an $M \times K$ matrix $A$, a $K \times N$ matrix $B$, and bias matrix $C$, each element of the output matrix $D$ is computed as:

$$D_{ij} = \sum_{k=0}^{K-1} A_{ik} \cdot B_{kj} + C_{ij}, \quad i \in [0, M), \; j \in [0, N)$$

The 8×8 systolic array maps this operation to hardware. Larger matrices are decomposed into 8×8 tiles and processed iteratively via a `pe_counter`-controlled tiling mechanism.

---

## Project Information

| Item | Details |
|------|---------|
| Competition | Zhongke Xin Cup — Multi-Precision Tensor Compute Unit Design |
| Institution | South China University of Technology (SCUT) |
| Advisor | Prof. Enyi Yao |
| Team | Yujing Zhu, Jinyang Chen, Xintong Wang |
| Team ID | CICC0900784 |
| Target Platform | Xilinx VCU118 (XCVU9P-L2FLGA2104E) |
| EDA Tool | Vivado 2024.1 |
| HDL | SystemVerilog |

---

## Architecture Overview

```
┌────────────────────────────────────────────────────────────────────────────────┐
│                AI + FPGA Heterogeneous Tensor Acceleration Platform            │
│                                                                                │
│  ┌─────────────┐    APB Bus     ┌────────────────────────────────────────┐    │
│  │  Host CPU   │◄──────────────►│         APB Configuration             │    │
│  │  (AI Fwk)   │  precision     │  [2:0] precision_mode (7 precisions)  │    │
│  │  PyTorch /  │  matrix_mode   │  [3]   mixed_mode                     │    │
│  │  TensorFlow │  start         │  [5:4] matrix_mode (3 dimensions)     │    │
│  └──────┬──────┘                │  [8]   start                          │    │
│         │                       └──────────────────┬─────────────────────┘    │
│         │ AXI4                                     │                          │
│  ┌──────▼─────────────────────────────────────────────────────────────────┐   │
│  │                      FPGA Tensor Compute Unit                          │   │
│  │                                                                        │   │
│  │  ┌────────────┐   ┌────────────┐   ┌────────────────────────────┐    │   │
│  │  │ AXI Slave  │──►│ Data_Load  │──►│  Block RAM (36-bit Sparse) │    │   │
│  │  │ Controller │   │ (Precision │   │  A[32][16]  B[32][16]      │    │   │
│  │  │            │   │  Decode)   │   │  C[16][16]                 │    │   │
│  │  └────────────┘   └────────────┘   └──────────┬─────────────────┘    │   │
│  │                                               │                       │   │
│  │  ┌────────────────────────────────────────────▼────────────────────┐  │   │
│  │  │              FSM Controller (6-State)                           │  │   │
│  │  │  IDLE → LOAD_C → LOAD_A → LOAD_B → COMPUTE → OUTPUT           │  │   │
│  │  └─────────────────────────────┬──────────────────────────────────┘  │   │
│  │                                │                                     │   │
│  │  ┌─────────────────────────────▼──────────────────────────────────┐  │   │
│  │  │          8 × 8 Systolic PE Array (64 PEs)                      │  │   │
│  │  │                                                                 │  │   │
│  │  │  A→ [PE00]→[PE01]→[PE02]→[PE03]→[PE04]→[PE05]→[PE06]→[PE07]  │  │   │
│  │  │       ↓      ↓      ↓      ↓      ↓      ↓      ↓      ↓     │  │   │
│  │  │  A→ [PE10]→[PE11]→[PE12]→[PE13]→[PE14]→[PE15]→[PE16]→[PE17]  │  │   │
│  │  │       ↓      ↓      ↓      ↓      ↓      ↓      ↓      ↓     │  │   │
│  │  │      ...   ...    ...    ...    ...    ...    ...    ...        │  │   │
│  │  │  A→ [PE70]→[PE71]→[PE72]→[PE73]→[PE74]→[PE75]→[PE76]→[PE77]  │  │   │
│  │  │       ↑B     ↑B     ↑B     ↑B     ↑B     ↑B     ↑B     ↑B    │  │   │
│  │  │                                                                 │  │   │
│  │  │  Each PE: 5-Stage Pipeline                                      │  │   │
│  │  │  S1:Fetch → S2:Multiply → S3:Align → S4:Accumulate → S5:Norm   │  │   │
│  │  │  Sparse: bitmap==0 → bypass multiply (zero-latency)            │  │   │
│  │  └─────────────────────────────┬──────────────────────────────────┘  │   │
│  │                                │                                     │   │
│  │  ┌─────────────────────────────▼──────────────────────────────────┐  │   │
│  │  │  AXI Master Controller → 64-bit Output (packed 2×32-bit)       │  │   │
│  │  └────────────────────────────────────────────────────────────────┘  │   │
│  └────────────────────────────────────────────────────────────────────────┘   │
│                                                                                │
│  ┌────────────────────────────────────────────────────────────────────────┐   │
│  │  Sparse Acceleration (3 Switchable Modes)                              │   │
│  │  ┌────────────────┐ ┌──────────────────┐ ┌────────────────────────┐  │   │
│  │  │ Basic Sparse   │ │ Structured       │ │ Tile-Wise Sparse      │  │   │
│  │  │ (Element-wise) │ │ Pruning          │ │ (4×4 Sub-tile)        │  │   │
│  │  │                │ │ (Row/Col-level)  │ │                        │  │   │
│  │  └────────────────┘ └──────────────────┘ └────────────────────────┘  │   │
│  └────────────────────────────────────────────────────────────────────────┘   │
└────────────────────────────────────────────────────────────────────────────────┘
```

---

## Key Technical Details

### 1. Multi-Precision Support: FP32-to-INT4 Hardware Reuse

#### Supported Precisions

| Precision | `precision_mode` | Input Width | Accumulator | Elements/Word | Format |
|-----------|-----------------|-------------|-------------|---------------|--------|
| INT4 | `3'b000` | 4-bit | 64-bit | 8 | Signed fixed-point |
| INT8 | `3'b001` | 8-bit | 64-bit | 4 | Signed fixed-point |
| INT32 | `3'b101` | 32-bit | 32-bit | 1 | Signed integer |
| FP16 | `3'b010` | 16-bit | 16-bit | 2 | IEEE 754 half-precision |
| FP32 | `3'b011` | 32-bit | 32-bit | 1 | IEEE 754 single-precision |
| FP64 | — | 64-bit | 64-bit | — | IEEE 754 double-precision |
| BF16 | `3'b100` | 16-bit | 16-bit | 2 | Brain Floating Point |

Three **mixed-precision modes** are supported, with PE multipliers shared across precisions (FP16/FP32 reuse the same floating-point multiply unit):

| Mixed Mode | Input Precision | Accumulator | Application |
|-----------|----------------|-------------|-------------|
| INT4/INT8 + INT32 | 4/8-bit int | 32-bit int | Quantized inference (overflow prevention) |
| FP16 + FP32 + FP64 | 16-bit float | 32/64-bit float | High-precision training/inference |
| BF16 + FP32 | 16-bit BF16 | 32-bit float | Large dynamic range AI inference |

> **Standards Compliance**: Floating-point operations conform to **IEEE 754-2019** [1] with full Round-to-Nearest-Even rounding, NaN/Inf propagation, denormal number handling, and overflow/underflow detection. BF16 follows Google Brain's **Brain Floating Point** specification [2], aligned with NVIDIA Tensor Core 3rd-generation (Ampere) precision definitions [3]. The mixed-precision strategy draws from NVIDIA's Mixed-Precision Training methodology [4].

#### Hardware Reuse Mechanism

The key design principle is a **unified 36-bit datapath with precision-aware function dispatch**.

**Core floating-point formulas** (per IEEE 754-2019 [1]):

- **FP32 Multiply**: $(-1)^{s_a \oplus s_b} \times 2^{(e_a + e_b - 127)} \times (1.m_a \times 1.m_b)$ — 24×24-bit significand product with Round-to-Nearest-Even
- **FP16→FP32 Promotion**: $e_{\text{FP32}} = e_{\text{FP16}} + 112$ (bias delta: $127 - 15 = 112$); denormals via leading-zero count normalization
- **BF16→FP32 Promotion**: Direct upper-16-bit mapping ($\text{BF16} \equiv \text{FP32}[31{:}16]$, lower 16 bits zero-padded)
- **Integer Overflow Detection**: $\text{overflow} = (A[63] = B[63]) \wedge (A[63] \neq R[63])$ (same-sign inputs, opposite-sign result)

```
               32-bit AXI Data Input
                      │
          ┌───────────▼───────────┐
          │   Precision Decoder   │
          │   (safe_bit_select)   │
          │                       │
          │  mode=000 (INT4)      │──► Extract 8 × 4-bit  → int4_mul()
          │  mode=001 (INT8)      │──► Extract 4 × 8-bit  → int8_mul()
          │  mode=010 (FP16)      │──► Extract 2 × 16-bit → fp16→fp32 → fp32_mult()
          │  mode=011 (FP32)      │──► Use 32-bit directly → fp32_mult()
          │  mode=100 (BF16)      │──► Extract 2 × 16-bit → bf16→32bit → bf16_mult()
          └───────────────────────┘
                      │
          ┌───────────▼───────────┐
          │  36-bit Sparse Format │
          │  [35:32] Bitmap Meta  │ ← Validity flags
          │  [31:0]  Data Payload │ ← Unified data width
          └───────────────────────┘
                      │
          ┌───────────▼───────────┐
          │  Shared MAC Datapath  │
          │  (5-Stage Pipeline)   │
          │                       │
          │  Multiplier: mode-    │
          │  selected from shared │
          │  hardware resources   │
          │  (LUT-only, 0 DSP)   │
          │                       │
          │  Adder: similarly     │
          │  shared               │
          └───────────────────────┘
```

**Key reuse principles:**

1. **Data packing reuse**: A single 32-bit AXI bus carries 8 INT4 elements, 4 INT8 elements, 2 FP16/BF16 elements, or 1 FP32 element. The `Data_Load` module dynamically adjusts via `elements_per_word`.

2. **Multiplier reuse**: PEs select `int4_mul()` / `int8_mul()` / `fp32_mult()` / `bf16_mult()` based on `precision_mode`. All functions share the same pipeline register stages, differing only in operand extraction and result formatting. Synthesis attribute `use_dsp = "no"` forces **pure LUT implementation**, freeing all DSP48E2 blocks for other accelerators.

3. **Accumulator reuse**: Integer modes use 64-bit accumulators (preventing INT4 accumulation overflow); floating-point modes use precision-specific FP adders. Accumulators are initialized with matrix $C$ bias values during the COMPUTE phase.

4. **Zero-overhead runtime switching**: Precision is changed by writing the `precision_mode` APB register — effective on the next GEMM launch with **no bitstream reconfiguration or hardware restart**.

---

### 2. Sparse Matrix Acceleration: Bitmap Encoding

#### Sparse GEMM Formulation

With sparsity masks $m_A$ and $m_B$ on matrices $A$ and $B$, the GEMM reduces to:

$$D_{ij} = \sum_{k=0}^{K-1} m_{A}(i,k) \cdot m_{B}(k,j) \cdot A_{ik} \cdot B_{kj} + C_{ij}$$

where $m_A(i,k), m_B(k,j) \in \{0, 1\}$ are bitmap masks. When either mask is 0, the corresponding multiply is bypassed with zero latency.

#### Encoding Format

The design adopts a **Bitmap sparse encoding** rather than traditional CSR/CSC. Rationale:

- **Hardware-friendly**: Fixed format, no complex pointer dereferencing
- **Parallelism**: Per-element validity is independently evaluated, ideal for systolic arrays
- **Low overhead**: Only 4-bit metadata per element vs. CSR's row-pointer + column-index

```
36-bit Sparse Data Format:
┌──────────────────────────────────────────┐
│ [35:32]  Bitmap Metadata (4-bit)         │  ← Element validity flags
│ [31:0]   Data Payload (32-bit)           │  ← Actual numeric value
└──────────────────────────────────────────┘

Bitmap[i] = 1  →  Non-zero: participates in MAC
Bitmap[i] = 0  →  Zero: PE bypasses multiply, forwards partial sum
```

In the sparse variant, each BRAM word is extended to **37 bits** (`[36:0]`): the upper 5 bits (`[36:32]`) store the **column/row index** for sparse element addressing. Per-row/column write pointers (`ptr_a0`–`ptr_a31`, `ptr_b0`–`ptr_b31`) implement compressed storage where only non-zero elements occupy memory.

#### Zero-Skip Mechanism

```systemverilog
// PE sparse bypass (simplified)
if (a_preprocessed == 0 || b_preprocessed == 0) begin
    int_product  <= 0;
    fp32_product <= 0;
    bf16_product <= 0;  // bypass multiply — zero latency
end else begin
    // execute MAC per precision_mode
    int_product  <= int4_mul(a, b) | int8_mul(a, b);
    fp32_product <= fp32_mult(a, b);
    bf16_product <= bf16_mult(a, b);
end
```

#### Three Sparse Acceleration Modes

| Mode | Module | Granularity | Application | Speedup (50% sparsity) |
|------|--------|------------|-------------|------------------------|
| Basic Sparse | `TOP_TPU_Sparse_Matrix` | Element-wise | General unstructured sparsity | ~1.8× |
| Structured Pruning | `Pruning_Sparse_Matrix` | Row/Column-level | Pruned DNN weights | ~2.0× |
| Tile-Wise Sparse | `Tile_Wise_Sparse_Matrix` | 4×4 Sub-tile | Block-sparse Transformers | ~1.9× |

#### Speedup Analysis

Theoretical speedup is determined by sparsity ratio $S$ (fraction of zero elements):

$$\text{Speedup}_{\text{ideal}} = \frac{1}{1-S}$$

Actual speedup is bounded by Amdahl's Law — data loading and FSM control are non-parallelizable:

$$\text{Speedup}_{\text{actual}} = \frac{T_{\text{load}} + T_{\text{compute}}}{T_{\text{load}} + (1-S) \cdot T_{\text{compute}}}$$

where $T_{\text{load}} = M \cdot N + M \cdot K + K \cdot N$ cycles (loading C/A/B), $T_{\text{compute}} = K + 27$ cycles (including pipeline flush). For a 16×16×16 matrix:

| Sparsity $S$ | Theoretical | Measured | Efficiency |
|-------------|-------------|----------|------------|
| 50% | 2.0× | ~1.8× | 90% |
| 75% | 4.0× | ~3.2× | 80% |

---

### 3. Systolic Array: Resources & Performance

#### 8×8 Array Specifications

| Metric | Value | Notes |
|--------|-------|-------|
| PE Count | 64 (8×8) | 2D output-stationary systolic array |
| Pipeline Depth | 5 stages/PE | Fetch → Multiply → Align → Accumulate → Normalize |
| Compute Latency | $K + P_R + P_C - 1 + 4$ cycles | K=reduction dim, $P_R$/$P_C$=8 (array dims), 4=pipeline overhead |
| Peak Throughput | 64 MAC/cycle | Equiv. 512 INT4-MAC/cycle (8 elements/word) |
| Data Flow | Output Stationary | A flows horizontally, B flows vertically |

#### FPGA Resource Utilization (Xilinx XCVU9P — Post-Implementation)

| Resource | Used | Available | Utilization | Notes |
|----------|------|-----------|-------------|-------|
| LUT | 92,722 | 1,182,240 | **7.84%** | Multi-precision multipliers, adders, control |
| FF | 46,307 | 2,364,480 | **1.96%** | Pipeline registers, FSM state |
| DSP48E2 | **0** | 6,840 | **0%** | `use_dsp = "no"` — pure LUT arithmetic |
| IO | 187 | 832 | **22.48%** | AXI4 + APB interface signals |

> **Design Decision**: DSP slices are intentionally unused. All arithmetic is implemented in LUT fabric, leaving DSP resources available for co-located accelerators (e.g., convolution engines) in heterogeneous SoC designs.

#### Timing

| Metric | Value |
|--------|-------|
| Fmax (Post-Implementation) | **214.6 MHz** |
| WNS (Worst Negative Slack) | **0.34 ns** (timing closure achieved) |
| TNS (Total Negative Slack) | 0 ns |
| Failing Endpoints | 0 / 35,927 |
| Critical Path | FP32 multiplier → alignment adder (Stage 2→3) |
| GEMM Latency | **163.1 ns** (single tile) |

#### Power Analysis

| Component | Power | Share |
|-----------|-------|-------|
| **Total On-Chip** | **3.663 W** | 100% |
| Dynamic | 1.177 W | 32% |
| — Clocks | 0.130 W | 11% |
| — Signals | 0.474 W | 40% |
| — Logic | 0.531 W | 45% |
| — I/O | 0.041 W | 4% |
| Static | 2.486 W | 68% |

**Sparse Mode Power Optimization:**

| Configuration | Fmax | LUT | Power | Dynamic Power |
|--------------|------|-----|-------|---------------|
| Dense (Baseline) | 214.6 MHz | 8% | 3.663 W | 1.177 W |
| 4:2 Structured Sparse | 206.53 MHz | 7% | 2.798 W | **reduced to 1/4** |
| Tile-wise + Pruning + BF16 Quant. | — | — | — | **down to 0.276 W** |

#### Performance Summary

| Metric | Value |
|--------|-------|
| Theoretical Throughput | **27.47 GOPS** |
| Simulated Throughput | **12.3 GOPS** |
| Memory Bandwidth | **3.2 GB/s** |
| Theoretical Energy Efficiency | **7.50 GOPS/W** |
| Simulated Energy Efficiency | **3.36 GOPS/W** |
| GEMM Latency | **163.1 ns** |

---

### 4. Bus Interfaces

| Interface | Protocol | Data Width | Addr Width | Function |
|-----------|----------|-----------|------------|----------|
| Data Input | AXI4 Slave | 32-bit | 32-bit | Receive matrices A/B/C |
| Result Output | AXI4 Master | 64-bit | 32-bit | Write back result D (dual-word packed) |
| Configuration | APB | 9-bit | — | Precision/dimension/mode/start control |

**AXI Slave Memory Map** (via `mem_sel[1:0]`):

| `mem_sel` | Target | Size |
|-----------|--------|------|
| `2'b00` | Matrix A | up to 32×16 |
| `2'b01` | Matrix B | up to 16×32 |
| `2'b10` | Matrix C | up to 16×16 |

### 5. Matrix Dimension Configurations

| Mode | Dimensions | Tiling Strategy | Application |
|------|-----------|----------------|-------------|
| `m8n32k16` | 8×32×16 | 1×4 (wide output) | FC layers (few inputs, many outputs) |
| `m16n16k16` | 16×16×16 | 2×2 (balanced) | General GEMM |
| `m32n8k16` | 32×8×16 | 4×1 (tall input) | Batch inference (large batch, few classes) |

Maximum dimensions: M=32, N=32, K=16. Larger matrices are tiled via 4 `pe_counter` iterations.

---

## FSM Control Flow

```
┌───────┐   start    ┌────────┐  M·N cycles  ┌────────┐  M·K cycles  ┌────────┐
│ IDLE  │──────────►│ LOAD_C │────────────►│ LOAD_A │────────────►│ LOAD_B │
└───────┘           └────────┘             └────────┘             └────┬───┘
    ▲                                                                  │
    │                                                            K·N cycles
    │               ┌────────┐  64 cycles   ┌─────────┐               │
    └───────────────│ OUTPUT │◄────────────│ COMPUTE │◄──────────────┘
      pe_counter    └────────┘  per tile    └─────────┘  K+27 cycles
      done (=4)                              (incl. pipeline flush)
```

---

## Comparison with Related Work

| Metric | **This Work** | NVDLA Small [5] | Xilinx DPU B1024 [6] | Gemmini [7] |
|--------|---------------|-----------------|----------------------|-------------|
| Platform | XCVU9P (VCU118) | ASIC (synthesis) | XCZU9EG (ZCU102) | XCVU9P |
| Array Size | 8×8 PE | 8×8 MAC | 1024 OPs | 16×16 PE |
| Precisions | **7** (INT4–FP64) | INT8/INT16/FP16 | INT8 only | INT8/FP16 |
| Mixed-Precision | **3 modes** | None | None | None |
| Sparse Accel. | **3 modes** | None | None | None |
| Fmax | **214.6 MHz** | — | 330 MHz | 200 MHz |
| LUT Utilization | 7.84% | — | ~70% | ~15% |
| DSP Usage | **0** (pure LUT) | — | Heavy | Heavy |
| Energy Eff. | 3.36 GOPS/W (sim) | ~5 TOPS/W (ASIC) | 2.37 TOPS/W | — |
| Runtime Precision Switch | **Zero-overhead** | Reconfigure | Not supported | Not supported |

> **Positioning**: NVDLA is an ASIC reference design with inherent process-level efficiency advantages. Xilinx DPU targets deployment with heavy DSP utilization. This work emphasizes **precision flexibility** (7 precisions + 3 mixed modes), **sparse acceleration** (3 modes), and **zero DSP usage**, making it ideal for heterogeneous FPGA resource sharing.

---

## Verification

### Test Coverage

| Category | Testbenches | Coverage |
|----------|------------|---------|
| Full-Precision GEMM | 15 | FP32/FP16/BF16/INT8/INT4 × 3 matrix dimensions |
| Mixed-Precision | 4 | INT4→INT32, INT8→INT32 accumulation |
| Sparse Acceleration | 5 | Basic/Pruning/Tile-wise × multiple sparsity levels |
| Component-Level | 4 | FP32 adder, multiplier, PE unit, data loader |
| **Total** | **28** | Full precision × dimension × mode coverage |

### Running Simulations

1. Open the Vivado project (`.srcs` directories under `源代码/复赛提交包/`)
2. Load test vectors from `测试数据/testcase/` for the target precision
3. Select the appropriate testbench and run behavioral simulation
4. Verify output matrix $D$ matches expected results

### Test Data Format

File naming: `{matrix}_{precision}_{dimension}.mem.txt`

- `a_` — Input matrix A
- `b_` — Input matrix B
- `c_` — Bias matrix C
- `d_` — Expected output matrix D

---

## Limitations & Future Work

- **Timing margin**: WNS = 0.34 ns provides limited margin; sustained operation at 200 MHz may require further optimization.
- **On-board verification**: Current results are post-implementation simulation only. Future work includes bitstream deployment on VCU118 with ILA waveform capture for physical validation.
- **Energy efficiency**: Optimizing the critical path (FP32 multiply → alignment) and exploring DSP-LUT hybrid arithmetic could improve both Fmax and energy efficiency.

---

## References

1. IEEE Std 754-2019, *IEEE Standard for Floating-Point Arithmetic*, IEEE, 2019.
2. Google Brain, "BFloat16: The secret to high performance on Cloud TPUs," 2019.
3. NVIDIA, "NVIDIA A100 Tensor Core GPU Architecture," Whitepaper, 2020.
4. P. Micikevicius et al., "Mixed Precision Training," *ICLR*, 2018.
5. NVIDIA, "NVDLA Open Source Project," http://nvdla.org, 2018.
6. Xilinx, "Vitis AI DPU Architecture," UG1414, 2023.
7. H. Genc et al., "Gemmini: Enabling Systematic Deep-Learning Architecture Evaluation via Full-Stack Integration," *DAC*, 2021.

---

## Award

<p align="center">
  <img src="获奖证书/集创赛 获奖证书.jpg" width="600">
</p>

---

## License

This project is a competition entry intended for academic and educational purposes only.
