# MINI-RISC-Pipeline

A hardware implementation of a 4-stage pipelined RISC processor designed in synthesizable Verilog. This architecture optimizes instruction throughput by implementing dedicated hardware hazard mitigation and data forwarding paths to handle control and data dependencies cleanly.

## Key Features

* **4-Stage Pipeline**: Parallel processing across specialized functional stages to maximize execution throughput.
* **Full Data Forwarding**: Dedicated Forwarding Unit to resolve data hazards dynamically without stalling when possible.
* **Comprehensive Hazard Management**: Integrated Hazard Unit executing precise flushing and stalling operations.
* **Dedicated I/O Interface**: Integrated I/O Port 1 and Port 2 for direct peripheral interaction.

## Repository Structure

```text
├── src/               # Synthesizable Verilog source files (.v) and Verification Testbenches
└── README.md          # Project documentation
```

## 📊 Microarchitecture & Pipeline Stages

![MINI-RISC Microarchitecture Diagram](Pipeline_microarchitecture.png)

The processor separates instruction execution into four explicit stages divided by distinct pipeline registers:
[ Fetch ] ──▶ (F/D Register) ──▶ [ Decode ] ──▶ (D/E Register) ──▶ [ Execute ] ──▶ (E/W Register) ──▶ [ Writeback/Mem ]
1. **Fetch (F)**
   * Utilizes a Program Counter (PC) with an incremental addition loop.
   * Pulls instructions from the dedicated **Instruction Memory** unit.
   * Can be stalled (`Stall_F`) or cleared (`Flush_F`) depending on pipeline hazards.

2. **Decode (D)**
   * Separated by the **F/D Register**.
   * Contains the centralized **Control Unit** for decoding operation codes (`opcode_D`).
   * Includes the **Register File** (`8 × 16-bit`) with dual source registers (`rs1_D`, `rs2_D`).
   * Evaluates branch logic using a dedicated **Branch Register** and calculates target branch addresses (`Branch_addr`).

3. **Execute (EX)**
   * Separated by the **D/E Register**.
   * Features an internal **ALU** (Arithmetic Logic Unit) driven by multiplexed operands.
   * Interacts with local **Data Memory** (RAM) block for storage operations.

4. **Writeback / Memory (WB)**
   * Separated by the **E/W Register**.
   * Commits computed results or loaded memory data directly back to destination registers (`rd_E`).
   * Integrates an independent dual **I/O Port** block (`I/O Port 1` & `I/O Port 2`) for external peripheral data mapping.

## Hazard & Control Mechanics

The design features specialized hardware protection modules to guarantee deterministic operation under conflicts:

* **Forwarding Unit**: Tracks source registers against downstream write-back configurations to feed operand results directly back to the ALU inputs, eliminating data delays.
* **HAZARD Unit**: Monitors pipeline dependencies continuously. It directly manages pipeline control lines:
  * `Stall_F` / `Stall_D`: Holds instruction steps when data dependencies cannot be resolved by forwarding alone.
  * `Flush_F` / `Flush_D`: Flushes instruction lines immediately following an invalid branch prediction or control change.
## Synthesis & Timing Results

The same RTL was synthesized and timed in two tools to compare mapping and timing.

### Vivado (Basys 3, Artix-7 `xc7a35tcpg236-1`), post-implementation, 100 MHz

| Metric | Result |
|---|---|
| Clock constraint | 10 ns (100 MHz) |
| Setup WNS | +0.585 ns |
| Hold WHS | +0.047 ns |
| Failing endpoints | 0 |
| Slice LUTs | 1305 (6%), of which 704 used as distributed RAM |
| Slice registers | 328 (<1%) |
| F7 muxes | 64 |
| DSP48 | 1 (16×16 `MUL`) |
| Block RAM | 0 |
| Bonded IOs | 34 |

Registers by block: register file 128, D/E 85, E/W 71, F/D 12, PC 5.
Critical path: `DE_Reg/immediate_out` to the ALU result and `EW_Reg` write data,
6-7 logic levels, ~9.2 ns (4.9 ns logic, 4.3 ns routing).

### Quartus Prime (Cyclone V 5CSEMA5F31C6), 50 MHz constraint (With DIV Operation)
### Resource Utilization (`Fitter Summary`)

| Resource | Used | Available | Utilization |
| :--- | :--- | :--- | :--- |
| **Logic Utilization (ALMs)** | 954 | 32,070 | **3%** |
| **Dedicated Logic Registers** | 394 | 64,140 | **< 1%** |
| **M10K Block Memory** | 8 blocks (65,536 bits) | 397 blocks (4,065,280 bits) | **2%** |
| **DSP Blocks** | 1 | 87 | **1%** |
| **I/O Pins** | 67 | 457 | **15%** |
| **Average Interconnect Usage** | — | — | **1.2%** |

* **Memory Architecture:** Inferred **8 M10K blocks** (`65,536 bits` total) supporting separate 16-bit Instruction ROM and Data RAM blocks.
* **Arithmetic Acceleration:** Inferred **1 dedicated DSP block** for 16×16 multiplication (`MUL`).

### Multi-Corner Static Timing Analysis (`50 MHz` Target Clock)

| PVT Corner | Worst-Case Setup Slack | Worst-Case Hold Slack | Status |
| :--- | :--- | :--- | :--- |
| **Slow 1100mV 85°C** *(Worst Setup)* | **+10.044 ns** | +0.609 ns | **PASS (~100.4 MHz Fmax)** |
| **Slow 1100mV 0°C** | +10.077 ns | +0.581 ns | **PASS** |
| **Fast 1100mV 85°C** | +13.627 ns | +0.242 ns | **PASS** |
| **Fast 1100mV 0°C** *(Worst Hold)* | +14.159 ns | **+0.220 ns** | **PASS** |

* **Maximum Operating Frequency (`Fmax`):** **~100.4 MHz** (`9.956 ns` critical path delay at Slow 1100mV 85°C), providing over **2× timing margin** at the DE1-SoC's 50 MHz onboard clock.
### DE1-SoC Board Hardware Mapping (`processor_top`)

Because the processor uses a 16-bit `input_reg` and 16-bit `output_reg` while the DE1-SoC provides 10 slide switches (`SW[9:0]`), 4 push buttons (`KEY[3:0]`), and 6 seven-segment displays (`HEX0`–`HEX5`), the top-level wrapper implements **time-multiplexed byte loading** and **hexadecimal 7-segment display decoding**:

| Board Peripheral | Signal | Function |
| :--- | :--- | :--- |
| **Slide Switches `[7:0]`** | `SW[7:0]` | 8-bit input data byte to load into `input_reg` |
| **Slide Switch `8`** | `SW[8]` | Byte select (`0` = lower byte `[7:0]`, `1` = upper byte `[15:8]`) |
| **Slide Switch `9`** | `SW[9]` | Execution mode (`0` = single-step via `KEY[2]`, `1` = continuous `50 MHz` clock) |
| **Push Button `0`** | `KEY[0]` | Active-low synchronous/asynchronous system reset (`rst = ~KEY[0]`) |
| **Push Button `1`** | `KEY[1]` | Latch `SW[7:0]` into the selected byte of `input_reg` (edge-detected) |
| **Push Button `2`** | `KEY[2]` | Manual single-step clock pulse when `SW[9] = 0` (edge-detected) |
| **7-Segment `HEX3`–`HEX0`** | `HEX[3:0]` | 16-bit hexadecimal readout (`0000`–`FFFF`) of `output_reg` / `input_reg` |
| **7-Segment `HEX5`–`HEX4`** | `HEX[5:4]` | Mode indicator (`Ot` = viewing `output_reg`, `In` = previewing `input_reg`) |
| **Red LEDs `[9:0]`** | `LEDR[9:0]` | Direct visual feedback of switch states (`SW[9:0]`) |


### Timing closure history

The first Vivado run at 100 MHz failed with WNS -26.879 ns (about 27 MHz).
The critical path had 84 logic levels and ended at `alu_result_1`: the
combinational 16-bit `DIV`/`MOD` in the ALU. With `DIV` removed from the
single-cycle ALU, timing closed with +0.585 ns slack.

## Known Limitations / Future Work

- **DIV/MOD** is stubbed out. Planned fix: a 16-cycle iterative restoring
  divider that stalls the pipeline.
- **I/O timing** (`input_reg`, `output_reg`) is not yet constrained.

## 🛠️ Getting Started

### Tools Needed
* **Simulation**: Xilinx Vivado
* **Synthesis and Implementation**: Xilinx Vivado/ Intel Quartus Prime
## 📄 License
This project is licensed under the MIT License.
