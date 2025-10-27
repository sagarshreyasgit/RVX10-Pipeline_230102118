# ⚙️ RVX10-Pipline: Five-Stage Pipelined RISC-V Core  

**RVX10-P** is a **five-stage pipelined RISC-V processor (RV32I)** extended with **10 custom ALU instructions** under the **RVX10 extension**.  
Developed as part of the course **Digital Logic and Computer Architecture** taught by **Dr. Satyajit Das**, **IIT Guwahati**.  

---

## 🚀 Overview  

This project transforms a single-cycle RISC-V (RVX10) processor into a **five-stage pipelined architecture**, achieving improved throughput and instruction-level parallelism.  

The design follows the standard **five pipeline stages**:  

> **IF → ID → EX → MEM → WB**  

Each stage is separated by dedicated **pipeline registers**, while **forwarding** and **hazard detection** units ensure correct execution under data and control dependencies.  

---

## 🧩 Key Features  

### 🧠 Architecture  
- **5-Stage Pipeline:** IF, ID, EX, MEM, WB  
- **Pipeline Registers:** IF/ID, ID/EX, EX/MEM, MEM/WB  
- **ISA Coverage:** Complete support for **RV32I** base instructions  
- **RVX10 Extension:** Adds 10 new ALU operations  
  > ANDN, ORN, XNOR, MIN, MAX, MINU, MAXU, ROL, ROR, ABS  

### 🔁 Hazard Handling  
- **Forwarding Unit:**  
  Resolves **data hazards (RAW)** by forwarding results from **MEM** and **WB** stages to the EX stage.  
- **Hazard Detection Unit:**  
  Detects **load-use hazards**, introduces a one-cycle stall, and performs **pipeline flushing** for taken branches and jumps.  
- **Branch Flush Logic:**  
  Injects NOPs automatically to maintain pipeline correctness.  

### ⚡ Performance  
- Average **CPI ≈ 1.28** on standard test programs.  
- Executes multiple instructions concurrently.  
- Produces identical results as the single-cycle core (final memory write **25** at address **100**).  
- Register `x0` remains hardwired to zero.  

---

## 🧱 Core Block Diagram  
![Pipeline Diagram](https://github.com/user-attachments/assets/0296251d-c06e-440d-a48d-3899437b4aa2)

---

## 📂 Repository Structure
├── src/
│   ├── riscvpipeline.sv      # Top-level core
│   ├── datapath.sv           # Pipelined datapath
│   ├── controller.sv         # Main controller
│   ├── forwarding_unit.sv    # Handles data hazards
│   ├── hazard_unit.sv        # Detects hazards
│   ├── alusv.sv              # ALU (User's 'alu.sv')
│   ├── regfile.sv            # Register File
│   ├── imem.sv               # Instruction Memory
│   ├── dmem.sv               # Data Memory
│   ├── pipe_IF_ID.sv         # Pipeline Register IF/ID
│   ├── pipe_ID_EX.sv         # Pipeline Register ID/EX
│   ├── pipe_EX_MEM.sv        # Pipeline Register EX/MEM
│   ├── pipe_MEM_WB.sv        # Pipeline Register MEM/WB
│   ├── ctrl_ID_EX.sv         # Control pipeline register
│   ├── ctrl_EX_MEM.sv        # Control pipeline register
│   ├── ctrl_MEM_WB.sv        # Control pipeline register
│   ├── aludec.sv             # ALU Decoder
│   ├── maindec.sv            # Main Decoder
│   ├── adder.sv
│   ├── extend.sv
│   ├── mux2.sv
│   ├── mux3.sv
│   ├── flop.sv
│   └── top.sv
│
├── tb/
│   ├── testbench.sv          # Testbench (User's 'tb_pipeline.sv')
│   ├── rvx10_pipeline.hex    # Test program (memory image)
│   └── riscvtest.mem         # Memory initialization file
│
└── docs/
    ├── TESTPLAN.md           # Documentation (similar to REPORT.md)
    ├── encoding.md           # Documentation
    └── waveforms/
        ├── Bonus_Performance-Execution_results.png
        ├── Dmem_store_25.png
        ├── Final_CPI.png
        ├── Register_file_1.png
        ├── Register_file_2.png
        ├── Simulation_succeded.png
        └── Value_25_at_address_100.png
## 🧪 How to Run  

### 🖥️ Option 1: Using Vivado  

1. Create a **new project** in **Vivado**.  
2. Add all `.sv` files from the `/src` folder as **Design Sources**.  
3. Add `tb_pipeline.sv` from `/tb` as a **Simulation Source**.  
4. Add `riscvtest.mem` (or `rvx10_pipeline.hex`) as a memory initialization file.  
5. Run **Behavioral Simulation**.  

### 💻 Option 2: Using Icarus Verilog / Verilator  

1. Install a SystemVerilog simulator (e.g., **Icarus Verilog** or **Verilator**).  
2. Run the following commands:  

```bash
iverilog -g2012 -o pipeline_sim tb/tb_pipeline.sv src/*.sv
vvp pipeline_sim
```
---
## 📈 Observations
- Load-Use Hazard: One-cycle bubble verified.
- Forwarding: Correctly resolves RAW hazards.
- Branch Flush: IF/ID and ID/EX registers flushed on taken branch.
- Pipeline Overlap: Multiple instructions active each cycle (verified via waveforms).
---
## 📚 References
Digital Design and Computer Architecture (RISC-V Edition)
David Harris & Sarah Harris

IIT Guwahati – CS322M: Digital Logic and Computer Architecture
Dr. Satyajit Das, Assistant Professor
---
## 🏫 Acknowledgment
Developed under the guidance of
Dr. Satyajit Das
Assistant Professor
Department of Computer Science and Engineering
Indian Institute of Technology, Guwahati
