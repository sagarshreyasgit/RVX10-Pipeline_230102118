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

This project’s design and pipeline architecture are based on:

> **Digital Design and Computer Architecture (RISC-V Edition)**  
> **David Harris and Sarah Harris**

> **CS322M - Digital Design And Computer Architecture**

---

## 🏫 Acknowledgment

Developed under the guidance of  
**Dr. Satyajit Das**  
_Assistant Professor_  
Department of **Computer Science and Engineering**  
**Indian Institute of Technology, Guwahati**
