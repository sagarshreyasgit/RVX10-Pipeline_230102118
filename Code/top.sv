`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.10.2025 21:39:49
// Design Name: 
// Module Name: top
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module top (
  input  logic       clk,
  input  logic       reset,
  
  // --- Testbench Monitoring Ports ---
  // These outputs allow the testbench to check the final memory write
  output logic [31:0] WriteData,
  output logic [31:0] DataAdr,
  output logic       MemWrite
);

  // --- Wires to connect core and memories ---
  
  // --- Core <--> Instruction Memory ---
  logic [31:0] PC_F;      // PC from core to imem address
  logic [31:0] Instr_F; // Instruction from imem to core

  // --- Core <--> Data Memory ---
  logic [31:0] MemReadData_M;  // Data from dmem to core

  // 1. Instantiate the Pipelined Processor Core
  //    This is the 'riscvpipeline.sv' file from the previous step.
  riscvpipeline core_inst (
    .clk(clk),
    .reset(reset),
    .PC_F(PC_F),
    .Instr_F(Instr_F),
    .MemReadData_M(MemReadData_M),
    .MemWrite_M(MemWrite),
    .MemAddr_M(DataAdr),
    .MemWriteData_M(WriteData)
  );

  // 2. Instantiate the Instruction Memory
  imem imem_inst (
    .a(PC_F),     // Address comes from the core's PC
    .rd(Instr_F)  // Instruction goes back to the core
  );

  // 3. Instantiate the Data Memory
  dmem dmem_inst (
    .clk(clk),
    .we(MemWrite), // Write enable from core
    .a(DataAdr),      // Address from core
    .wd(WriteData),     // Write data from core
    .rd(MemReadData_M)    // Read data back to core
  );

endmodule

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.10.2025 21:33:04
// Design Name: 
// Module Name: riscvpipeline
// Project Name: RVX10-P
// Target Devices: 
// Tool Versions: 
// Description: Top-level pipelined RISC-V processor core.
//              This module instantiates and connects the datapath, 
//              controller, forwarding unit, and hazard detection unit.
// 
// Dependencies: datapath.sv, controller_pipelined.sv, 
//               forwarding_unit.sv, hazard_unit.sv
// 
//////////////////////////////////////////////////////////////////////////////////

module riscvpipeline (
  // --- Global Inputs ---
  input  logic       clk,
  input  logic       reset,

  // --- Instruction Memory Interface ---
  output logic [31:0] PC_F,    // PC address to imem
  input  logic [31:0] Instr_F, // Instruction from imem

  // --- Data Memory Interface ---
  input  logic [31:0] MemReadData_M, // Data from dmem
  output logic       MemWrite_M,    // Write enable to dmem
  output logic [31:0] MemAddr_M,     // Address to dmem
  output logic [31:0] MemWriteData_M // Write data to dmem
);

  // --- Internal Wires: Control Signals ---
  logic [1:0] ResultSrc_W;
  logic       PCSrc_E;
  logic       ALUSrc_E;
  logic       RegWrite_W, RegWrite_M;
  logic [1:0] ImmSrc_D;
  logic [4:0] ALUControl_E; // 5-bits for RVX10
  logic       Zero_E;
  logic       ResultSrc_E_0;
  logic       Jump_E, Branch_E;

  // --- Internal Wires: Hazard Unit ---
  logic       stall_F, stall_D;
  logic       flush_D, flush_E;
  logic [1:0] FwdSel_A, FwdSel_B;

  // --- Internal Wires: Datapath Feedback ---
  logic [31:0] Instr_D;
  logic [4:0]  rs1_D, rs2_D;
  logic [4:0]  rs1_E, rs2_E, rd_E, rd_M, rd_W;
 



  // 
  // --- 1. Datapath Instantiation ---
  // 
  datapath processor_datapath (
    .clk(clk), //
    .reset(reset), //
    .ResultSrc_W(ResultSrc_W), //
    .PCSrc_E(PCSrc_E), //
    .ALUSrc_E(ALUSrc_E), //
    .RegWrite_W(RegWrite_W), //
    .ImmSrc_D(ImmSrc_D), //
    .ALUControl_E(ALUControl_E), //
    .stall_F(stall_F), //
    .stall_D(stall_D), //
    .flush_D(flush_D), //
    .flush_E(flush_E), //
    .FwdSel_A(FwdSel_A), //
    .FwdSel_B(FwdSel_B), //
    .Instr_F(Instr_F), //
    .MemReadData_M(MemReadData_M), // 
    .Zero_E(Zero_E), //
    .Instr_D(Instr_D), //
    .rs1_D(rs1_D), //
    .rs2_D(rs2_D), //
    .rs1_E(rs1_E), //
    .rs2_E(rs2_E), //
    .rd_E(rd_E), //
    .rd_M(rd_M), //
    .rd_W(rd_W), //
    .PC_F(PC_F), //
    .MemAddr_M(MemAddr_M), //
    .MemWriteData_M(MemWriteData_M)
  );

  // 
  // --- 2. Controller Instantiation ---
  // 
  controller processor_controller (
    .clk(clk), //
    .reset(reset), //
    .op(Instr_D[6:0]), //
    .funct3(Instr_D[14:12]), //
    .funct7(Instr_D[31:25]), //
    .funct7b5(Instr_D[30]), //
    .Zero_E(Zero_E), //
    .Flush_E(flush_E), // Fix capitalization
    .ImmSrc_D(ImmSrc_D), //
    .PCSrc_E(PCSrc_E), //
    .ALUSrc_E(ALUSrc_E), //
    .ALUControl_E(ALUControl_E), //
    .MemWrite_M(MemWrite_M), //
    .RegWrite_W(RegWrite_W), //
    .ResultSrc_W(ResultSrc_W), //
    .RegWrite_M(RegWrite_M), //
    .ResultSrcE0(ResultSrc_E_0) // This is for hazard detection //
  );

  // 
  // --- 3. Forwarding Unit Instantiation ---
  // 
  forwarding_unit fwd_unit (
    .rs1_E(rs1_E),
    .rs2_E(rs2_E),
    .rd_M(rd_M),
    .rd_W(rd_W),
    .RegWrite_M(RegWrite_M),
    .RegWrite_W(RegWrite_W),
    .FwdSel_A(FwdSel_A),
    .FwdSel_B(FwdSel_B)
  );

  // 
  // --- 4. Hazard Detection Unit Instantiation ---
  // 
  hazard_unit hazard_detect (
    .rs1_D(rs1_D),
    .rs2_D(rs2_D),
    .rd_E(rd_E),
    .ResultSrc_E_0(ResultSrc_E_0), // 1 if EX is 'lw'
    .PCSrc_E(PCSrc_E),         // 1 if branch taken or jump
    .stall_F(stall_F),
    .stall_D(stall_D),
    .flush_D(flush_D),
    .flush_E(flush_E)
  );


  // 
  // --- (Optional) Performance Counters ---
  // 
  logic [31:0] total_cycles;
  logic [31:0] retired_instructions;

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      total_cycles         <= 32'b0;
      retired_instructions <= 32'b0;
    end 
    else begin
      // Cycle counter always increments
      total_cycles <= total_cycles + 1;

      // Increment instruction counter ONLY if a *valid* instruction
      // retires from the WB stage. This correctly counts all
      // instructions (R-type, loads, stores, branches) and
      // ignores flushed NOPs (bubbles).
      if (RegWrite_W) begin
        retired_instructions <= retired_instructions + 1;
      end
    end
  end

endmodule


module controller (
  input  logic       clk, reset,
  
  // --- Inputs from ID Stage (Instruction) ---
  input  logic [6:0] op, //
  input  logic [2:0] funct3, //
  input  logic [6:0] funct7, //      // Full funct7 for RVX10
  input  logic       funct7b5,
  // --- Inputs from Datapath / Hazard Unit ---
  input  logic       Zero_E,       // Zero flag from ALU in EX stage
  input  logic       Flush_E,      // Flush signal from Hazard Unit
  output logic ResultSrcE0, //
  // --- Control Outputs to ID Stage ---
  output logic [1:0] ImmSrc_D, //
  
  // --- Control Outputs to EX Stage ---
  output logic       PCSrc_E, //
  output logic       ALUSrc_E, //
  output logic [4:0] ALUControl_E, // 5-bits for RVX10 //
  
  // --- Control Outputs to MEM Stage ---
  output logic       MemWrite_M, //
  
  // --- Control Outputs to WB Stage ---
  output logic       RegWrite_W, //
  output logic [1:0] ResultSrc_W, //
  
  // --- Outputs to Hazard/Forwarding Units ---
  // (Signals are latched from internal registers)
  output logic       RegWrite_M // To Forwarding Unit //
);
  
  //--- Internal Wires for Decode (ID) Stage ---
  logic [1:0] ALUOp_D;
  logic [1:0] ResultSrc_D;
  logic [4:0] ALUControl_D;
  logic       Branch_D, MemWrite_D, Jump_D;
  logic       ALUSrc_D, RegWrite_D;

  //--- Internal Wires for Execute (EX) Stage ---
  logic [1:0] ResultSrc_E; // Renamed to avoid port conflict
  logic       Branch_E, MemWrite_E, Jump_E;

  //--- Internal Wires for Memory (MEM) Stage ---
  logic [1:0] ResultSrc_M;
  logic       RegWrite_E;

  //--- Decoder Instantiation (in ID stage) ---
  
  // Main Decoder
  maindec mdec (
    .op(op),
    .ResultSrc(ResultSrc_D),
    .MemWrite(MemWrite_D),
    .Branch(Branch_D),
    .ALUSrc(ALUSrc_D),
    .RegWrite(RegWrite_D),
    .Jump(Jump_D),
    .ImmSrc(ImmSrc_D),
    .ALUOp(ALUOp_D)
  );

  // ALU Decoder (using your 5-bit control version)
  aludec adec (
    .opb5(op[5]),
    .funct3(funct3),
    .funct7b5(funct7b5),      // Bit 5 of funct7 for SUB
    .funct7(funct7),           // Full funct7 for RVX10
    .ALUOp(ALUOp_D),
    .ALUControl(ALUControl_D)  // 5-bit output
  );

  //--- Control Pipeline Registers Instantiation ---

  // ID/EX Control Register
  // (Using the module `ctrl_ID_EX` you provided)
  ctrl_ID_EX c_reg_id_ex (
    .clk(clk), //
    .reset(reset), //
    .flush(Flush_E),           // Flush_E clears this register
    
    // Decode Inputs
    .RegWriteD(RegWrite_D), //
    .MemWriteD(MemWrite_D), //
    .JumpD(Jump_D), //
    .BranchD(Branch_D), //
    .ALUSrcD(ALUSrc_D), //
    .ResultSrcD(ResultSrc_D), //
    .ALUControlD(ALUControl_D), //
    
    // Execute Outputs
    .RegWriteE(RegWrite_E),    // To EX/MEM reg and Forwarding Unit
    .MemWriteE(MemWrite_E),    // To EX/MEM reg
    .JumpE(Jump_E),            // To PCSrc logic
    .BranchE(Branch_E),        // To PCSrc logic
    .ALUSrcE(ALUSrc_E), // To ALUSrc_E output
    .ResultSrcE(ResultSrc_E), // To ResultSrc_E output
    .ALUControlE(ALUControl_E)
  );
  assign ResultSrcE0 = ResultSrc_E[0];

  // EX/MEM Control Register
  // (Using the module `ctrl_EX_MEM` you provided)
  ctrl_EX_MEM c_reg_ex_mem (
    .clk(clk), //
    .reset(reset), //
    
    // Execute Inputs
    .RegWriteE(RegWrite_E),
    .MemWriteE(MemWrite_E),
    .ResultSrcE(ResultSrc_E),
    
    // Memory Outputs
    .RegWriteM(RegWrite_M),          // To MEM/WB reg and Forwarding Unit
    .MemWriteM(MemWrite_M), // To MemWrite_M output
    .ResultSrcM(ResultSrc_M)
  );

  // MEM/WB Control Register
  // (Using the module `ctrl_MEM_WB` you provided)
  ctrl_MEM_WB c_reg_mem_wb (
    .clk(clk),
    .reset(reset),
    
    // Memory Inputs
    .RegWriteM(RegWrite_M),
    .ResultSrcM(ResultSrc_M),
    
    // Writeback Outputs
    .RegWriteW(RegWrite_W),     // To RegWrite_W output
    .ResultSrcW(ResultSrc_W)
  );

  //--- Final Control Logic ---

  // PC Source Logic (Branch/Jump decision)
  // This logic is in the EX stage, using latched signals from ID/EX
  assign PCSrc_E = (Branch_E & Zero_E) | Jump_E;

endmodule

module datapath(
  input  logic       clk, reset, //

  // --- Control Signals (from controller) ---
  input  logic [1:0] ResultSrc_W, //
  input  logic       PCSrc_E, ALUSrc_E, //
  input  logic       RegWrite_W, //
  input  logic [1:0] ImmSrc_D, //
  input  logic [4:0] ALUControl_E, // 5-bits for RVX10

  // --- Hazard Signals (from hazard/fwd units) ---
  input  logic       stall_F, stall_D, // Stall PC reg, IF/ID reg
  input  logic       flush_D, flush_E, // Flush IF/ID reg, ID/EX reg
  input  logic [1:0] FwdSel_A, FwdSel_B, // Forwarding Mux selectors

  // --- Memory Interface (to top module) ---
  input  logic [31:0] Instr_F,       // Input: Instruction from imem
  input  logic [31:0] MemReadData_M, // Input: Data from dmem
  output logic [31:0] PC_F,            // Output: PC to imem
  output logic [31:0] MemAddr_M,     // Output: Address to dmem
  output logic [31:0] MemWriteData_M,// Output: Data to dmem

  // --- Feedback (to controller/hazard units) ---
  output logic       Zero_E, //
  output logic [31:0] Instr_D, //
  output logic [4:0] rs1_D, rs2_D, // Reg addrs from ID
  output logic [4:0] rs1_E, rs2_E, rd_E, rd_M, rd_W // Reg addrs from E, M, W

);

  //--- Internal Signal Wires ---

  // Fetch Stage
  logic [31:0] next_pc, pc_plus_4_F, pc_target_E;
  
  // Decode Stage
  logic [31:0] pc_D, pc_plus_4_D;
  logic [31:0] reg_data_1_D, reg_data_2_D, imm_ext_D;
  logic [4:0]  rd_D;

  // Execute Stage
  logic [31:0] pc_E, pc_plus_4_E;
  logic [31:0] reg_data_1_E, reg_data_2_E, imm_ext_E;
  logic [31:0] alu_in_A, alu_in_B, fwd_mux_b_out;
  logic [31:0] alu_result_E;

  // Memory Stage
  logic [31:0] pc_plus_4_M;

  // Writeback Stage
  logic [31:0] alu_result_W, mem_read_data_W, pc_plus_4_W;
  logic [31:0] R_W;



  // 
  // -----------------
  // --- Fetch Stage ---
  // -----------------
  // 
  
  // Mux for selecting next PC: PC+4 or branch/jump target
  mux2 #(.WIDTH(32)) pc_mux (
    .d0(pc_plus_4_F),
    .d1(pc_target_E), // From EX stage
    .s(PCSrc_E),      // From EX stage
    .y(next_pc)
  );

  // PC Register (stalls if stall_F is high)
  flopren #(.WIDTH(32)) pc_register (
    .clk(clk),
    .reset(reset),
    .en(~stall_F), // Enable only if not stalling
    .d(next_pc),
    .q(PC_F)
  );
  
  // Adder for PC + 4
  adder pc_add_4 (
    .a(PC_F),
    .b(32'd4),
    .y(pc_plus_4_F)
  );
  
  // 
  // --- IF/ID Pipeline Register ---
  // 
  
  pipe_IF_ID if_id_reg (
    .clk(clk),
    .reset(reset),
    .enable(~stall_D), // Stall (hold value) if stall_D is high
    .flush(flush_D),   // Clear (to NOP) if flush_D is high
    .InstrF(Instr_F),
    .PCF(PC_F),
    .PCPlus4F(pc_plus_4_F),
    .InstrD(Instr_D),
    .PCD(pc_D),
    .PCPlus4D(pc_plus_4_D)
    
  );

  // 
  // ------------------
  // --- Decode Stage ---
  // ------------------
  // 
  
  // Extract register addresses from instruction
  assign rs1_D = Instr_D[19:15];
  assign rs2_D = Instr_D[24:20];
  assign rd_D = Instr_D[11:7];
  
  // Register File
  // Reads are combinational (in ID stage)
  // Writes are synchronous (in WB stage)
  regfile reg_file (
    .clk(clk),
    .we3(RegWrite_W),     // Write enable from WB
    .a1(rs1_D),
    .a2(rs2_D),
    .a3(rd_W),           // Write address from WB
    .wd3(R_W), // Write data from WB
    .rd1(reg_data_1_D), //READ Data from output 1
    .rd2(reg_data_2_D)  //READ Data from output 2
  );
  
  // Sign/Immediate Extension Unit
  extend sign_extender (
    .instr(Instr_D[31:7]),
    .immsrc(ImmSrc_D),
    .immext(imm_ext_D)
  );
  
  // 
  // --- ID/EX Pipeline Register ---
  // 
  
  pipe_ID_EX id_ex_reg (
    .clk(clk), //
    .reset(reset), //
    .flush(flush_E), // Flushed by hazard unit
    .RD1D(reg_data_1_D), //
    .RD2D(reg_data_2_D), //
    .PCD(pc_D), //
    .ImmExtD(imm_ext_D), //
    .PCPlus4D(pc_plus_4_D), //
    .Rs1D(rs1_D), //
    .Rs2D(rs2_D), //
    .RdD(rd_D), //
    .RD1E(reg_data_1_E), //
    .RD2E(reg_data_2_E), //
    .PCE(pc_E), // 
    .ImmExtE(imm_ext_E), //
    .PCPlus4E(pc_plus_4_E), //
    .Rs1E(rs1_E), //
    .Rs2E(rs2_E), //
    .RdE(rd_E)
  );

  // 
  // -------------------
  // --- Execute Stage ---
  // -------------------
  // 
  
  // Forwarding Mux for ALU Operand A
  // 00: From regfile (ID/EX)
  // 01: From WB stage
  // 10: From MEM stage
  mux3 #(.WIDTH(32)) fwd_mux_A (
    .d0(reg_data_1_E),
    .d1(R_W),
    .d2(MemAddr_M),     // MemAddr_M is ALUResultM
    .s(FwdSel_A),
    .y(alu_in_A)
  );
  
  // Forwarding Mux for ALU Operand B
  mux3 #(.WIDTH(32)) fwd_mux_B (
    .d0(reg_data_2_E),
    .d1(R_W),
    .d2(MemAddr_M),     // MemAddr_M is ALUResultM
    .s(FwdSel_B),
    .y(fwd_mux_b_out)  // This result is also WriteData for SW
  );
  
  // Mux to select ALU Operand B (reg data or immediate)
  mux2 #(.WIDTH(32)) src_b_mux (
    .d0(fwd_mux_b_out), // From regfile/forwarding
    .d1(imm_ext_E),     // From immediate extender
    .s(ALUSrc_E),
    .y(alu_in_B)
  );
  
  // Adder for branch/jump target address
  adder pc_target_add (
    .a(pc_E),
    .b(imm_ext_E),
    .y(pc_target_E)
  );
  
  // The main Arithmetic Logic Unit (ALU)
  alu alu_unit (
    .a(alu_in_A),
    .b(alu_in_B),
    .alucontrol(ALUControl_E),
    .result(alu_result_E),
    .zero(Zero_E)
  );
  
  // 
  // --- EX/MEM Pipeline Register ---
  // 
  
  pipe_EX_MEM ex_mem_reg (
    .clk(clk), //
    .reset(reset), //
    .ALUResultE(alu_result_E), //
    .WriteDataE(fwd_mux_b_out), // Pass along reg data 2
    .PCPlus4E(pc_plus_4_E), //
    .RdE(rd_E),//
    .ALUResultM(MemAddr_M),     // ALU Result becomes Mem Addr
    .WriteDataM(MemWriteData_M), // Data to be written to dmem
    .PCPlus4M(pc_plus_4_M),
    .RdM(rd_M)
  );
  
  // 
  // -----------------
  // --- Memory Stage ---
  // -----------------
  // 
  // Datapath is passive in this stage.
  // MemAddr_M and MemWriteData_M are output to dmem.
  // MemReadData_M is input from dmem.
  
  // 
  // --- MEM/WB Pipeline Register ---
  // 
  
  pipe_MEM_WB mem_wb_reg (
    .clk(clk), ///
    .reset(reset), //
    .ReadDataM(MemReadData_M), // From dmem
    .ALUResultM(MemAddr_M),    // Pass ALU result through
    .PCPlus4M(pc_plus_4_M), //
    .RdM(rd_M), //
    .ReadDataW(mem_read_data_W), //
    .ALUResultW(alu_result_W), //
    .PCPlus4W(pc_plus_4_W),
    .RdW(rd_W)
  );

  // 
  // ----------------------
  // --- WriteBack Stage ---
  // ----------------------
  // 
  
  // Mux to select the final data to write to register file
  mux3 #(.WIDTH(32)) writeback_mux (
    .d0(alu_result_W),  // 00: From ALU
    .d1(mem_read_data_W),// 01: From Data Memory (LW)
    .d2(pc_plus_4_W),    // 10: From PC+4 (JAL)
    .s(ResultSrc_W),
    .y(R_W)
  );
endmodule

module forwarding_unit (
  // --- Inputs from Pipeline Registers ---
  input  logic [4:0] rs1_E,     // Source reg 1 addr in EX stage
  input  logic [4:0] rs2_E,     // Source reg 2 addr in EX stage
  input  logic [4:0] rd_M,      // Dest reg addr in MEM stage
  input  logic [4:0] rd_W,      // Dest reg addr in WB stage
  
  // --- Inputs from Controller ---
  input  logic       RegWrite_M,  // Write enable for MEM stage
  input  logic       RegWrite_W,  // Write enable for WB stage

  // --- Outputs to Datapath (EX Muxes) ---
  output logic [1:0] FwdSel_A,  // Forwarding select for ALU operand A
  output logic [1:0] FwdSel_B   // Forwarding select for ALU operand B
);

  /*
   * Forwarding Mux Selectors:
   * 2'b00: No forward (use value from ID/EX register)
   * 2'b01: Forward from WB stage (MEM/WB register)
   * 2'b10: Forward from MEM stage (EX/MEM register)
   */

  always_comb begin
    // --- Default: No forwarding ---
    FwdSel_A = 2'b00;
    FwdSel_B = 2'b00;

    // --- Case 1: Forward from WB to EX (Lower Priority) ---
    // Check if the instruction in WB is writing to a register (not x0)
    // that the instruction in EX needs.
    if (RegWrite_W && (rd_W != 5'b0)) begin
      if (rd_W == rs1_E)
        FwdSel_A = 2'b01; // Forward WB result to ALU input A
      if (rd_W == rs2_E)
        FwdSel_B = 2'b01; // Forward WB result to ALU input B
    end

    // --- Case 2: Forward from MEM to EX (Highest Priority) ---
    // Check if the instruction in MEM is writing to a register (not x0)
    // that the instruction in EX needs.
    // This logic comes *last* so it overwrites Case 1 if both are true.
    if (RegWrite_M && (rd_M != 5'b0)) begin
      if (rd_M == rs1_E)
        FwdSel_A = 2'b10; // Forward MEM result to ALU input A
      if (rd_M == rs2_E)
        FwdSel_B = 2'b10; // Forward MEM result to ALU input B
    end
  end

endmodule

module hazard_unit (
  // --- Inputs from Datapath & Controller ---
  input  logic [4:0] rs1_D,         // Source reg 1 addr in ID stage
  input  logic [4:0] rs2_D,         // Source reg 2 addr in ID stage
  input  logic [4:0] rd_E,          // Dest reg addr in EX stage
  input  logic       ResultSrc_E_0, // Control signal, 1 if EX instr is 'lw'
  input  logic       PCSrc_E,       // Control signal, 1 if branch taken or jump

  // --- Outputs to Datapath & Controller ---
  output logic       stall_F,       // Stall PC
  output logic       stall_D,       // Stall IF/ID Register
  output logic       flush_D,       // Flush IF/ID Register
  output logic       flush_E        // Flush ID/EX Register
);

  // --- Load-Use Hazard Detection ---
  // A load-use hazard occurs if the instruction in the EX stage is a load (`lw`)
  // and its destination register (`rd_E`) is one of the source registers
  // (`rs1_D` or `rs2_D`) for the instruction currently in the ID stage.
  
  logic load_use_hazard;
  
  assign load_use_hazard = ResultSrc_E_0 & (rd_E != 5'b0) & 
                           ((rd_E == rs1_D) | (rd_E == rs2_D));

  // --- Stall Signal Generation ---
  // If a load-use hazard is detected, we must stall the pipeline for one cycle.
  // We do this by freezing the PC and the IF/ID register.
  
  assign stall_F = load_use_hazard; // Stalls the PC register
  assign stall_D = load_use_hazard; // Stalls the IF/ID register (holds instruction)

  // --- Flush Signal Generation ---
  
  // 1. Flush for Control Hazard (Branch/Jump)
  //    If a branch is taken, the instructions in ID and IF are wrong.
  //    We must flush both the IF/ID and ID/EX registers.
  assign flush_D = PCSrc_E;       // Flushes the instruction in IF/ID register
  
  // 2. Flush for Load-Use Stall
  //    When we stall, we must also inject a NOP (bubble) into the
  //    EX stage. We do this by flushing the ID/EX register.
  //    This signal also handles the branch flush.
  assign flush_E = load_use_hazard | PCSrc_E; // Flushes the instruction in ID/EX register

endmodule

module maindec(input  logic [6:0] op,
               output logic [1:0] ResultSrc,
               output logic       MemWrite,
               output logic       Branch, ALUSrc,
               output logic       RegWrite, Jump,
               output logic [1:0] ImmSrc,
               output logic [1:0] ALUOp);

  logic [10:0] controls;

  assign {RegWrite, ImmSrc, ALUSrc, MemWrite,
          ResultSrc, Branch, ALUOp, Jump} = controls;

  always_comb
    case(op)
    // RegWrite_ImmSrc_ALUSrc_MemWrite_ResultSrc_Branch_ALUOp_Jump
      7'b0000011: controls = 11'b1_00_1_0_01_0_00_0; // lw
      7'b0100011: controls = 11'b0_01_1_1_00_0_00_0; // sw
      7'b0110011: controls = 11'b1_xx_0_0_00_0_10_0; // R-type 
      7'b1100011: controls = 11'b0_10_0_0_00_1_01_0; // beq
      7'b0010011: controls = 11'b1_00_1_0_00_0_10_0; // I-type ALU
      7'b1101111: controls = 11'b1_11_0_0_10_0_00_1; // jal
      // MODIFIED: Added case for RVX10 custom instructions
      7'b0001011: controls = 11'b1_xx_0_0_00_0_11_0; // RVX10 (R-Type, ALUOp=11)
      default:    controls = 11'bx_xx_x_x_xx_x_xx_x; // non-implemented instruction
    endcase
endmodule

module aludec(input  logic       opb5,
              input  logic [2:0] funct3,
              input  logic       funct7b5,
              input  logic [6:0] funct7, // MODIFIED: Added full funct7 for RVX10
              input  logic [1:0] ALUOp,
              output logic [4:0] ALUControl); // MODIFIED: Widened to 5 bits

  logic  RtypeSub;
  assign RtypeSub = funct7b5 & opb5;  // TRUE for R-type subtract instruction

  always_comb
    case(ALUOp)
      2'b00:                ALUControl = 5'b00000; // addition
      2'b01:                ALUControl = 5'b00001; // subtraction
      2'b10: case(funct3) // R-type or I-type ALU
                 3'b000:  if (RtypeSub) 
                            ALUControl = 5'b00001; // sub
                          else          
                            ALUControl = 5'b00000; // add, addi
                 3'b010:    ALUControl = 5'b00101; // slt, slti
                 3'b110:    ALUControl = 5'b00011; // or, ori
                 3'b111:    ALUControl = 5'b00010; // and, andi
                 default:   ALUControl = 5'bxxxxx; // ???
               endcase
      // MODIFIED: Added case for RVX10 instructions
      2'b11: case(funct7)
               7'b0000000: case(funct3)
                             3'b000: ALUControl = 5'b01000; // ANDN
                             3'b001: ALUControl = 5'b01001; // ORN
                             3'b010: ALUControl = 5'b01010; // XNOR
                             default: ALUControl = 5'bxxxxx;
                           endcase
               7'b0000001: case(funct3)
                             3'b000: ALUControl = 5'b01011; // MIN
                             3'b001: ALUControl = 5'b01100; // MAX
                             3'b010: ALUControl = 5'b01101; // MINU
                             3'b011: ALUControl = 5'b01110; // MAXU
                             default: ALUControl = 5'bxxxxx;
                           endcase
               7'b0000010: case(funct3)
                             3'b000: ALUControl = 5'b01111; // ROL
                             3'b001: ALUControl = 5'b10000; // ROR
                             default: ALUControl = 5'bxxxxx;
                           endcase
               7'b0000011: if (funct3 == 3'b000)
                             ALUControl = 5'b10001; // ABS
                           else
                             ALUControl = 5'bxxxxx;
               default: ALUControl = 5'bxxxxx;
             endcase
      default: ALUControl = 5'bxxxxx;
    endcase
endmodule

module ctrl_ID_EX (
  input  logic        clk, reset, flush,
  // --- Incoming control signals from ID ---
  input  logic        RegWriteD, MemWriteD, JumpD, BranchD, ALUSrcD,
  input  logic [1:0]  ResultSrcD,
  input  logic [4:0]  ALUControlD,     // 5 bits for RVX10 extended ALU ops
  // --- Outgoing latched signals to EX ---
  output logic        RegWriteE, MemWriteE, JumpE, BranchE, ALUSrcE,
  output logic [1:0]  ResultSrcE,
  output logic [4:0]  ALUControlE
);

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      RegWriteE   <= 1'b0;
      MemWriteE   <= 1'b0;
      JumpE       <= 1'b0;
      BranchE     <= 1'b0;
      ALUSrcE     <= 1'b0;
      ResultSrcE  <= 2'b00;
      ALUControlE <= 5'b00000;
    end
    else if (flush) begin
      // Flush on branch/jump or hazard: insert NOP control
      RegWriteE   <= 1'b0;
      MemWriteE   <= 1'b0;
      JumpE       <= 1'b0;
      BranchE     <= 1'b0;
      ALUSrcE     <= 1'b0;
      ResultSrcE  <= 2'b00;
      ALUControlE <= 5'b00000;
    end
    else begin
      RegWriteE   <= RegWriteD;
      MemWriteE   <= MemWriteD;
      JumpE       <= JumpD;
      BranchE     <= BranchD;
      ALUSrcE     <= ALUSrcD;
      ResultSrcE  <= ResultSrcD;
      ALUControlE <= ALUControlD;
    end
  end
  
endmodule

module ctrl_EX_MEM (
  input  logic        clk, reset,
  // --- Incoming control signals from EX ---
  input  logic        RegWriteE, MemWriteE,
  input  logic [1:0]  ResultSrcE,
  // --- Outgoing latched signals to MEM ---
  output logic        RegWriteM, MemWriteM,
  output logic [1:0]  ResultSrcM
);

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      RegWriteM  <= 1'b0;
      MemWriteM  <= 1'b0;
      ResultSrcM <= 2'b00;
    end
    else begin
      RegWriteM  <= RegWriteE;
      MemWriteM  <= MemWriteE;
      ResultSrcM <= ResultSrcE;
    end
  end

endmodule

module ctrl_MEM_WB (
  input  logic        clk, reset,
  // --- Incoming control signals from MEM ---
  input  logic        RegWriteM,
  input  logic [1:0]  ResultSrcM,
  // --- Outgoing latched signals to WB ---
  output logic        RegWriteW,
  output logic [1:0]  ResultSrcW
);

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      RegWriteW  <= 1'b0;
      ResultSrcW <= 2'b00;
    end
    else begin
      RegWriteW  <= RegWriteM;
      ResultSrcW <= ResultSrcM;
    end
  end

endmodule

module mux2 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, 
              input  logic             s, 
              output logic [WIDTH-1:0] y);

  assign y = s ? d1 : d0; 
endmodule

module flopren #(
  parameter WIDTH = 8
) (
  input  logic             clk,   // Clock
  input  logic             reset, // Asynchronous reset
  input  logic             en,    // Synchronous enable
  input  logic [WIDTH-1:0] d,     // Data input
  output logic [WIDTH-1:0] q      // Data output
);

  // Sequential logic with an asynchronous reset.
  always_ff @(posedge clk or posedge reset) begin
    // Asynchronous reset has the highest priority.
    if (reset)
      q <= 0;
    // On a clock edge, load 'd' into 'q' only if enabled.
    else if (en)
      q <= d;
    // If 'en' is low, 'q' holds its previous value.
    // else if (!en) // This is redundant
    //   q <= q;
  end
endmodule
module adder(
  input  [31:0] a, b, // 32-bit inputs
  output [31:0] y     // 32-bit output (a + b)
);

  assign y = a + b;
endmodule


module regfile(input  logic        clk, 
               input  logic        we3, 
               input  logic [ 4:0] a1, a2, a3, 
               input  logic [31:0] wd3, 
               output logic [31:0] rd1, rd2);

  logic [31:0] rf[31:0];

  // three ported register file
  // read two ports combinationally (A1/RD1, A2/RD2)
  // write third port on rising edge of clock (A3/WD3/WE3)
  // register 0 hardwired to 0

  always_ff @(posedge clk)
    if (we3) rf[a3] <= wd3;	

  assign rd1 = (a1 != 0) ? rf[a1] : 0;
  assign rd2 = (a2 != 0) ? rf[a2] : 0;
endmodule

module extend(
  // --- Input ---
  input  logic [31:7] instr,   // Relevant bits of the instruction
  input  logic [1:0]  immsrc,  // Selects immediate type
  
  // --- Output ---
  output logic [31:0] immext   // 32-bit sign-extended immediate
);
 
  // Combinational logic to generate the correct immediate
  always_comb
    case(immsrc) 
      // I-type (lw, addi, slti, etc.)
      2'b00:  immext = {{20{instr[31]}}, instr[31:20]}; 
            
      // S-type (stores)
      2'b01:  immext = {{20{instr[31]}}, instr[31:25], instr[11:7]}; 
            
      // B-type (branches)
      2'b10:  immext = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0}; 
            
      // J-type (jal)
      2'b11:  immext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0}; 
            
      default: immext = 32'bx; // undefined
    endcase       
endmodule

module mux3 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, d2,
              input  logic [1:0]       s, 
              output logic [WIDTH-1:0] y);

  assign y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule

module pipe_IF_ID (
  input  logic        clk, reset,
  input  logic        enable, flush,     // enable=stall control, flush=branch/jump
  input  logic [31:0] InstrF, PCF, PCPlus4F,
  output logic [31:0] InstrD, PCD, PCPlus4D
);

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      InstrD   <= 32'b0;
      PCD      <= 32'b0;
      PCPlus4D <= 32'b0;
    end 
    else if (enable) begin
      if (flush) begin
        // Insert a NOP (add x0, x0, x0) when flushed
        InstrD   <= 32'h00000033;
        PCD      <= 32'b0;
        PCPlus4D <= 32'b0;
      end 
      else begin
        InstrD   <= InstrF;
        PCD      <= PCF;
        PCPlus4D <= PCPlus4F;
      end
    end
    // If enable==0, pipeline is stalled: hold previous values
  end

endmodule

module pipe_ID_EX (
  input  logic        clk, reset, flush,
  input  logic [31:0] RD1D, RD2D, PCD, ImmExtD, PCPlus4D,
  input  logic [4:0]  Rs1D, Rs2D, RdD,
  output logic [31:0] RD1E, RD2E, PCE, ImmExtE, PCPlus4E,
  output logic [4:0]  Rs1E, Rs2E, RdE
);

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      RD1E     <= 32'b0;
      RD2E     <= 32'b0;
      PCE      <= 32'b0;
      ImmExtE  <= 32'b0;
      PCPlus4E <= 32'b0;
      Rs1E     <= 5'b0;
      Rs2E     <= 5'b0;
      RdE      <= 5'b0;
    end 
    else if (flush) begin
      // Flush on branch/jump or hazard (insert NOP)
      RD1E     <= 32'b0;
      RD2E     <= 32'b0;
      PCE      <= 32'b0;
      ImmExtE  <= 32'b0;
      PCPlus4E <= 32'b0;
      Rs1E     <= 5'b0;
      Rs2E     <= 5'b0;
      RdE      <= 5'b0;
    end 
    else begin
      RD1E     <= RD1D;
      RD2E     <= RD2D;
      PCE      <= PCD;
      ImmExtE  <= ImmExtD;
      PCPlus4E <= PCPlus4D;
      Rs1E     <= Rs1D;
      Rs2E     <= Rs2D;
      RdE      <= RdD;
    end
  end

endmodule

module imem(input  logic [31:0] a,
            output logic [31:0] rd);

  logic [31:0] RAM[63:0];

  initial begin
      $readmemh("riscvtest.mem",RAM);
      end

  assign rd = RAM[a[31:2]]; // word aligned
endmodule

module pipe_EX_MEM (
  input  logic        clk, reset,
  input  logic [31:0] ALUResultE, WriteDataE, PCPlus4E,
  input  logic [4:0]  RdE,
  output logic [31:0] ALUResultM, WriteDataM, PCPlus4M,
  output logic [4:0]  RdM
);

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      ALUResultM <= 32'b0;
      WriteDataM <= 32'b0;
      PCPlus4M   <= 32'b0;
      RdM        <= 5'b0;
    end 
    else begin
      ALUResultM <= ALUResultE;
      WriteDataM <= WriteDataE;
      PCPlus4M   <= PCPlus4E;
      RdM        <= RdE;
    end
  end

endmodule

module pipe_MEM_WB (
  input  logic        clk, reset,
  input  logic [31:0] ReadDataM, ALUResultM, PCPlus4M,
  input  logic [4:0]  RdM,
  output logic [31:0] ReadDataW, ALUResultW, PCPlus4W,
  output logic [4:0]  RdW
);

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      ReadDataW  <= 32'b0;
      ALUResultW <= 32'b0;
      PCPlus4W   <= 32'b0;
      RdW        <= 5'b0;
    end 
    else begin
      ReadDataW  <= ReadDataM;
      ALUResultW <= ALUResultM;
      PCPlus4W   <= PCPlus4M;
      RdW        <= RdM;
    end
  end

endmodule

module dmem(input  logic        clk, we,
            input  logic [31:0] a, wd,
            output logic [31:0] rd);

  logic [31:0] RAM[63:0];

  assign rd = RAM[a[31:2]]; // word aligned

  always_ff @(posedge clk)
    if (we) RAM[a[31:2]] <= wd;
endmodule

module alu(input  logic [31:0] a, b,
           input  logic [4:0]  alucontrol, // MODIFIED: Widened to 5 bits
           output logic [31:0] result,
           output logic        zero);

  logic [31:0] condinvb, sum;
  logic        v;              // overflow
  logic        isAddSub;       // true when is add or subtract operation
  logic signed [31:0] signed_a, signed_b;
  logic [4:0] shamt;

  assign condinvb = alucontrol[0] ? ~b : b;
  assign sum = a + condinvb + alucontrol[0];
  assign isAddSub = (alucontrol == 5'b00000) || (alucontrol == 5'b00001);

  assign signed_a = a;
  assign signed_b = b;
  assign shamt = b[4:0];
  
  always_comb
    case (alucontrol)
      // Base ISA operations
      5'b00000: result = sum;         // add
      5'b00001: result = sum;         // subtract
      5'b00010: result = a & b;       // and
      5'b00011: result = a | b;       // or
      5'b00101: result = sum[31] ^ v; // slt
      5'b00100: result = a ^ b;       // xor
      5'b00110: result = a << b[4:0]; // sll
      5'b00111: result = a >> b[4:0]; // srl
      // RVX10 custom operations
      5'b01000: result = a & ~b;                                                     // ANDN
      5'b01001: result = a | ~b;                                                     // ORN
      5'b01010: result = ~(a ^ b);                                                   // XNOR
      5'b01011: result = (signed_a < signed_b) ? a : b;                              // MIN
      5'b01100: result = (signed_a > signed_b) ? a : b;                              // MAX
      5'b01101: result = (a < b) ? a : b;                                            // MINU
      5'b01110: result = (a > b) ? a : b;                                            // MAXU
      5'b01111: result = (shamt == 5'b0) ? a : (a << shamt) | (a >> (32 - shamt));    // ROL
      5'b10000: result = (shamt == 5'b0) ? a : (a >> shamt) | (a << (32 - shamt));    // ROR
      5'b10001: result = (signed_a >= 32'sd0) ? a : -signed_a;                       // ABS
      default: result = 32'bx;
    endcase

  assign zero = (result == 32'b0);
  assign v = ~(alucontrol[0] ^ a[31] ^ b[31]) & (a[31] ^ sum[31]) & isAddSub;
  
endmodule
