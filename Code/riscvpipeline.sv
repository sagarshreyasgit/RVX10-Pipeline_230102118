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
  logic [1:0] w_ResultSrc_W;
  logic       w_PCSrc_E;
  logic       w_ALUSrc_E;
  logic       w_RegWrite_W, w_RegWrite_M;
  logic [1:0] w_ImmSrc_D;
  logic [4:0] w_ALUControl_E; // 5-bits for RVX10
  logic       w_Zero_E;
  logic       w_ResultSrc_E_0;
  logic       w_Jump_E, w_Branch_E;

  // --- Internal Wires: Hazard Unit ---
  logic       w_stall_F, w_stall_D;
  logic       w_flush_D, w_flush_E;
  logic [1:0] w_FwdSel_A, w_FwdSel_B;

  // --- Internal Wires: Datapath Feedback ---
  logic [31:0] w_Instr_D;
  logic [4:0]  w_rs1_D, w_rs2_D;
  logic [4:0]  w_rs1_E, w_rs2_E, w_rd_E, w_rd_M, w_rd_W;
  
  // --- Internal Wires: Decoded Instruction Fields ---
  // These wires slice the instruction from the datapath to feed the controller
  logic [6:0] w_op_D;
  logic [2:0] w_funct3_D;
  logic [6:0] w_funct7_D;
  logic       w_funct7b5_D;
  
  assign w_op_D     = w_Instr_D[6:0];
  assign w_funct3_D = w_Instr_D[14:12];
  assign w_funct7_D = w_Instr_D[31:25];
  assign w_funct7b5_D = w_Instr_D[30];
  
  // --- (Optional) Valid Bit Wires ---
  logic w_valid_D, w_valid_E, w_valid_M, w_valid_W;


  // 
  // --- 1. Datapath Instantiation ---
  // 
  datapath processor_datapath (
    .clk(clk),
    .reset(reset),
    .ResultSrc_W(w_ResultSrc_W),
    .PCSrc_E(w_PCSrc_E),
    .ALUSrc_E(w_ALUSrc_E),
    .RegWrite_W(w_RegWrite_W),
    .ImmSrc_D(w_ImmSrc_D),
    .ALUControl_E(w_ALUControl_E),
    .stall_F(w_stall_F),
    .stall_D(w_stall_D),
    .flush_D(w_flush_D),
    .flush_E(w_flush_E),
    .FwdSel_A(w_FwdSel_A),
    .FwdSel_B(w_FwdSel_B),
    .Instr_F(Instr_F),
    .MemReadData_M(MemReadData_M),
    .Zero_E(w_Zero_E),
    .Instr_D(w_Instr_D),
    .rs1_D(w_rs1_D),
    .rs2_D(w_rs2_D),
    .rs1_E(w_rs1_E),
    .rs2_E(w_rs2_E),
    .rd_E(w_rd_E),
    .rd_M(w_rd_M),
    .rd_W(w_rd_W),
    .PC_F(PC_F),
    .MemAddr_M(MemAddr_M),
    .MemWriteData_M(MemWriteData_M),
    
    // Connect valid bit outputs
    .valid_D(w_valid_D),
    .valid_E(w_valid_E),
    .valid_M(w_valid_M),
    .valid_W(w_valid_W)
  );

  // 
  // --- 2. Controller Instantiation ---
  // 
  controller processor_controller (
    .clk(clk),
    .reset(reset),
    .op(w_op_D),
    .funct3(w_funct3_D),
    .funct7(w_funct7_D),
    .Zero_E(w_Zero_E),
    .Flush_E(w_flush_E), // Fix capitalization
    .ImmSrc_D(w_ImmSrc_D),
    .PCSrc_E(w_PCSrc_E),
    .ALUSrc_E(w_ALUSrc_E),
    .ALUControl_E(w_ALUControl_E),
    .MemWrite_M(MemWrite_M),
    .RegWrite_W(w_RegWrite_W),
    .ResultSrc_W(w_ResultSrc_W),
    .RegWrite_M(w_RegWrite_M),
    .ResultSrcE0(w_ResultSrc_E_0) // This is for hazard detection
  );

  // 
  // --- 3. Forwarding Unit Instantiation ---
  // 
  forwarding_unit fwd_unit (
    .rs1_E(w_rs1_E),
    .rs2_E(w_rs2_E),
    .rd_M(w_rd_M),
    .rd_W(w_rd_W),
    .RegWrite_M(w_RegWrite_M),
    .RegWrite_W(w_RegWrite_W),
    .FwdSel_A(w_FwdSel_A),
    .FwdSel_B(w_FwdSel_B)
  );

  // 
  // --- 4. Hazard Detection Unit Instantiation ---
  // 
  hazard_unit hazard_detect (
    .rs1_D(w_rs1_D),
    .rs2_D(w_rs2_D),
    .rd_E(w_rd_E),
    .ResultSrc_E_0(w_ResultSrc_E_0), // 1 if EX is 'lw'
    .PCSrc_E(w_PCSrc_E),         // 1 if branch taken or jump
    .stall_F(w_stall_F),
    .stall_D(w_stall_D),
    .flush_D(w_flush_D),
    .flush_E(w_flush_E)
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
      if (w_valid_W) begin
        retired_instructions <= retired_instructions + 1;
      end
    end
  end

endmodule


