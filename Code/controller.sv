`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.10.2025 19:30:00
// Design Name: 
// Module Name: controller
// Project Name: RVX10-P
// Target Devices: 
// Tool Versions: 
// Description: Pipelined controller for the RVX10-P processor.
//              This module contains the decoders (maindec, aludec)
//              and the pipeline registers for the control signals.
// 
// Dependencies: maindec.sv, aludec.sv, ctrl_ID_EX.sv, ctrl_EX_MEM.sv, ctrl_MEM_WB.sv
// 
//////////////////////////////////////////////////////////////////////////////////

module controller (
  input  logic       clk, reset,
  
  // --- Inputs from ID Stage (Instruction) ---
  input  logic [6:0] op,
  input  logic [2:0] funct3,
  input  logic [6:0] funct7,       // Full funct7 for RVX10
  
  // --- Inputs from Datapath / Hazard Unit ---
  input  logic       Zero_E,       // Zero flag from ALU in EX stage
  input  logic       Flush_E,      // Flush signal from Hazard Unit
  
  // --- Control Outputs to ID Stage ---
  output logic [1:0] ImmSrc_D,
  
  // --- Control Outputs to EX Stage ---
  output logic       PCSrc_E,
  output logic       ALUSrc_E,
  output logic [4:0] ALUControl_E, // 5-bits for RVX10
  
  // --- Control Outputs to MEM Stage ---
  output logic       MemWrite_M,
  
  // --- Control Outputs to WB Stage ---
  output logic       RegWrite_W,
  output logic [1:0] ResultSrc_W,
  
  // --- Outputs to Hazard/Forwarding Units ---
  // (Signals are latched from internal registers)
  output logic ResultSrcE0,
  output logic       RegWrite_M // To Forwarding Unit
);
  
  //--- Internal Wires for Decode (ID) Stage ---
  logic [1:0] ALUOp_D;
  logic [1:0] ResultSrc_D;
  logic [4:0] ALUControl_D;
  logic       Branch_D, MemWrite_D, Jump_D;
  logic       ALUSrc_D, RegWrite_D;

  //--- Internal Wires for Execute (EX) Stage ---
  logic [1:0] ResultSrc_E; // Renamed to avoid port conflict
  logic [4:0] ALUControl_E_internal;
  logic       Branch_E, MemWrite_E, Jump_E;
  logic       ALUSrc_E_internal;

  //--- Internal Wires for Memory (MEM) Stage ---
  logic [1:0] ResultSrc_M;
  logic       MemWrite_M_internal;
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
    .funct7b5(funct7[5]),      // Bit 5 of funct7 for SUB
    .funct7(funct7),           // Full funct7 for RVX10
    .ALUOp(ALUOp_D),
    .ALUControl(ALUControl_D)  // 5-bit output
  );

  //--- Control Pipeline Registers Instantiation ---

  // ID/EX Control Register
  // (Using the module `ctrl_ID_EX` you provided)
  ctrl_ID_EX c_reg_id_ex (
    .clk(clk),
    .reset(reset),
    .flush(Flush_E),           // Flush_E clears this register
    
    // Decode Inputs
    .RegWriteD(RegWrite_D),
    .MemWriteD(MemWrite_D),
    .JumpD(Jump_D),
    .BranchD(Branch_D),
    .ALUSrcD(ALUSrc_D),
    .ResultSrcD(ResultSrc_D),
    .ALUControlD(ALUControl_D),
    
    // Execute Outputs
    .RegWriteE(RegWrite_E),    // To EX/MEM reg and Forwarding Unit
    .MemWriteE(MemWrite_E),    // To EX/MEM reg
    .JumpE(Jump_E),            // To PCSrc logic
    .BranchE(Branch_E),        // To PCSrc logic
    .ALUSrcE(ALUSrc_E_internal), // To ALUSrc_E output
    .ResultSrcE(ResultSrc_E_internal), // To ResultSrc_E output
    .ALUControlE(ALUControl_E_internal), // To ALUControl_E output

    .valid_in(1'b1),           // Assuming valid unless stalled
    .valid_out()               // Not used
  );
  assign ResultSrcE0 = ResultSrc_E[0];

  // EX/MEM Control Register
  // (Using the module `ctrl_EX_MEM` you provided)
  ctrl_EX_MEM c_reg_ex_mem (
    .clk(clk),
    .reset(reset),
    
    // Execute Inputs
    .RegWriteE(RegWrite_E),
    .MemWriteE(MemWrite_E),
    .ResultSrcE(ResultSrc_E_internal),
    
    // Memory Outputs
    .RegWriteM(RegWrite_M),          // To MEM/WB reg and Forwarding Unit
    .MemWriteM(MemWrite_M_internal), // To MemWrite_M output
    .ResultSrcM(ResultSrc_M),
    
    .valid_in(1'b1),
    .valid_out()
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
    .ResultSrcW(ResultSrc_W),   // To ResultSrc_W output

    .valid_in(1'b1),
    .valid_out()
  );

  //--- Final Control Logic ---

  // PC Source Logic (Branch/Jump decision)
  // This logic is in the EX stage, using latched signals from ID/EX
  assign PCSrc_E = (Branch_E & Zero_E) | Jump_E;

  // Assign internal latched signals to module outputs
  assign ALUSrc_E = ALUSrc_E_internal;
  assign ALUControl_E = ALUControl_E_internal;
  assign ResultSrc_E = ResultSrc_E_internal;
  assign MemWrite_M = MemWrite_M_internal;
  // RegWrite_M, RegWrite_W, and ResultSrc_W are already connected

endmodule
