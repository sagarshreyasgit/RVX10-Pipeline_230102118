`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.10.2025 21:23:21
// Design Name: 
// Module Name: hazard_unit
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
