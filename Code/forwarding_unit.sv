`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.10.2025 21:24:30
// Design Name: 
// Module Name: forwarding_unit
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
