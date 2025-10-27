`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.10.2025 18:08:26
// Design Name: 
// Module Name: pipe_ID_EX
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


//======================================================================
// Pipeline Register: ID/EX
// Purpose : Passes decoded operands, immediate, PC, and register indices
//           to Execute stage.
// Features:
//   - Asynchronous reset
//   - Synchronous flush (clear)
//   - Valid bit propagation
//======================================================================
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

