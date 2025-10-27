`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.10.2025 18:08:26
// Design Name: 
// Module Name: pipe_IF_ID
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
// Pipeline Register: IF/ID
// Purpose : Holds fetched instruction and PC for Decode stage.
// Features:
//   - Asynchronous reset
//   - Synchronous flush (insert NOP)
//   - Enable signal for stalling
//   - Valid bit propagation
//======================================================================
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
