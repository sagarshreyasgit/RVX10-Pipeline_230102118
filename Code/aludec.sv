`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.10.2025 17:54:09
// Design Name: 
// Module Name: aludec
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
