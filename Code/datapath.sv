`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.10.2025 20:32:46
// Design Name: 
// Module Name: datapath
// Project Name: RVX10-P
// Target Devices: 
// Tool Versions: 
// Description: 5-Stage Pipelined Datapath for RVX10-P.
//              Based on user's single-cycle components and pipeline registers.
// 
// Dependencies: pipe_IF_ID, pipe_ID_EX, pipe_EX_MEM, pipe_MEM_WB,
//               regfile, alu, extend, adder, mux2, mux3, flopren
// 
//////////////////////////////////////////////////////////////////////////////////

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
    .RdW(rd_W),
    
    .valid_in(valid_M_internal),
    .valid_out(valid_W_internal)
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

