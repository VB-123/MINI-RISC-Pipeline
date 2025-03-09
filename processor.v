`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 15.02.2025 19:05:46
// Design Name:
// Module Name: ALU_with_reg_tb
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


module processor;

  `include "parameters.v"
  // register that gets upper mul bits
  // and the remainder of div
  localparam ALU_EXTRA_OUTPUT_REG = 3'b001; 
  // special regs
  reg clk;
  reg rst;
  wire [15:0] flag;
  reg [15:0] instruction_mem[0:MEMORY_DEPTH-1];  // 11 bits to index
  reg [10:0] branch_reg;

  // alias
  wire [15:0] instruction_reg;
  wire [4:0] opcode;

  wire [7:0] immediate_byte;
  wire [2:0] reg_sel_write_0;
  wire [2:0] reg_sel_read_0;
  wire [2:0] reg_sel_read_1;
  wire [3:0] bit_pos;

  // connections
  wire [15:0] reg_file_out_0;
  wire [15:0] reg_file_out_1;
  wire [15:0] ram_out;
  wire [10:0] instruction_addr;
  wire [15:0] alu_result_0;
  wire [15:0] alu_result_1;

  //MUXed
  reg [15:0] reg_file_data_in_0;
  reg [0:1] reg_file_write_en_0;

  // assignemtents
  assign instruction_reg = instruction_mem[instruction_addr];
  assign opcode = instruction_reg[15:11];

  assign immediate_byte = instruction_reg[7:0];
  assign reg_sel_write_0 = instruction_reg[10:8];
  assign reg_sel_read_0 = instruction_reg[7:5];
  assign reg_sel_read_1 = instruction_reg[4:2];
  assign bit_pos = instruction_reg[4:1];








  // control signals
   wire alu_output_control;
   wire bit_control;

  control_unit cu (
    .clk(clk),
    .opcode(opcode),
    .alu_output(alu_output_control)
 );


  program_counter pc (
     .clk(clk),
     .rst(rst),
     .inc(1'b1),
     .branch_en(1'b0),  // TODO
     .branch_addr(branch_reg),
     .current_addr(instruction_addr)
  );

  reg_file reg_file (
      .clk(clk),
      .rst(rst),
      .write_en_0(reg_file_write_en_0),
      .write_en_1((opcode == MUL) || (opcode == DIV) ? 2'b11 : 2'b00),
      .read_addr_0(reg_sel_read_0),
      .read_addr_1(reg_sel_read_1),
      .reg_write_addr_0(reg_sel_write_0),
      .reg_write_addr_1(ALU_EXTRA_OUTPUT_REG),
      .data_in_0(reg_file_data_in_0),
      .data_in_1(alu_result_1), 
      .read_data_0(reg_file_out_0),
      .read_data_1(reg_file_out_1)
  );


   memory ram (
     .clk(clk),
     .write_en(opcode == STORE),
     .address(reg_file_out_0),
     .data_in(reg_file_out_1),
     .read_data(ram_out) 
   );


   ALU alu (
     .clk(clk),
     .opcode(opcode),  
     .operand_1(reg_file_out_0),  
     .operand_2(reg_file_out_1), 
     .bit_position(bit_pos),

     .result_0(alu_result_0),  
     .result_1(alu_result_1), 
     .flag_reg(flag)
   );


  always @(posedge clk) begin
   reg_file_write_en_0 = 2'b00;
   reg_file_data_in_0 = 16'bx;

    if (opcode == HALT) $finish; // TODO: also set inc to 0

    if (opcode == LOAD ) begin
      reg_file_write_en_0 = 2'b11;
      reg_file_data_in_0 = ram_out;
    end

    if (opcode == LBL) begin
     reg_file_write_en_0 = 2'b01;
     reg_file_data_in_0 = {8'b0, immediate_byte};
    end
    if (opcode == LBH) begin
     reg_file_write_en_0 = 2'b10;
     reg_file_data_in_0 = {immediate_byte, 8'b0};
    end
    if (opcode == MOV) begin 
      reg_file_write_en_0 = 2'b11;
      reg_file_data_in_0 = reg_file_out_0;
    end

    if (alu_output_control) begin
      reg_file_write_en_0 = 2'b11;
      reg_file_data_in_0 = alu_result_0;
    end

  if (reg_file_write_en_0 != 2'b00 && reg_file_data_in_0 == 16'bx) $fatal;

  end

  initial begin
    clk = 0;
    forever #5 clk = ~clk;
  end

  // Test block
  initial begin
    alu_test_inc;
    reset;
  end

  task reset;
    begin
    rst = 0;
    @(posedge clk);
    rst = 1;
    @(posedge clk);
    rst = 0;
    @(posedge clk);
    end
  endtask

  task load_test_1;
    begin
      $display("load test 1");
      instruction_mem[0] = {LBL, REG0, 8'hFF};
      instruction_mem[1] = {LBL, REG1, 8'hFF};
      instruction_mem[2] = {LBL, REG2, 8'hFF};
      instruction_mem[3] = {LBH, REG0, 8'hFF};
      instruction_mem[4] = {LBH, REG1, 8'hFF};
      instruction_mem[5] = {LBH, REG2, 8'hFF};
      instruction_mem[6] = {HALT, 11'bx};
    end
  endtask

  task load_test_2;
    begin
      $display("load test 2");
      instruction_mem[0] = {LBL, REG0, 8'h01};
      instruction_mem[1] = {LBL, REG1, 8'h02};
      instruction_mem[2] = {LBL, REG2, 8'hF3};
      instruction_mem[3] = {LBH, REG0, 8'hF4};
      instruction_mem[4] = {LBH, REG1, 8'hF5};
      instruction_mem[5] = {LBH, REG2, 8'hF6};
      instruction_mem[6] = {HALT, 11'bx};
    end
  endtask

  task mov_test_1;
    begin
      $display("mov test 1");
      instruction_mem[0] = {LBL, REG0, 8'h01};
      instruction_mem[1] = {LBL, REG1, 8'h02};
      instruction_mem[2] = {LBL, REG2, 8'hF3};
      instruction_mem[3] = {LBH, REG0, 8'hF1};
      instruction_mem[4] = {LBH, REG1, 8'hF2};
      instruction_mem[5] = {LBH, REG2, 8'hF3};
      instruction_mem[6] = {MOV, REG7, REG0, 5'bx};
      instruction_mem[7] = {MOV, REG6, REG1, 5'bx};
      instruction_mem[8] = {MOV, REG5, REG2, 5'bx};
      instruction_mem[9] = {HALT, 11'bx};
    end
  endtask

  task load_store_test;
  begin
  $display("load_store_test");
    instruction_mem[0] = {LBH, REG0, 8'd1};
    instruction_mem[1] = {LBL, REG0, 8'd1};
    instruction_mem[2] = {LBH, REG1, 8'd255};
    instruction_mem[3] = {LBL, REG1, 8'd255};
    instruction_mem[4] = {LBH, REG2, 8'd2};
    instruction_mem[5] = {LBL, REG2, 8'd2};
    instruction_mem[6] = {STORE, 3'bx, REG2, REG1, 2'bx};
    instruction_mem[7] = {LOAD, REG3, REG2, 5'bx};
    instruction_mem[8] = {HALT, 11'bx};
  end
  endtask

  task alu_test_inc;
  begin
  $display("alu_test_inc");
    instruction_mem[0] = {LBH, REG0, 8'd255};      
    instruction_mem[1] = {LBL, REG0, 8'd16};      
    instruction_mem[2] = {INC, REG1, REG0, 5'bx};
    instruction_mem[3] = {INC, REG2, REG1, 5'bx};
    instruction_mem[4] = {INC, REG3, REG2, 5'bx};
    instruction_mem[5] = {INC, REG4, REG3, 5'bx};
    instruction_mem[6] = {INC, REG5, REG4, 5'bx};
    instruction_mem[7] = {INC, REG6, REG5, 5'bx};
    instruction_mem[8] = {INC, REG7, REG6, 5'bx};
    instruction_mem[9] = {HALT, 11'bx};
  end
  endtask

endmodule
