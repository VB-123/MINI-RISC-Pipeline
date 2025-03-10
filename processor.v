`include "parameters.v"
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

module pipelined_processor;
  
  // Special registers
  reg clk;
  reg rst;
  
  // Memory arrays
  reg [15:0] instruction_mem[0:`MEMORY_DEPTH-1];  // Instruction memory
  
  // =====================
  // FETCH STAGE SIGNALS
  // =====================
  wire [10:0] pc_F;                // Current program counter
  wire [15:0] instruction_F;       // Instruction fetched from memory
  wire stall_F;                    // Stall signal for fetch stage
  wire flush_F;                    // Flush signal for fetch stage
  
  // =====================
  // DECODE STAGE SIGNALS
  // =====================
  wire [15:0] instruction_D;       // Instruction in decode stage
  wire [10:0] pc_D;                // PC in decode stage
  wire [4:0] opcode_D;             // Opcode in decode stage
  wire [2:0] rs1_D;                // Source register 1 address
  wire [2:0] rs2_D;                // Source register 2 address
  wire [2:0] rd_D;                 // Destination register address
  wire [7:0] immediate_D;          // Immediate value
  wire [3:0] bit_pos_D;            // Bit position
  wire [15:0] reg_data_1_D;        // Register data 1
  wire [15:0] reg_data_2_D;        // Register data 2
  wire stall_D;                    // Stall signal for decode stage
  wire flush_D;                    // Flush signal for decode stage
  
  // Control signals from decode
  wire alu_src_D;
  wire reg_write_D;
  wire mem_read_D;
  wire mem_write_D;
  wire mem_to_reg_D;
  wire branch_D;
  wire jump_D;
  wire [1:0] write_mode_D;
  wire alu_op_D;
  
  // =====================
  // EXECUTE STAGE SIGNALS
  // =====================
  wire [4:0] opcode_E;             // Opcode in execute stage
  wire [2:0] rd_E;                 // Destination register in execute
  wire [15:0] reg_data_1_E;        // Register data 1
  wire [15:0] reg_data_2_E;        // Register data 2
  wire [7:0] immediate_E;          // Immediate value
  wire [3:0] bit_pos_E;            // Bit position
  wire [10:0] pc_E;                // PC in execute stage
  wire [15:0] alu_result_0_E;      // ALU result 0
  wire [15:0] alu_result_1_E;      // ALU result 1 (for MUL/DIV)
  wire [15:0] mem_read_data_E;     // Memory read data
  wire [15:0] next_flags_E;        // Next flags from ALU
  wire [15:0] current_flags_E;     // Current flags
  
  // Control signals in execute
  wire alu_src_E;
  wire reg_write_E;
  wire mem_read_E;
  wire mem_write_E;
  wire mem_to_reg_E;
  wire [1:0] write_mode_E;
  wire alu_op_E;
  
  // Forwarding signals
  wire [1:0] forward_A;
  wire [1:0] forward_B;
  wire [15:0] alu_operand_1;
  wire [15:0] alu_operand_2;
  
  // =====================
  // WRITEBACK STAGE SIGNALS
  // =====================
  wire [4:0] opcode_W;             // Opcode in writeback stage
  wire [2:0] rd_W;                 // Destination register in writeback
  wire [15:0] alu_result_0_W;      // ALU result 0
  wire [15:0] alu_result_1_W;      // ALU result 1 (for MUL/DIV)
  wire [15:0] mem_data_W;          // Memory data
  wire [15:0] reg_write_data_W;    // Data to write to register
  
  // Control signals in writeback
  wire reg_write_W;
  wire mem_to_reg_W;
  wire [1:0] write_mode_W;
  
  // =====================
  // ASSIGNMENTS
  // =====================
  // Fetch stage
  assign instruction_F = instruction_mem[pc_F];
  
  // Decode stage 
  assign opcode_D = instruction_D[15:11];
  assign rd_D = instruction_D[10:8];       // Destination register
  assign rs1_D = instruction_D[7:5];       // Source register 1
  assign rs2_D = instruction_D[4:2];       // Source register 2
  assign immediate_D = instruction_D[7:0]; // Immediate value
  assign bit_pos_D = instruction_D[4:1];   // Bit position
  
  // Writeback stage
  assign reg_write_data_W = mem_to_reg_W ? mem_data_W : alu_result_0_W;
  
  // ALU forwarding logic
  assign alu_operand_1 = (forward_A == 2'b00) ? reg_data_1_E :
                         (forward_A == 2'b01) ? alu_result_0_E :
                         (forward_A == 2'b10) ? reg_write_data_W : 
                         reg_data_1_E;
                       
  assign alu_operand_2 = (forward_B == 2'b00) ? (alu_src_E ? {8'b0, immediate_E} : reg_data_2_E) :
                         (forward_B == 2'b01) ? alu_result_0_E :
                         (forward_B == 2'b10) ? reg_write_data_W : 
                         (alu_src_E ? {8'b0, immediate_E} : reg_data_2_E);
  
  // =====================
  // MODULE INSTANTIATIONS
  // =====================
  
  // Program Counter
  program_counter pc_unit (
    .clk(clk),
    .rst(rst),
    .inc(!stall_F),
    .branch_en(branch_D || jump_D),
    .branch_addr(branch_D ? alu_result_0_E[10:0] : immediate_D),  // Simplified branch/jump logic
    .current_addr(pc_F)
  );
  
  // F/D Register
  FD_Register fd_reg (
    .clk(clk),
    .reset(rst),
    .stall_F(stall_F),
    .flush_F(flush_F),
    .instruction_in(instruction_F),
    .pc_in(pc_F),
    .instruction_out(instruction_D),
    .pc_out(pc_D)
  );
  
  // Control Unit
  control_unit cu (
    .opcode(opcode_D),
    .alu_src(alu_src_D),
    .reg_write(reg_write_D),
    .mem_read(mem_read_D),
    .mem_write(mem_write_D),
    .mem_to_reg(mem_to_reg_D),
    .branch(branch_D),
    .jump(jump_D),
    .write_mode(write_mode_D),
    .alu_op(alu_op_D)
  );
  
  // Register File
  reg_file reg_file_unit (
    .clk(clk),
    .rst(rst),
    .write_en_0(reg_write_W ? write_mode_W : 2'b00),
    .write_en_1((opcode_W == `MUL) || (opcode_W == `DIV) ? 2'b11 : 2'b00), // For high result in MUL/DIV
    .read_addr_0(rs1_D),
    .read_addr_1(rs2_D),
    .reg_write_addr_0(rd_W),
    .reg_write_addr_1(3'b001),  // Fixed reg for high result in MUL/DIV
    .data_in_0(reg_write_data_W),
    .data_in_1(alu_result_1_W), 
    .read_data_0(reg_data_1_D),
    .read_data_1(reg_data_2_D)
  );
  
  // D/E Register
  DE_Register de_reg (
    .clk(clk),
    .reset(rst),
    .stall_D(stall_D),
    .flush_D(flush_D),
    .opcode_in(opcode_D),
    .reg_write_addr_in(rd_D),
    .reg_data_1_in(reg_data_1_D),
    .reg_data_2_in(reg_data_2_D),
    .immediate_in(immediate_D),
    .bit_position_in(bit_pos_D),
    .pc_in(pc_D),
    .alu_src_in(alu_src_D),
    .reg_write_in(reg_write_D),
    .mem_read_in(mem_read_D),
    .mem_write_in(mem_write_D),
    .write_mode_in(write_mode_D),
    .opcode_out(opcode_E),
    .reg_write_addr_out(rd_E),
    .reg_data_1_out(reg_data_1_E),
    .reg_data_2_out(reg_data_2_E),
    .immediate_out(immediate_E),
    .bit_position_out(bit_pos_E),
    .pc_out(pc_E),
    .alu_src_out(alu_src_E),
    .reg_write_out(reg_write_E),
    .mem_read_out(mem_read_E),
    .mem_write_out(mem_write_E),
    .write_mode_out(write_mode_E)
  );
  
  // ALU
  ALU alu_unit (
    .opcode(opcode_E),
    .operand_1(alu_operand_1),
    .operand_2(alu_operand_2),
    .bit_position(bit_pos_E),
    .current_flags(current_flags_E),
    .result_0(alu_result_0_E),
    .result_1(alu_result_1_E),
    .next_flags(next_flags_E)
  );
  
  // Flag Register
  Flag_Register flag_reg (
    .clk(clk),
    .reset(rst),
    .next_flags(next_flags_E),
    .current_flags(current_flags_E)
  );
  
  // Memory
  memory data_memory (
    .clk(clk),
    .write_en(mem_write_E),
    .address(alu_result_0_E[10:0]),
    .data_in(reg_data_2_E),
    .read_data(mem_read_data_E)
  );
  
  // E/W Register
  EW_Register ew_reg (
    .clk(clk),
    .reset(rst),
    .opcode_in(opcode_E),
    .reg_write_addr_in(rd_E),
    .alu_result_0_in(alu_result_0_E),
    .alu_result_1_in(alu_result_1_E),
    .mem_data_in(mem_read_data_E),
    .reg_write_in(reg_write_E),
    .mem_to_reg_in(mem_read_E),
    .write_mode_in(write_mode_E),
    .opcode_out(opcode_W),
    .reg_write_addr_out(rd_W),
    .alu_result_0_out(alu_result_0_W),
    .alu_result_1_out(alu_result_1_W),
    .mem_data_out(mem_data_W),
    .reg_write_out(reg_write_W),
    .mem_to_reg_out(mem_to_reg_W),
    .write_mode_out(write_mode_W)
  );
  
  // Hazard Unit
  HAZARD_Unit hazard_unit (
    .opcode_D(opcode_D),
    .opcode_E(opcode_E),
    .rs1_D(rs1_D),
    .rs2_D(rs2_D),
    .rd_E(rd_E),
    .rd_W(rd_W),
    .mem_read_E(mem_read_E),
    .branch_D(branch_D),
    .jump_D(jump_D),
    .reg_write_E(reg_write_E),
    .reg_write_W(reg_write_W),
    .stall_F(stall_F),
    .stall_D(stall_D),
    .flush_F(flush_F),
    .flush_D(flush_D),
    .forward_A(forward_A),
    .forward_B(forward_B)
  );
  
  // =====================
  // CLOCK AND RESET LOGIC
  // =====================
  initial begin
    clk = 0;
    forever #5 clk = ~clk;
  end
  
  // Reset task
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
  
  // =====================
  // TEST PROGRAM
  // =====================
  initial begin
    $display("Loading test program...");
    // Sample test program
      instruction_mem[0] = {`LBL, `REG0, 8'hFF};
      instruction_mem[1] = {`LBL, `REG1, 8'hFF};
      instruction_mem[2] = {`LBL, `REG2, 8'hFF};
      instruction_mem[3] = {`LBH, `REG0, 8'hFF};
      instruction_mem[4] = {`LBH, `REG1, 8'hFF};
      instruction_mem[5] = {`LBH, `REG2, 8'hFF};
      instruction_mem[6] = {`HALT, 11'bx};
    reset;
    
    // Run for some cycles and finish
    repeat(20) @(posedge clk);
    
    if (opcode_W == `HALT)
      $display("Program completed successfully!");
    else
      $display("Program did not complete (no HALT instruction reached)");
      
    $finish;
  end
  
  // Monitor pipeline stages for debugging

  // Test block
initial begin
  $display("Starting processor test suite...");
  
  // Run all test cases sequentially
  alu_test_inc;
  repeat(20) @(posedge clk);
  $display("\n");
  
  load_test_1;
  reset;
  repeat(20) @(posedge clk);
  $display("\n");
  
  load_test_2;
  reset;
  repeat(20) @(posedge clk);
  $display("\n");
  
  mov_test_1;
  reset;
  repeat(20) @(posedge clk);
  $display("\n");
  
  load_store_test;
  reset;
  repeat(20) @(posedge clk);
  
  $display("All tests completed!");
  $finish;
end

// Add monitoring logic to track pipeline stages
initial begin
  $monitor("Time=%0t, PC_F=%0d, Instr_F=%h, Instr_D=%h, Opcode_E=%h, ALU_Result=%h, Reg_Write_W=%b, Reg_Addr_W=%d, Write_Data=%h",
           $time, pc_F, instruction_F, instruction_D, opcode_E, alu_result_0_E, reg_write_W, rd_W, reg_write_data_W);
end

initial begin
  $dumpfile("pipelined_processor.vcd");
  $dumpvars(0, pipelined_processor);
end


  task load_test_1;
    begin
      $display("load test 1");
      instruction_mem[0] = {`LBL, `REG0, 8'hFF};
      instruction_mem[1] = {`LBL, `REG1, 8'hFF};
      instruction_mem[2] = {`LBL, `REG2, 8'hFF};
      instruction_mem[3] = {`LBH, `REG0, 8'hFF};
      instruction_mem[4] = {`LBH, `REG1, 8'hFF};
      instruction_mem[5] = {`LBH, `REG2, 8'hFF};
      instruction_mem[6] = {`HALT, 11'bx};
    end
  endtask

  task load_test_2;
    begin
      $display("load test 2");
      instruction_mem[0] = {`LBL, `REG0, 8'h01};
      instruction_mem[1] = {`LBL, `REG1, 8'h02};
      instruction_mem[2] = {`LBL, `REG2, 8'hF3};
      instruction_mem[3] = {`LBH, `REG0, 8'hF4};
      instruction_mem[4] = {`LBH, `REG1, 8'hF5};
      instruction_mem[5] = {`LBH, `REG2, 8'hF6};
      instruction_mem[6] = {`HALT, 11'bx};
    end
  endtask

  task mov_test_1;
    begin
      $display("mov test 1");
      instruction_mem[0] = {`LBL, `REG0, 8'h01};
      instruction_mem[1] = {`LBL, `REG1, 8'h02};
      instruction_mem[2] = {`LBL, `REG2, 8'hF3};
      instruction_mem[3] = {`LBH, `REG0, 8'hF1};
      instruction_mem[4] = {`LBH, `REG1, 8'hF2};
      instruction_mem[5] = {`LBH, `REG2, 8'hF3};
      instruction_mem[6] = {`MOV, `REG7, `REG0, 5'bx};
      instruction_mem[7] = {`MOV, `REG6, `REG1, 5'bx};
      instruction_mem[8] = {`MOV, `REG5, `REG2, 5'bx};
      instruction_mem[9] = {`HALT, 11'bx};
    end
  endtask

  task load_store_test;
  begin
  $display("load_store_test");
    instruction_mem[0] = {`LBH, `REG0, 8'd1};
    instruction_mem[1] = {`LBL, `REG0, 8'd1};
    instruction_mem[2] = {`LBH, `REG1, 8'd255};
    instruction_mem[3] = {`LBL, `REG1, 8'd255};
    instruction_mem[4] = {`LBH, `REG2, 8'd2};
    instruction_mem[5] = {`LBL, `REG2, 8'd2};
    instruction_mem[6] = {`STORE, 3'bx, `REG2, `REG1, 2'bx};
    instruction_mem[7] = {`LOAD, `REG3, `REG2, 5'bx};
    instruction_mem[8] = {`HALT, 11'bx};
  end
  endtask

  task alu_test_inc;
  begin
  $display("alu_test_inc");
    instruction_mem[0] = {`LBH, `REG0, 8'd255};      
    instruction_mem[1] = {`LBL, `REG0, 8'd16};      
    instruction_mem[2] = {`INC, `REG1, `REG0, 5'bx};
    instruction_mem[3] = {`INC, `REG2, `REG1, 5'bx};
    instruction_mem[4] = {`INC, `REG3, `REG2, 5'bx};
    instruction_mem[5] = {`INC, `REG4, `REG3, 5'bx};
    instruction_mem[6] = {`INC, `REG5, `REG4, 5'bx};
    instruction_mem[7] = {`INC, `REG6, `REG5, 5'bx};
    instruction_mem[8] = {`INC, `REG7, `REG6, 5'bx};
    instruction_mem[9] = {`HALT,11'bx};
  end
  endtask

endmodule
