`timescale 1ps/1ps
`include "parameters.v"
module pipelined_processor(
  input wire clk,
  input wire rst,
  input wire [15:0] input_reg,
  output reg [15:0] output_reg
);
  // Register to store previous ALU result
  reg [15:0] prev_result_E;
  // =====================
  // Processor components
  // =====================
  // Instruction memory
  reg [15:0] instruction_mem[0:`MEMORY_DEPTH-1];
  // ====================
  // Connecting Wires
  // ====================
  // Fetch Stage wires
  wire [10:0] PC_F;
  wire [15:0] instruction_F;
  // Decode Stage wires
  wire [10:0] PC_D;
  wire [10:0] branch_addr_D;
  wire [15:0] instruction_D;
  wire [4:0] opcode_D;             // Opcode in decode stage
  wire [2:0] rs1_D;                // Source register 1 address
  wire [2:0] rs2_D;                // Source register 2 address
  wire [2:0] rd_D;                 // Destination register address
  wire [7:0] immediate_D;          // Immediate value
  wire [3:0] bit_pos_D;            // Bit position
  wire [15:0] reg_data_1_D;        // Register data 1
  wire [15:0] reg_data_2_D;        // Register data 2
  wire [15:0] current_flags_D;     // Current flags
  wire [15:0] next_flags_D;        // Next flags
  wire stall_DE;                    // Stall signal for decode stage
  wire flush_DE;                    // Flush signal for decode stage
  wire stall_FD;                    // Stall signal for fetch stage
  wire flush_FD;                    // Flush signal for fetch stage
  wire jump_D;                     // Jump signal for hazard unit
  wire reg_write_D;                // Register write signal
  // Control signals from decode
  wire alu_src_D;
  wire read_write_D;
  wire [1:0] write_mode_D;
  wire mem_write_D;
  wire mem_to_reg_D;
  wire branch_en_D;
  wire [10:0] branch_addr_in;
  wire inc_pc_D;                   // Increment PC signal from control unit
  wire IO_OP_D;
  // Execute Stage wires
  wire [1:0] forward_A, forward_B; // From forwarding unit (2-bit signals)
  wire [15:0] fwd_A, fwd_B;       // Forwarded data
  wire [4:0] opcode_E;             // Opcode in execute stage
  wire [2:0] reg_write_addr_E;     // Register write address (rd_E)
  wire [2:0] rs1_E;                // Source register 1 address
  wire [2:0] rs2_E;                // Source register 2 address
  wire [15:0] reg_data_1_E;        // Register data 1
  wire [15:0] reg_data_2_E;        // Register data 2
  wire [7:0] immediate_E;          // Immediate value
  wire [3:0] bit_position_E;       // Bit position
  wire [15:0] flags_E;             // Current flags in execute stage
  wire [10:0] PC_E;                // Program counter value, from decode stage
  wire ALU_EN_E;                   // ALU enable signal
  wire flag_reg_en_E;              // Flag register enable signal
  wire alu_src_E;                  // ALU source
  wire read_write_E;               // Register write
  wire [1:0] write_mode_E;         // Write mode
  wire mem_read_E;                 // Memory read signal
  wire mem_write_E;                // Memory write signal
  wire [10:0] mem_addr_E;          // Memory address
  wire mem_to_reg_E;               // Memory to register signal
  wire [15:0] mem_data_E;          // Data from Data Memory
  wire [15:0] next_flags_W;        // Next flags from ALU
  wire mem_addr_D;                 // Memory address from decode stage
  wire [10:0] branch_addr_E;       // Branch address
  wire branch_en_E;
  wire [15:0] next_flags_E;        // Next flags from ALU
  wire [15:0] alu_operand_2;       // ALU operand 2
  wire flush_EW;                   // Flush signal for execute stage
  wire stall_EW;                   // Stall signal for execute stage
  wire IO_OP_E;
  wire forward_decode_A;
  wire forward_decode_B;
  // Write back Stage wires
  wire [10:0] mem_addr_W;
  wire [15:0] mem_write_data_W;
  wire mem_to_reg_W;
  wire mem_write_W;
  wire [4:0] opcode_W;
  wire [2:0] reg_write_addr_W;
  wire [2:0] rs1_W;
  wire [2:0] rs2_W;
  wire [15:0] reg_write_data_0_W;
  wire [15:0] reg_write_data_1_W;
  wire read_write_W;
  wire [1:0] write_mode_W;
  wire flag_reg_en_W; // Flag register enable signal
  wire [15:0] flags_W;
  wire IO_OP_W;
  wire [10:0] branch_addr_W;
  wire [15:0] alu_result_0_E, alu_result_1_E;
  wire ALU_EN_D;
  wire alu_en_hzd;
  wire bit_in;
  // Assignments for fetch stage
  assign instruction_F = instruction_mem[PC_F];
  // Assignments for decode stage
  assign opcode_D = instruction_D[15:11];
  assign rd_D = instruction_D[10:8];       // Destination register
  assign rs1_D = (opcode_D == `SETB || opcode_D == `CLRB || opcode_D == `CPLB) ? instruction_D[10:8] : instruction_D[7:5];       // Source register 1
  assign rs2_D = instruction_D[4:2];       // Source register 2
  assign immediate_D = instruction_D[7:0]; // Immediate value
  assign bit_pos_D = instruction_D[7:4];   // Bit position
  assign branch_addr_in = instruction_D[10:0];
  // Assignments for execute stage
  assign bit_in = input_reg[bit_pos_E];
  // Forwarding logic
  assign fwd_A = (stall_EW || flush_EW)? reg_data_1_E: (forward_A == 2'b10) ? reg_write_data_0_W : reg_data_1_E;    
  assign fwd_B =  (stall_EW || flush_EW)? reg_data_2_E: (forward_B == 2'b10) ? 
               (stall_EW ? prev_result_E : reg_write_data_0_W) : 
               reg_data_2_E;
  wire [15:0] fwd_decode_A;
  wire [15:0] fwd_decode_B;
  assign fwd_decode_A = forward_decode_A ? reg_write_data_0_W : reg_data_1_D;
  assign fwd_decode_B = forward_decode_B ? reg_write_data_0_W : reg_data_2_D;
  assign alu_operand_2 = (alu_src_E) ? {8'h00, immediate_E} : fwd_B;
  assign mem_write_data_W = reg_data_2_E; // Data to write to memory
  assign mem_addr_W = reg_data_1_E [10:0]; // Address to write to memory

  // =======================
  // Module Instantiations
  // =======================
  // Program Counter
  program_counter PC (
    // Inputs
    .clk(clk),
    .rst(rst),
    // Controls
    .inc(inc_pc_D & ~stall_FD), // Use stall_F to control increment
    .branch_en(branch_en_E),      // Control Unit output via EW register
    .halt(~inc_pc_D),               
    .branch_addr(branch_addr_D), // Branch Register
    // Output
    .current_addr(PC_F) // To FD Register  
  );

  FD_Register FD_Reg (
    // Inputs
    .clk(clk),
    .reset(rst),
    .stall_F(stall_FD), // From Hazard Unit   
    .flush_F(flush_FD), // From Hazard Unit  
    .instruction_in(instruction_F), // From Instruction Memory  
    .pc_in(PC_F), // From Program Counter  
    // Outputs
    .instruction_out(instruction_D), // To D/E Register  
    .pc_out(PC_D) // To D/E Register  
  );

  branch_register Branch_Reg (
    // Inputs
    .reset(rst),
    .branch_addr_in(branch_addr_W), // From e/w register  
    .branch_en(branch_en_D), // From control Unit  
    // Output
    .branch_addr_out(branch_addr_D) // To Program Counter  
  );

  reg_file reg_file_unit (
    // Inputs
    .clk(clk),
    .rst(rst),
    // Read ports (decode stage)
    .read_addr_0(rs1_D), // From FD Register  
    .read_addr_1(rs2_D), // From FD Register  
    .read_data_0(reg_data_1_D), // To DE Register  
    .read_data_1(reg_data_2_D), // To DE Register  
    // Write ports (writeback stage) 
    .reg_write_en(read_write_W), // From E/W Register  
    .write_mode(write_mode_W), // From E/W Register
    .reg_write_addr_0(reg_write_addr_W), // From E/W Register  
    .reg_write_addr_1(reg_write_addr_W == 3'b111 ? 3'b000 : reg_write_addr_W + 3'b001), // From E/W Register  
    .data_in_0(reg_write_data_0_W), // From E/W Register
    .data_in_1(reg_write_data_1_W) // From E/W Register   
  );

  Flag_Register flag_reg (
    // Inputs
    .reset(rst),
    .flag_reg_en(flag_reg_en_W), // From E/W Register
    .next_flags(next_flags_W),
    // Outputs
    .current_flags(current_flags_D) // To DE Register
  );

  control_unit control_unit (
    // Inputs
    .opcode(opcode_D), // From DE Register  
    .flag_reg_values(current_flags_D),
    .flag_index(bit_pos_D),
    // Outputs
    .alu_src(alu_src_D), // To DE Register  
    .read_write(read_write_D), // To Register file  
    .mem_write(mem_write_D), // To Data Memory, via EW Register  
    .mem_to_reg(mem_to_reg_D), // To EW Register  
    .branch_en(branch_en_D), // To Program Counter  
    .inc_pc(inc_pc_D), // To DE Register  
    .jump(jump_D), // To Hazard Unit  
    .mem_read(mem_addr_D), // To Data Memory  
    .alu_op(ALU_EN_D), // To DE Register  
    .io_op(IO_OP_D), // To DE Register  
    .write_mode(write_mode_D) // To Register file via E/W Register  
  );

  DE_Register DE_Reg (
    // Inputs
    .clk(clk),
    .reset(rst),
    .stall_D(stall_DE), // From Hazard Unit  
    .flush_D(flush_DE), // From Hazard Unit  
    .opcode_in(opcode_D), // From FD Register  
    .reg_write_addr_in(rd_D), // From FD Register  
    .source_reg1_in(rs1_D), // From FD Register  
    .source_reg2_in(rs2_D), // From FD Register  
    .reg_data_1_in(fwd_decode_A), // From Register File  
    .reg_data_2_in(fwd_decode_B),  // From Register File  
    .immediate_in(immediate_D), // From FD Register  
    .bit_position_in(bit_pos_D), // From FD Register  
    .pc_in(PC_D), // From FD Register  
    .flags_in(current_flags_D), // From Flag Register  
    .branch_addr_in(branch_addr_in), // From FD Register  
    // Control signals inputs
    .alu_src_in(alu_src_D),
    .reg_write_in(write_mode_D),
    .mem_write_in(mem_write_D),
    .mem_to_reg_in(mem_to_reg_D),
    .mem_read_in(mem_addr_D),
    .read_write_in(read_write_D),
    .alu_op_in(ALU_EN_D), // ALU enable signal input  
    .branch_en_in(branch_en_D),           // From Control Unit
    .io_op_in(IO_OP_D), // From control unit  
    // Outputs for control signals
    .alu_src_out(alu_src_E),
    .read_write_out(read_write_E),
    .mem_write_out(mem_write_E),
    .mem_to_reg_out(mem_to_reg_E),
    .mem_read_out(mem_read_E),
    .write_mode_out(write_mode_E),
    // Outputs for data
    .opcode_out(opcode_E),
    .reg_write_addr_out(reg_write_addr_E),
    .source_reg1_out(rs1_E), 
    .source_reg2_out(rs2_E),
    .reg_data_1_out(reg_data_1_E),
    .reg_data_2_out(reg_data_2_E),
    .immediate_out(immediate_E),
    .bit_pos_E(bit_pos_E),    // bit position that is read from the instruction
    .bit_position_out(bit_position_E),
    .pc_out(PC_E),
    .flags_out(flags_E),
    .branch_addr_out(branch_addr_E),
    .mem_read_addr_out(mem_addr_E),
    .alu_op_out(ALU_EN_E), // To ALU  
    .io_op_out(IO_OP_E),
    .branch_en_out(branch_en_E)
  );

  ALU alu(
    // Inputs
    .alu_en(ALU_EN_E), // From DE Register 
    .opcode(opcode_E), // From DE Register  
    .operand_1(fwd_A), // From DE Register  
    .operand_2(alu_operand_2), // From DE Register  
    .bit_in(bit_in),
    .bit_position(bit_position_E), // From DE Register  
    .immediate(immediate_E), // From DE Register  
    .current_flags(flags_E), // From DE Register  
    .rd(reg_write_addr_E), // From DE Register  
    // Outputs
    .result_0(alu_result_0_E), // To E/W Register  
    .result_1(alu_result_1_E), // To E/W Register   
    .alu_en_out(flag_reg_en_E), // To E/W Register  
    .next_flags(next_flags_E)  // To E/W Register    
    );


  EW_Register EW_Reg (
    // Inputs
    .clk(clk),
    .reset(rst),
    .opcode_in(opcode_E), // From DE Register  
    .reg_write_addr_in(reg_write_addr_E), // From DE Register, rd_E  
    .source_reg1_in(rs1_E), // From DE Register, rs1_E  
    .source_reg2_in(rs2_E), // From DE Register, rs1_E  
    .alu_result_0_in(alu_result_0_E), // From ALU Execute stage  
    .alu_result_1_in(alu_result_1_E), // From ALU Execute stage  
    .prev_alu_result_0(prev_result_E), 
    .mem_data_in(mem_data_E), // From mem Execute stage, data read out of memory  
    .flags_in(next_flags_E), // From ALU Execute stage  
    .branch_addr_in(branch_addr_E), // To Branch Register  
    .input_port_in(input_reg), // From input register  
    // Controls
    .stall_E(stall_EW), // From hazard unit 
    .read_write_in(read_write_E), // From DE Register  
    .write_mode_in(write_mode_E), // From Control Unit  
    .flag_reg_en_in(flag_reg_en_E), // From ALU
    .mem_to_reg_in(mem_to_reg_E), // From DE Register  
    .mem_write_in(mem_write_E), // From DE Register  
    .io_op_in(IO_OP_E), // From DE Register  
    // Outputs
    .flush_E(flush_EW), // from hazard Unit  
    .opcode_out(opcode_W), // To Hazard unit  
    .reg_write_addr_out(reg_write_addr_W), // To Register file  
    .source_reg1_out(rs1_W), // To Hazard Unit
    .source_reg2_out(rs2_W), // To Hazard Unit
    .reg_write_data_0_out(reg_write_data_0_W), // To Register File  
    .reg_write_data_1_out(reg_write_data_1_W), // To Register File  
    .read_write_out(read_write_W), // To Register File  
    .write_mode_out(write_mode_W), // To Register File  
    .flag_reg_en_out(flag_reg_en_W), // To flag register
    .flags_out(next_flags_W), // To Flag Register  
    .branch_addr_out(branch_addr_W), // To Branch Register  
    .mem_addr_out(mem_addr_W), // To Data Memory  
    .mem_write_data_out(mem_write_data_W), // To Data Memory  
    .mem_write_out(mem_write_W), // To Data Memory
    .io_op_out(IO_OP_W),  // Unconnected
    .mem_to_reg_out(mem_to_reg_W) // To Register File  
  );

  memory data_memory (
    // Inputs
    .clk(clk),
    .read_address(mem_addr_E), // From D/E Register  
    .write_address(mem_addr_W), // From E/W Register  
    .data_in(mem_write_data_W), // From E/W Register  
    .write_en(mem_write_W), // From Control Unit via E/W Register  
    // Outputs
    .data_out(mem_data_E) // To E/W Register  
  );

  HAZARD_Unit hazard_unit(
  // Inputs
  .clk(clk),
  .alu_en_in(ALU_EN_D),                // From DE Register
  .opcode_D(opcode_D),              // From Decode stage
  .opcode_E(opcode_E),              // From Execute stage
  .opcode_W(opcode_W),
  .rd_D(rd_D),                      // From Decode stage
  .source_reg1_D(rs1_D),                    // From Decode stage
  .source_reg2_D(rs2_D),                    // From Decode stage
  .rd_E(reg_write_addr_E),          // From Execute stage
  .source_reg1_E(rs1_E),                    // From Execute stage
  .source_reg2_E(rs2_E),                    // From Execute stage
  .rd_W(reg_write_addr_W),          // From Writeback stage
  .source_reg1_W(rs1_W),                    // From Writeback stage
  .source_reg2_W(rs2_W),                    // From Writeback stage
  .mem_read_E(mem_read_E),          // From Control Unit via DE Register
  .branch_en(branch_en_E),              // From Control Unit
  .jump_hzd(jump_D),                    // Connect to control_unit
  .reg_write_D(reg_write_D),         // From Control Unit via DE Register
  .reg_write_E(write_mode_E [0]),    // From Control Unit via DE Register
  .reg_write_W(write_mode_W[0]),    // From Control Unit via EW Register
  // Outputs
  .stall_F(stall_FD),                // To FD Register
  .stall_D(stall_DE),                // To DE Register
  .stall_E(stall_EW),                // To EW Register
  .flush_F(flush_FD),                // To FD Register
  .flush_D(flush_DE),                // To DE Register
  .flush_E(flush_EW),                // To EW Register
  .forward_A(forward_A),            // To forwarding logic
  .forward_B(forward_B),             // To forwarding logic
  .forward_decode_A (forward_decode_A),
  .forward_decode_B (forward_decode_B),
  .alu_en_out(alu_en_hzd)  // Unconnected
);

// =====================
// TEST PROGRAM
// =====================

  initial begin
    $display("\n=== Starting Processor Test ===\n");
    // Load test program
      instruction_mem[1] = {`MOVIN, 3'b0, `REG4, 5'b0}; // I need the 3rd fibonacci number
      instruction_mem[2] = {`LBH, `REG5, 8'h00};
      instruction_mem[3] = {`LBL, `REG5, 8'h01}; // Loading 0001, to decrement counter
      instruction_mem[4] = {`LBH, `REG0, 8'h00}; // Let this be reg A       
      instruction_mem[5] = {`LBL, `REG0, 8'h01}; // reg A = 0001
      instruction_mem[6] = {`LBH, `REG1, 8'h00}; // Let this be reg B 
      instruction_mem[7] = {`LBL, `REG1, 8'h01}; // reg B = 0001       
      instruction_mem[8] = {`ADD, `REG2, `REG0, `REG1, 2'b0}; // reg C = reg A + reg B
      instruction_mem[9] = {`MOV, `REG0, `REG1, 5'b0}; // A <- B      
      instruction_mem[10] = {`MOV, `REG1, `REG2, 5'b0}; // B <- C      
      instruction_mem[11] = {`SUB, `REG4, `REG4, `REG5, 2'b0}; // Decrement Counter reg
      instruction_mem[12] = {`CMP, 3'b0, `REG4, `REG5, 2'b0}; // Sets the cmp flag
      instruction_mem[13] = {`LOADBR, 11'd8};              
      instruction_mem[14] = {`JF, 3'b0, 4'b0010,4'b0};
      instruction_mem[15] = {`MOVOUT, 3'b0, `REG2, 5'b0};
  // Debug trace
  repeat(76) @(posedge clk) begin
    $display("\nTime=%0t: Clock cycle", $time);
    $display("Fetch    : PC=%h, Instruction=%h", PC_F, instruction_F);
    $display("Decode   : Opcode=%h, rs1=%h, rs2=%h, rd=%h, alu_en_d = %b, io_op_D = %b", opcode_D, rs1_D, rs2_D, rd_D, ALU_EN_D, IO_OP_D);
    $display("Execute  : ALU_out=%h, Operand1 = %h, Operand2 = %h, ALU_en = %b, forward_A = %b, forward_B = %b, prev_result = %h, fwd_B = %h, input_reg_data = %h", alu_result_0_E, fwd_A, alu_operand_2,ALU_EN_E, forward_A, forward_B, prev_result_E, fwd_B, input_reg);
    $display("Writeback: WriteAddr=%h, WriteData=%h, WriteEn=%b, Flag_write_en = %b, Flag_register = %b", reg_write_addr_W, reg_write_data_0_W, read_write_W, flag_reg_en_W, next_flags_W);
    $display("\n Branch debug from processor_top.v: branch_en_D = %b, branch_en_E = %b\n", branch_en_D,branch_en_E);
  end
  // Display final state
  $display("\n=== Final Processor State ===");
  $display("Register File:");
  $display("R0 = %h", reg_file_unit.registers[0]);
  $display("R1 = %h", reg_file_unit.registers[1]);
  $display("R2 = %h", reg_file_unit.registers[2]);
  $display("R3 = %h", reg_file_unit.registers[3]);
  $display("R4 = %h", reg_file_unit.registers[4]);
  $display("R5 = %h", reg_file_unit.registers[5]);
  $display("R6 = %h", reg_file_unit.registers[6]);
  $display("Output register contents: %h", output_reg);
  $display("\nFlags = %b", current_flags_D);
  $finish;
end

always @(posedge clk or posedge rst) begin
  if (rst) begin
    prev_result_E <= 16'b0;
  end else if (flush_EW) begin 
    prev_result_E <= alu_result_0_E;
  end
  else if(opcode_E == `MOVOUT) begin
    output_reg <= fwd_A;
  end
end
endmodule

module pipelined_processor_tb;
reg clk;
reg rst;
reg [15:0] input_reg;
wire [15:0] output_reg;

  initial begin
    clk = 0;
    forever #5 clk = ~clk;
  end
  initial begin
    rst = 1'b1;
    // Wait for reset
    repeat(2) @(posedge clk);
    $display("Time=%0t: Reset released", $time);
    rst = 1'b0;
    input_reg <= 16'h0007;
  end
  initial begin
    $monitor("Time=%0t: Output Register = %h", $time, output_reg);
  end
  pipelined_processor dut(
    .clk(clk),
    .rst(rst),
    .input_reg(input_reg),
    .output_reg(output_reg)
  );
endmodule
