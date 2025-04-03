`timescale 1ps/1ps
`include "parameters.v"
//////////////////////////////////////////////////////////////////////////////////
// Company: Student at College of Engg, Trivandrum
// Engineer: Vasanthi
//
// Create Date: 22.03.2025 12:42:46
// Design Name: Processor
// Module Name: processor.v
// Project Name: MINI-RISC Processor
// Target Devices: Basys3 Artix-7 FPGA
// Tool Versions: 
// Description: Final Processor Module
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments: 
// This is a 16-bit processor, with a 4 stage pipeline. 
// Certain design specifications: 
// 1. All access to data memory is through register-addressing only. (LOAD [Ra], Rb or STORE [Ra], Rb). 
// 2. Register 0 is not hardwired to zero. 
// 3. I have a special function register, called branch register. In this, I will load the 11 bit address, to which I have to jump. I use an instruction called LOADBR. Then I use JF (Jump on Flag) to make the jump. It only uses absolute addressing. 
// 4. I have 2 read ports and 2 write ports. I write using the second write port, only to store the higher byte in multiplication operation. 
//     Otherwise, the second write port is not used. 
//     The LSB of the write mode is used to control the write enable signal (reg_write_en). 
//     If 00: read, 01: write a word through port 1, 11: write a word from both ports, to two registers. 
// 5. ALU operates only on registers. It does not take immediate values.
// 6. LBL, LBH, MOV are non-ALU operations. They take the path through the ALU, but the ALU is disabled.
// 
// TODO: Make IO Ports
//////////////////////////////////////////////////////////////////////////////////

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
    .halt(1'b0),               
    .branch_addr(branch_addr_D), // Branch Register

    // Output
    .current_addr(PC_F) // To FD Register (checked)
  );

  FD_Register FD_Reg (
    // Inputs
    .clk(clk),
    .reset(rst),
    .stall_F(stall_FD), // From Hazard Unit  (checked)
    .flush_F(flush_FD), // From Hazard Unit (checked)
    .instruction_in(instruction_F), // From Instruction Memory (checked)
    .pc_in(PC_F), // From Program Counter (checked)

    // Outputs
    .instruction_out(instruction_D), // To D/E Register (checked)
    .pc_out(PC_D) // To D/E Register (checked)
  );

  branch_register Branch_Reg (
    // Inputs
    .reset(rst),
    .branch_addr_in(branch_addr_W), // From e/w register (checked)
    .branch_en(branch_en_D), // From control Unit (checked)
    // Output
    .branch_addr_out(branch_addr_D) // To Program Counter (checked)
  );

  reg_file reg_file_unit (
    // Inputs
    .clk(clk),
    .rst(rst),
    
    // Read ports (decode stage)
    .read_addr_0(rs1_D), // From FD Register (checked)
    .read_addr_1(rs2_D), // From FD Register (checked)
    .read_data_0(reg_data_1_D), // To DE Register (checked)
    .read_data_1(reg_data_2_D), // To DE Register (checked)
    
    // Write ports (writeback stage) 
    .reg_write_en(read_write_W), // From E/W Register (checked)
    .write_mode(write_mode_W), // From E/W Register (checked) (unchecked: flow of read_write from control unit to E/W register)
    .reg_write_addr_0(reg_write_addr_W), // From E/W Register (checked)
    .reg_write_addr_1(reg_write_addr_W == 3'b111 ? 3'b000 : reg_write_addr_W + 3'b001), // From E/W Register (checked)
    .data_in_0(reg_write_data_0_W), // From E/W Register (checked) (unchecked: flow of reg_write_data from E/W register to reg_file, forwarding unit)
    .data_in_1(reg_write_data_1_W) // From E/W Register (checked) 
  );

  Flag_Register flag_reg (
    // Inputs
    .reset(rst),
    .flag_reg_en(flag_reg_en_W), // From E/W Register (trial run)
    .next_flags(next_flags_W),
    // Outputs
    .current_flags(current_flags_D) // To DE Register
  );

  control_unit control_unit (
    // Inputs
    .opcode(opcode_D), // From DE Register (checked)
    .flag_reg_values(current_flags_D),
    .flag_index(bit_pos_D),
    // Outputs
    .alu_src(alu_src_D), // To DE Register (checked)
    .read_write(read_write_D), // To Register file (checked)
    .mem_write(mem_write_D), // To Data Memory, via EW Register (checked)
    .mem_to_reg(mem_to_reg_D), // To EW Register (checked)
    .branch_en(branch_en_D), // To Program Counter (checked)
    .inc_pc(inc_pc_D), // To DE Register (checked)
    .jump(jump_D), // To Hazard Unit (checked)
    .mem_read(mem_addr_D), // To Data Memory (checked)
    .alu_op(ALU_EN_D), // To DE Register (checked)
    .io_op(IO_OP_D), // To DE Register (checked)
    .write_mode(write_mode_D) // To Register file via E/W Register (checked)
  );

  DE_Register DE_Reg (
    // Inputs
    .clk(clk),
    .reset(rst),
    .stall_D(stall_DE), // From Hazard Unit (checked)
    .flush_D(flush_DE), // From Hazard Unit (checked)
    .opcode_in(opcode_D), // From FD Register (checked)
    .reg_write_addr_in(rd_D), // From FD Register (checked)
    .source_reg1_in(rs1_D), // From FD Register (checked)
    .source_reg2_in(rs2_D), // From FD Register (checked)
    .reg_data_1_in(fwd_decode_A), // From Register File (checked)
    .reg_data_2_in(fwd_decode_B),  // From Register File (checked)
    .immediate_in(immediate_D), // From FD Register (checked)
    .bit_position_in(bit_pos_D), // From FD Register (checked)
    .pc_in(PC_D), // From FD Register (checked)
    .flags_in(current_flags_D), // From Flag Register (checked)
    .branch_addr_in(branch_addr_in), // From FD Register (checked)

    // Control signals inputs
    .alu_src_in(alu_src_D),
    .reg_write_in(write_mode_D),
    .mem_write_in(mem_write_D),
    .mem_to_reg_in(mem_to_reg_D),
    .mem_read_in(mem_addr_D),
    .read_write_in(read_write_D),
    .alu_op_in(ALU_EN_D), // ALU enable signal input (checked)
    .branch_en_in(branch_en_D),           // From Control Unit
    .io_op_in(IO_OP_D), // From control unit (checked)
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
    .alu_op_out(ALU_EN_E), // To ALU (checked)
    .io_op_out(IO_OP_E),
    .branch_en_out(branch_en_E)
  );

  ALU alu(
    // Inputs
    .alu_en(ALU_EN_E), // From DE Register(checked)
    .opcode(opcode_E), // From DE Register (checked)
    .operand_1(fwd_A), // From DE Register (checked)
    .operand_2(alu_operand_2), // From DE Register (checked)
    .bit_in(bit_in),
    .bit_position(bit_position_E), // From DE Register (checked)
    .immediate(immediate_E), // From DE Register (checked)
    .current_flags(flags_E), // From DE Register (checked)
    .rd(reg_write_addr_E), // From DE Register (checked)
    // Outputs
    .result_0(alu_result_0_E), // To E/W Register (checked)
    .result_1(alu_result_1_E), // To E/W Register (checked) 
    .alu_en_out(flag_reg_en_E), // To E/W Register (checked)
    .next_flags(next_flags_E)  // To E/W Register (checked)  
    );


  EW_Register EW_Reg (
    // Inputs
    .clk(clk),
    .reset(rst),
    .opcode_in(opcode_E), // From DE Register (checked)
    .reg_write_addr_in(reg_write_addr_E), // From DE Register, rd_E (checked)
    .source_reg1_in(rs1_E), // From DE Register, rs1_E (checked)
    .source_reg2_in(rs2_E), // From DE Register, rs1_E (checked)
    .alu_result_0_in(alu_result_0_E), // From ALU Execute stage (checked)
    .alu_result_1_in(alu_result_1_E), // From ALU Execute stage (checked)
    .prev_alu_result_0(prev_result_E), 
    .mem_data_in(mem_data_E), // From mem Execute stage, data read out of memory (checked)
    .flags_in(next_flags_E), // From ALU Execute stage (checked)
    .branch_addr_in(branch_addr_E), // To Branch Register (checked)
    .input_port_in(input_reg), // From input register (checked)
    // Controls
    .stall_E(stall_EW), // From hazard unit 
    .read_write_in(read_write_E), // From DE Register (checked)
    .write_mode_in(write_mode_E), // From Control Unit (checked)
    .flag_reg_en_in(flag_reg_en_E), // From ALU (trial run)
    .mem_to_reg_in(mem_to_reg_E), // From DE Register (checked)
    .mem_write_in(mem_write_E), // From DE Register (checked)
    .io_op_in(IO_OP_E), // From DE Register (checked)

    // Outputs
    .flush_E(flush_EW), // from hazard Unit (checked)
    .opcode_out(opcode_W), // To Hazard unit (checked)
    .reg_write_addr_out(reg_write_addr_W), // To Register file (checked)
    .source_reg1_out(rs1_W), // To Hazard Unit
    .source_reg2_out(rs2_W), // To Hazard Unit
    .reg_write_data_0_out(reg_write_data_0_W), // To Register File (checked)
    .reg_write_data_1_out(reg_write_data_1_W), // To Register File (checked)
    .read_write_out(read_write_W), // To Register File (checked)
    .write_mode_out(write_mode_W), // To Register File (checked)
    .flag_reg_en_out(flag_reg_en_W), // To flag register (trial run)
    .flags_out(next_flags_W), // To Flag Register (checked)
    .branch_addr_out(branch_addr_W), // To Branch Register (checked)
    .mem_addr_out(mem_addr_W), // To Data Memory (checked)
    .mem_write_data_out(mem_write_data_W), // To Data Memory (checked)
    .mem_write_out(mem_write_W), // To Data Memory
    .io_op_out(IO_OP_W),  // Unconnected
    .mem_to_reg_out(mem_to_reg_W) // To Register File (checked)
  );

  memory data_memory (
    // Inputs
    .clk(clk),
    .read_address(mem_addr_E), // From D/E Register (checked)
    .write_address(mem_addr_W), // From E/W Register (checked)
    .data_in(mem_write_data_W), // From E/W Register (checked)
    .write_en(mem_write_W), // From Control Unit via E/W Register (checked)
    // Outputs
    .data_out(mem_data_E) // To E/W Register (checked)
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
  
  
  // Monitor pipeline stages
  repeat(76) @(posedge clk) begin
    /* $display("\nTime=%0t: Clock cycle", $time);
    $display("Fetch    : PC=%h, Instruction=%h", PC_F, instruction_F);
    $display("Decode   : Opcode=%h, rs1=%h, rs2=%h, rd=%h, alu_en_d = %b, io_op_D = %b", opcode_D, rs1_D, rs2_D, rd_D, ALU_EN_D, IO_OP_D);
    $display("Execute  : ALU_out=%h, Operand1 = %h, Operand2 = %h, ALU_en = %b, forward_A = %b, forward_B = %b, prev_result = %h, fwd_B = %h, input_reg_data = %h", alu_result_0_E, fwd_A, alu_operand_2,ALU_EN_E, forward_A, forward_B, prev_result_E, fwd_B, input_reg);
    $display("Writeback: WriteAddr=%h, WriteData=%h, WriteEn=%b, Flag_write_en = %b, Flag_register = %b", reg_write_addr_W, reg_write_data_0_W, read_write_W, flag_reg_en_W, next_flags_W);
    $display("\n Branch debug from processor_top.v: branch_en_D = %b, branch_en_E = %b\n", branch_en_D,branch_en_E); */
    //$display("\n Branch debug from processor_top.v: branch_en_D = %b, branch_en_E = %b\n", branch_en_D,branch_en_E);
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
  end else if (flush_EW) begin  // Only update when not stalled
    prev_result_E <= alu_result_0_E;
  end
  else if(opcode_E == `MOVOUT) begin
    output_reg <= fwd_A;
  end
end


/* Fibonacci series
LBH REG4, #00           instruction_mem[1] = {`LBH, `REG4, 8'h00};
LBL REG4, #02           instruction_mem[2] = {`LBL, `REG4, 8'h01};
LOAD REG5, [REG4]       instruction_mem[3] = {`LOAD, `REG5, `REG4, 5'b0};
LBH REG6, #00           instruction_mem[4] = {`LBH, `REG6, 8'h00};       
LBH REG6, #01           instruction_mem[5] = {`LBL, `REG6, 8'h01};        
LBH REG0, #00           instruction_mem[6] = {`LBH, `REG0, 8'h00};        
LBL REG0, #01           instruction_mem[7] = {`LBL, `REG0, 8'h01};        
LBH REG1, #00           instruction_mem[8] = {`LBH, `REG1, 8'h00};        
LBL REG1, #01           instruction_mem[9] = {`LBL, `REG1, 8'h01};        

loop:                   
ADD REG2, REG0, REG1    instruction_mem[10] = {`ADD, `REG2, `REG0, `REG1, 2'b0};    
MOV REG0, REG1          instruction_mem[11] = {`MOV, `REG0, `REG1, 5'b0};           
MOV REG1, REG2          instruction_mem[12] = {`MOV, `REG1, `REG2, 5'b0};           
SUB REG5, REG5, REG6    instruction_mem[13] = {`SUB, `REG5, `REG5, `REG6, 2'b0};  
LOADBR #000A            instruction_mem[14] = {`LOADBR, 11'h000A};                  
JF #3                   instruction_mem[15] = {`JF, 3'b0, 4'b0011,4'b0};            


*/
/* instruction_mem[1] = {`LBH, `REG0, 8'hFF};
  instruction_mem[2] = {`LBL, `REG0, 8'hFF};
  instruction_mem[3] = {`LBH, `REG1, 8'h01};
  instruction_mem[4] = {`LBL, `REG1, 8'h01};
  instruction_mem[5] = {`SETB, `REG2, 4'b0000, 4'b0};
  instruction_mem[6] = {`SUB, `REG3, `REG0, `REG2, 2'b0};
  instruction_mem[7] = {`NOP, 11'b0};
  instruction_mem[8] = {`SETF, 3'b0, 4'b000, 4'b0};
  instruction_mem[9] = {`LOADBR, 11'd14};
  instruction_mem[10] = {`JF, 3'b0, 4'b000, 4'b0};
  instruction_mem [11] = {`NOP, 11'b0};
  instruction_mem [12] = {`NOP, 11'b0};
  instruction_mem [13] = {`ADD, `REG4, `REG1, `REG2, 2'b0};
  instruction_mem [14] = {`ADD, `REG5, `REG0, `REG1, 2'b0};
  instruction_mem[15] = {`NOP, 11'b0}; */

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
