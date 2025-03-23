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
// TODO: Fix the hazard unit and Forwarding unit and IO Ports
// TODO: Add a reset to branch register
//////////////////////////////////////////////////////////////////////////////////

module pipelined_processor;
  
  // Special registers
  reg clk;
  reg rst;

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
  wire branch_D;
  wire [10:0] branch_addr_in;
  wire inc_pc_D;                   // Increment PC signal from control unit

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
  wire [15:0] next_flags_E;        // Next flags from ALU
  wire [15:0] alu_operand_2;       // ALU operand 2

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
  wire [15:0] flags_W;
  wire [10:0] branch_addr_W;
  wire [15:0] alu_result_0_E, alu_result_1_E;
  wire ALU_EN_D;

  // Assignments for fetch stage
  assign instruction_F = instruction_mem[PC_F];

  // Assignments for decode stage
  assign opcode_D = instruction_D[15:11];
  assign rd_D = instruction_D[10:8];       // Destination register
  assign rs1_D = instruction_D[7:5];       // Source register 1
  assign rs2_D = instruction_D[4:2];       // Source register 2
  assign immediate_D = instruction_D[7:0]; // Immediate value
  assign bit_pos_D = instruction_D[4:1];   // Bit position
  assign branch_addr_in = instruction_D[10:0];

  
  // Forwarding logic
  assign fwd_A = (forward_A == 2'b10) ? alu_result_0_E :
              (forward_A == 2'b01) ? reg_write_data_0_W :
              reg_data_1_E;
              
  assign fwd_B = (forward_B == 2'b10) ? alu_result_0_E :
              (forward_B == 2'b01) ? reg_write_data_0_W :
              reg_data_2_E;
// Assignments for execute stage
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
    .inc(inc_pc_D & ~stall_FD), // Use stall_F to control increment (checked)
    .branch_en(branch_D),      // Control Unit output (checked)
    .halt(1'b0),               // Hazard Unit output? (unchecked)
    .branch_addr(branch_addr_D), // Branch Register (checked)

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
    .branch_en(branch_D), // From control Unit (checked)
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
    .clk(clk),
    .reset(rst),
    .next_flags(next_flags_W), // From ALU via EW register
    // Outputs
    .current_flags(current_flags_D) // To DE Register
  );

  control_unit control_unit (
    // Inputs
    .opcode(opcode_D), // From DE Register (checked)
    // Outputs
    .alu_src(alu_src_D), // To DE Register (checked)
    .read_write(read_write_D), // To Register file (checked)
    .mem_write(mem_write_D), // To Data Memory, via EW Register (checked)
    .mem_to_reg(mem_to_reg_D), // To EW Register (checked)
    .branch(branch_D), // To Program Counter (checked)
    .inc_pc(inc_pc_D), // To Program Counter (checked)
    .jump(jump_D), // To Hazard Unit (checked)
    .mem_read(mem_addr_D), // To Data Memory (checked)
    .alu_op(ALU_EN_D), // To DE Register (checked)
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
    .source_reg1_in(rs1_D), // From FD Register (checked) // NEWLY ADDED
    // Try priting out this
    .source_reg2_in(rs2_D), // From FD Register (checked) // NEWLY ADDED
    .reg_data_1_in(reg_data_1_D), // From Register File (checked)
    // Try priting out this
    .reg_data_2_in(reg_data_2_D),  // From Register File (checked)
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
    .bit_position_out(bit_position_E),
    .pc_out(PC_E),
    .flags_out(flags_E),
    .branch_addr_out(branch_addr_E),
    .mem_read_addr_out(mem_addr_E),
    .alu_op_out(ALU_EN_E) // To ALU (checked)
  );

  ALU alu(
    // Inputs
    .alu_en(ALU_EN_E), // From DE Register(checked)
    .opcode(opcode_E), // From DE Register (checked)
    .operand_1(fwd_A), // From DE Register (checked)
    .operand_2(alu_operand_2), // From DE Register (checked)
    .bit_position(bit_position_E), // From DE Register (checked)
    .immediate(immediate_E), // From DE Register (checked)
    .current_flags(flags_E), // From DE Register (checked)
    // Outputs
    .result_0(alu_result_0_E), // To E/W Register (checked)
    .result_1(alu_result_1_E), // To E/W Register (checked) 
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
    .mem_data_in(mem_data_E), // From mem Execute stage, data read out of memory (checked)
    .flags_in(next_flags_E), // From ALU Execute stage (checked)
    .branch_addr_in(branch_addr_E), // To Branch Register (checked)
    // Controls
    .read_write_in(read_write_E), // From DE Register (checked)
    .write_mode_in(write_mode_E), // From Control Unit (checked)
    .mem_to_reg_in(mem_to_reg_E), // From DE Register (checked)
    .mem_write_in(mem_write_E), // From DE Register (checked)

    // Outputs
    .opcode_out(opcode_W), // To Writeback (unused)
    .reg_write_addr_out(reg_write_addr_W), // To Register file (checked)
    .source_reg1_out(rs1_W), // To Hazard Unit
    .source_reg2_out(rs2_W), // To Hazard Unit
    .reg_write_data_0_out(reg_write_data_0_W), // To Register File (checked)
    .reg_write_data_1_out(reg_write_data_1_W), // To Register File (checked)
    .read_write_out(read_write_W), // To Register File (checked)
    .write_mode_out(write_mode_W), // To Register File (checked)
    .flags_out(next_flags_W), // To Flag Register (checked)
    .branch_addr_out(branch_addr_W), // To Branch Register (checked)
    .mem_addr_out(mem_addr_W), // To Data Memory (checked)
    .mem_write_data_out(mem_write_data_W), // To Data Memory (checked)
    .mem_write_out(mem_write_W), // To Data Memory
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
  .opcode_D(opcode_D),              // From Decode stage
  .opcode_E(opcode_E),              // From Execute stage
  .source_reg1_D(rs1_D),                    // From Decode stage
  .source_reg2_D(rs2_D),                    // From Decode stage
  .rd_E(reg_write_addr_E),          // From Execute stage
  .source_reg1_E(rs1_E),                    // From Execute stage
  .source_reg2_E(rs2_E),                    // From Execute stage
  .rd_W(reg_write_addr_W),          // From Writeback stage
  .source_reg1_W(rs1_W),                    // From Writeback stage
  .source_reg2_W(rs2_W),                    // From Writeback stage
  .mem_read_E(mem_read_E),          // From Control Unit via DE Register
  .branch(branch_D),              // From Control Unit
  .jump_hzd(jump_D),                    // Connect to control_unit
  .reg_write_E(write_mode_E [0]),    // From Control Unit via DE Register
  .reg_write_W(write_mode_W[0]),    // From Control Unit via EW Register
  // Outputs
  .stall_F(stall_FD),                // To FD Register
  .stall_D(stall_DE),                // To DE Register
  .flush_F(flush_FD),                // To FD Register
  .flush_D(flush_DE),                // To DE Register
  .forward_A(forward_A),            // To forwarding logic
  .forward_B(forward_B)             // To forwarding logic
);

// =====================
// TEST PROGRAM
// =====================
initial begin
  // Initialize clk
  clk = 0;
  
  // Generate clock indefinitely
  forever #5 clk = ~clk;
end

initial begin
  // Test program initialization
  $display("\n=== Starting Processor Test ===\n");
  
  // Initialize rst
  rst = 1'b1;
  
  // Load test program
  instruction_mem[1] = {`LBL, `REG0, 8'h05};      // Load 0x05 into R0
  instruction_mem[2] = {`LBL, `REG1, 8'h03};      // Load 0x03 into R1: 1011000100000011: b103
  instruction_mem[3] = {`MOV, `REG2, `REG0, 5'b0};      // Copy R0 to R2 : 110000100000000: c200
  instruction_mem[4] = {`ADD, `REG3, `REG0, `REG1,2'b0}; // R3 = R0 + R1 : 0000001100000100: 0304
  
  // Wait for reset
  repeat(2) @(posedge clk);
  $display("Time=%0t: Reset released", $time);
  rst = 1'b0;
  
  // Monitor pipeline stages
  repeat(9) @(posedge clk) begin
    $display("\nTime=%0t: Clock cycle", $time);
    $display("Fetch    : PC=%h, Instruction=%h", PC_F, instruction_F);
    $display("Decode   : Opcode=%h, rs1=%h, rs2=%h, rd=%h, reg_data_2_D = %h", opcode_D, rs1_D, rs2_D, rd_D,reg_data_2_D);
    $display("Execute  : ALU_out=%h, Operand1 = %h, Operand2 = %h, ALU_en = %b, Forward_A=%b, Forward_B=%b", alu_result_0_E, fwd_A, alu_operand_2,ALU_EN_E, forward_A, forward_B);
    $display("Writeback: WriteAddr=%h, WriteData=%h, WriteEn=%b", reg_write_addr_W, reg_write_data_0_W, read_write_W);
  end
  
  // Display final state
  $display("\n=== Final Processor State ===");
  $display("Register File:");
  $display("R0 = %h", reg_file_unit.registers[0]);
  $display("R1 = %h", reg_file_unit.registers[1]);
  $display("R2 = %h", reg_file_unit.registers[2]);
  $display("R3 = %h", reg_file_unit.registers[3]);
  $display("\nFlags = %b", current_flags_D);
  
  $finish;
end
endmodule