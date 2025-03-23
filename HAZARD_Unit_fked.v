`include "parameters.v"
module HAZARD_Unit_old (
    input wire [4:0] opcode_D,
    input wire [4:0] opcode_E,
    input wire [2:0] rs1_D,
    input wire [2:0] rs2_D,
    input wire [2:0] rd_E,
    input wire [2:0] rd_W,
    input wire mem_read_E,
    input wire branch_D,
    input wire jump_D,
    input wire reg_write_E,
    input wire reg_write_W,
    output reg stall_F,
    output reg stall_D,
    output reg flush_F,
    output reg flush_D,
    output reg [1:0] forward_A,
    output reg [1:0] forward_B
);

  // Detect forwarding needs for ALU instructions
  wire is_alu_op_D;
  assign is_alu_op_D = (opcode_D == `INC || opcode_D == `ADD || opcode_D == `SUB || 
                       opcode_D == `AND || opcode_D == `OR || opcode_D == `XOR);

 // In the HAZARD_Unit module:
always @(*) begin
  // Default values
  forward_A = 2'b00;
  forward_B = 2'b00;
  stall_F = 1'b0;
  stall_D = 1'b0;
  flush_F = 1'b0;
  flush_D = 1'b0;

  // Load-use hazard detection - must check this first
  if (mem_read_E && ((rs1_D == rd_E) || (rs2_D == rd_E)) && (rd_E != 0)) begin
    stall_F = 1'b1;
    stall_D = 1'b1;
    // Don't do forwarding when stalling
  end
  // Only do forwarding if not stalling
  else begin
    // Forwarding from execute stage
    if (reg_write_E && (rd_E != 0)) begin
      if (rs1_D == rd_E) begin
        forward_A = 2'b01;
      end
      if (rs2_D == rd_E) begin
        forward_B = 2'b01;
      end
    end
    
    // Forwarding from writeback stage
    if (reg_write_W && (rd_W != 0)) begin
      // Only forward from WB if EX doesn't provide the value
      if ((rs1_D == rd_W) && !(reg_write_E && (rs1_D == rd_E))) begin
        forward_A = 2'b10;
      end
      if ((rs2_D == rd_W) && !(reg_write_E && (rs2_D == rd_E))) begin
        forward_B = 2'b10;
      end
    end
  end

  // Branch/Jump handling - modify logic to avoid conflicts with stalls
  if ((branch_D || jump_D) && !stall_D) begin
    flush_F = 1'b1;
  end

endmodule