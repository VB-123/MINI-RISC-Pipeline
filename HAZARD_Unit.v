`include "parameters.v"
`timescale 1ps/1ps
module HAZARD_Unit(
  // Inputs
  input wire clk,                         // Clock signal
  input wire alu_en_in,                      // From Control Unit
  input wire [4:0] opcode_D,              // From Decode stage
  input wire [4:0] opcode_E,              // From Execute stage
  input wire [4:0] opcode_W,
  input wire [2:0] rd_D,                  // From Decode stage
  input wire [2:0] source_reg1_D,         // From Decode stage
  input wire [2:0] source_reg2_D,         // From Decode stage
  input wire [2:0] rd_E,                  // From Execute stage
  input wire [2:0] source_reg1_E,         // From Execute stage
  input wire [2:0] source_reg2_E,         // From Execute stage
  input wire [2:0] rd_W,                  // From Writeback stage
  input wire [2:0] source_reg1_W,         // From Writeback stage
  input wire [2:0] source_reg2_W,         // From Writeback stage
  input wire mem_read_E,                  // From Control Unit via DE Register
  input wire branch_en,                      // From Control Unit
  input wire jump_hzd,                    // Connect to control_unit
  input wire reg_write_D, 
  input wire reg_write_E,                 // From Control Unit via DE Register
  input wire reg_write_W,                 // From Control Unit via EW Register
  
  // Outputs
  output reg stall_F,                     // To FD Register
  output reg stall_D,                     // To DE Register
  output reg stall_E,                     // To EW Register
  output reg flush_F,                     // To FD Register
  output reg flush_D,                     // To DE Register
  output reg flush_E,                     // To EW Register
  output reg [1:0] forward_A,             // To forwarding logic
  output reg [1:0] forward_B,              // To forwarding logic
  output reg forward_decode_A,  // Forward to Decode stage for source_reg1
  output reg forward_decode_B,  // Forward to Decode stage for source_reg2
  output reg alu_en_out
);
  // Forwarding logic
  always @(*) begin
    // Default: no forwarding
    forward_A = 2'b00;
    forward_B = 2'b00;
    forward_decode_A = 1'b0;
    forward_decode_B = 1'b0;
    // Forwarding logic
    if (reg_write_W) begin
      if (rd_W == source_reg1_E) begin
        forward_A = 2'b10;
      end
      
      if (rd_W == source_reg2_E) begin
        forward_B = 2'b10;
      end

      if (rd_W == source_reg1_D) begin
        forward_decode_A = 1'b1; // Forward from writeback to decode
      end
      
      if (rd_W == source_reg2_D) begin
        forward_decode_B = 1'b1; // Forward from writeback to decode
      end

      if (reg_write_W && (opcode_D == `LOAD) && (rd_W == source_reg1_D)) begin
        forward_decode_A = 1'b1;  // Forward from WB to Decode
      end
    end
  end

  // RAW hazard detection
  wire raw_hazard;
  wire lbh_lbl_hazard;              
  assign lbh_lbl_hazard = (
    ((opcode_E == `LBH || opcode_E == `LBL)) && 
    (reg_write_E) && 
    ((rd_E == source_reg1_D) || (rd_E == source_reg2_D))
  );
  assign raw_hazard = reg_write_E && reg_write_W &&
                  ((rd_E == source_reg1_D) || (rd_E == source_reg2_D));
  // Control hazard detection                       
  wire control_hazard;
  assign control_hazard = branch_en && ((opcode_W == `SETF) || (opcode_W == `CPLF));
  
  // Pipeline control logic
  always @(*) begin
    // Default values
    stall_F = 1'b0;
    stall_D = 1'b0;
    stall_E = 1'b0;
    flush_F = 1'b0;
    flush_D = 1'b0;
    flush_E = 1'b0;
    alu_en_out = 1'b0; 
    if (branch_en) begin
        flush_F = 1'b1;
        flush_D = 1'b1;
        stall_F = 1'b1;  
        stall_D = 1'b1; 
        stall_E = 1'b1; 
        alu_en_out = 1'b0;
    end
  end
endmodule