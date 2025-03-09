module control_unit (
    input wire clk,
    input wire [4:0] opcode,  // from instruction register
    output wire alu_output
);
  `include "parameters.v"

  assign alu_output = opcode <= CPLB;

  
endmodule

