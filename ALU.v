module ALU (
    // input wire alu_en,
    input wire [4:0] opcode,  // from instruction register
    input wire [15:0] operand_1,  // reg_arg_2 in control unit
    input wire [15:0] operand_2,  // reg_arg_3 in control unit
    input wire [3:0] bit_position,  // set the bit in the postion given by this input
    input wire [15:0] current_flags,
    output reg [15:0] result_0,  // to the data_in_0
    output reg [15:0] result_1,  // to the data_in_1
    output reg [15:0] next_flags
);

  // The Bits of the flag Register
  /*
next_flags[0] : Carry Flag (C)
next_flags[1] : Overflow Flag (V)
next_flags[2] : Compare Flag (CMP)
next_flags[3] : Equal Flag (Eq)
next_flags[4] : IO Flag (IO)
next_flags[5] : Parity Flag (P) - Set when number of ones are even
next_flags[6] : Negative Flag (N)
next_flags[7] : Zero  Flag (Z)
*/
  `include "parameters.v"
  reg [31:0] mul_temp;
  reg [15:0] div_temp;
  reg [16:0] add_temp;

  // Function to calculate parity
  function automatic parity;
    input [15:0] value;
    begin
      parity = ~^value;  // XNOR reduction - 1 for even parity
    end
  endfunction

  // Common flag setting function
  task set_common_flags;
    input [15:0] value;
    begin

      next_flags[7] = (value == 16'b0);
      //Note: 6 and 5 used to be = not <=
      next_flags[6] = value[15];
      next_flags[5] = parity(value);
    end
  endtask

  always @* begin
    // Default flag values
    next_flags = current_flags;
    case (opcode)
      ADD: begin
        add_temp = {1'b0, operand_1} + {1'b0, operand_2};
        result_0 = add_temp[15:0];

        // Flag settings
        next_flags[0] = add_temp[16];
        next_flags[1] = (operand_1[15] == operand_2[15]) && (result_0[15] != operand_1[15]);
        set_common_flags(result_0);
      end
      MUL: begin
        mul_temp = operand_1 * operand_2;
        result_0 = mul_temp[15:0];
        // TODO: the write_en should change to 11
        result_1 = mul_temp[31:16];
        next_flags[0] = 0;
      end
      SUB: begin
        result_0 = operand_1 - operand_2;

        // Flag settings
        next_flags[0] = (operand_1 < operand_2);
        next_flags[1] = (operand_1[15] != operand_2[15]) && (result_0[15] == operand_2[15]);
        set_common_flags(result_0);
      end
      DIV: begin
        if (operand_2 != 0) begin
          result_0 = operand_1 / operand_2;
          // TODO: the write_en should change to 11
          result_1 = operand_1 % operand_2;

          // Flag settings
          set_common_flags(result_0);
        end else begin
          result_0 = 16'hFFFF;
          next_flags[1] = 1'b1;
        end
      end

      NOT: begin
        result_0 = ~operand_1;
        set_common_flags(result_0);
      end

      AND: begin
        result_0 = operand_1 & operand_2;
        set_common_flags(result_0);
      end

      OR: begin
        result_0 = operand_1 | operand_2;
        set_common_flags(result_0);
      end

      XOR: begin
        result_0 = operand_1 ^ operand_2;
        set_common_flags(result_0);
      end

      INC: begin
        add_temp = {1'b0, operand_1} + 16'b1;
        result_0 = add_temp[15:0];
        // Flag settings
        next_flags[0] = add_temp[16];
        next_flags[1] = (operand_1[15] == 1'b0) && (result_0[15] == 1'b1);
        set_common_flags(result_0);
      end

      CMP: begin
        result_0 = operand_1 - operand_2;
        // Flag settings
        next_flags[2] = (operand_1 > operand_2);
        next_flags[3] = (operand_1 == operand_2);
        next_flags[0] = (operand_1 < operand_2);
        set_common_flags(result_0);
      end

      RR: begin
        result_0 = {operand_1[0], operand_1[15:1]};
        set_common_flags(result_0);
      end

      RL: begin
        result_0 = {operand_1[14:0], operand_1[15]};
        set_common_flags(result_0);
      end

      SETB: begin
        result_0 = operand_1 | (16'b1 << bit_position);
        set_common_flags(result_0);
      end

      CLRB: begin
        result_0 = operand_1 & ~(16'b1 << bit_position);
        set_common_flags(result_0);
      end

      CPLB: begin
        result_0 = operand_1 ^ (16'b1 << bit_position);
        set_common_flags(result_0);
      end
        
      SETF: begin
        next_flags[bit_position] = 1'b1;
      end

      CLRF: begin
        next_flags[bit_position] = 1'b0;
      end

      CPLF: begin 
        next_flags[bit_position] = ~next_flags[bit_position];
      end
      default: $display("No operation");
    endcase
  end
endmodule

module Flag_Register(
  input wire clk,
  input wire reset,
  input wire [15:0] next_flags,
  output reg [15:0] current_flags
);
  always @(posedge clk or posedge reset) begin
    if (reset) begin
      current_flags <= 16'b0;
    end else begin
      current_flags <= next_flags;
    end
  end
endmodule
