`ifndef _PARAMETERS_V_
`define _PARAMETERS_V_

`define ADD 5'b00000
`define MUL 5'b00001
`define SUB 5'b00010
`define DIV 5'b00011
`define NOT 5'b00100
`define AND 5'b00101
`define OR 5'b00110
`define XOR 5'b00111
`define INC 5'b01000
`define CMP 5'b01001
`define RR 5'b01010
`define RL 5'b01011
`define SETB 5'b01100
`define CLRB 5'b01101
`define CPLB 5'b01110
`define SETF 5'b01111
`define CLRF 5'b10000
`define CPLF 5'b10001
`define LOADBR 5'b10010
`define JF 5'b10011
`define LOAD 5'b10100
`define STORE 5'b10101
`define LBL 5'b10110
`define LBH 5'b10111
`define MOV 5'b11000
`define MOVOUT 5'b11100
`define MOVIN 5'b11101
`define MOVB 5'b11110
`define HALT 5'b11111
`define ADDR_WIDTH 11  // adress is 11 bits
`define DATA_WIDTH 16  // data is 16 bits
`define INSTRUCTION_WIDTH 16  // data is 16 bits
// define memory with 16 bit word size implies 2048 words (2^11)
`define MEMORY_DEPTH 2048
`define REG0 3'b000
`define REG1 3'b001
`define REG2 3'b010
`define REG3 3'b011
`define REG4 3'b100
`define REG5 3'b101
`define REG6 3'b110
`define REG7 3'b111

`endif