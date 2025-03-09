parameter ADD = 5'b00000;
parameter MUL = 5'b00001;
parameter SUB = 5'b00010;
parameter DIV = 5'b00011;
parameter NOT = 5'b00100;
parameter AND = 5'b00101;
parameter OR = 5'b00110;
parameter XOR = 5'b00111;
parameter INC = 5'b01000;
parameter CMP = 5'b01001;
parameter RR = 5'b01010;
parameter RL = 5'b01011;
parameter SETB = 5'b01100;
parameter CLRB = 5'b01101;
parameter CPLB = 5'b01110;
parameter SETF = 5'b01111;
parameter CLRF = 5'b10000;
parameter CPLF = 5'b10001;
parameter LOADBR = 5'b10010;
parameter JF = 5'b10011;
parameter LOAD = 5'b10100;
parameter STORE = 5'b10101;
parameter LBL = 5'b10110;
parameter LBH = 5'b10111;
parameter MOV = 5'b11000;
parameter MOVOUT = 5'b11100;
parameter MOVIN = 5'b11101;
parameter MOVB = 5'b11110;
parameter HALT = 5'b11111;

parameter ADDR_WIDTH = 11;  // adress is 11 bits
parameter DATA_WIDTH = 16;  // data is 16 bits
parameter INSTRUCTION_WIDTH = 16;  // data is 16 bits
// 4Kb memory with 16 bit word size implies 2048 words (2^11)
parameter MEMORY_DEPTH = 2 ** ADDR_WIDTH;

parameter REG0 = 3'b000;
parameter REG1 = 3'b001;
parameter REG2 = 3'b010;
parameter REG3 = 3'b011;
parameter REG4 = 3'b100;
parameter REG5 = 3'b101;
parameter REG6 = 3'b110;
parameter REG7 = 3'b111;

