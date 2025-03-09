// Register testbench
module registers_tb();
    reg clk;
    reg rst;
    reg [1:0] write_en;
    reg [2:0] read_addr_0;
    reg [2:0] read_addr_1;
    reg [2:0] reg_write_addr_0;
    reg [2:0] reg_write_addr_1;
    reg [15:0] data_in_0;
    reg [15:0] data_in_1;
    wire [15:0] read_data_0;
    wire [15:0] read_data_1;
    
    reg_file uut (
      .clk(clk),
      .rst(rst),
      .write_en(write_en),
      .read_addr_0(read_addr_0),
      .read_addr_1(read_addr_1),
      .reg_write_addr_0(reg_write_addr_0),
      .reg_write_addr_1(reg_write_addr_1),
      .data_in_0(data_in_0),
      .data_in_1(data_in_1),
      .read_data_0(read_data_0),
      .read_data_1(read_data_1)
    );
    
    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end
    
     initial begin
        $dumpfile("registers_tb.vcd");
        $dumpvars(0, registers_tb);    
    end
    initial begin
        rst = 1;
        write_en = 0;
        read_addr_0 = 0;
        read_addr_1 = 0;
        reg_write_addr_0 = 0;
        data_in_0 = 0;
        
        // 100 ns for global reset
        #100;
        rst = 0;
        
        // Test case 1: Write to register
        @(posedge clk);
        write_en = 2'b01;
        reg_write_addr_0 = 3'b010;
        data_in_0 = 16'h1234;
        
        // Test case 2: Write to register
        @(posedge clk);
        reg_write_addr_0 = 3'b101;
        data_in_0 = 16'h5678;
        
        // Test case 3: Read
        @(posedge clk);
        write_en = 2'b00;
        read_addr_0 = 3'b010;
        read_addr_1 = 3'b101;
	#10;
	
	// Test case 5: Simultaneous read and write
        @(posedge clk);
        write_en = 2'b01;
        reg_write_addr_0 = 3'b010;  
        data_in_0 = 16'h2345;
        read_addr_0 = 3'b010; 
        read_addr_1 = 3'b000;
	
	// Test case 5a: Read
	@(posedge clk);
        write_en = 2'b00;
        read_addr_0 = 3'b010;
        read_addr_1 = 3'b101;
	#10;
	
	// Test case 6: Write_en is 11
	@(posedge clk);
	write_en = 2'b11;
	reg_write_addr_1 = 3'b011;
	data_in_1 = 16'h1234;
	#10;

	// Test case 6a: Read
	@(posedge clk);
        write_en = 2'b00;
        read_addr_0 = 3'b011;
	#10;
	
	// Test case 7: Write using port 2, when write_en = 01
	@(posedge clk);
	write_en = 2'b01;
	reg_write_addr_1 = 3'b011;
	data_in_1 = 16'h1010;
	#10;
	@(posedge clk);
        write_en = 2'b00;
        read_addr_0 = 3'b011;
	#10;
	
	// Test case X: Reset
        @(posedge clk);
        rst = 1;
	write_en = 2'b00;
        #10;
        rst = 0;

        // End simulation
        #50;
        $finish;
    end
    initial begin
        $monitor("Time=%0t rst=%b we=%b wa0=%b din0=%h wa1=%b din1=%h ra0=%b rd0=%h ra1=%b rd1=%h",
                 $time, rst, write_en, reg_write_addr_0, data_in_0, reg_write_addr_1, data_in_1,
                 read_addr_0, read_data_0, read_addr_1, read_data_1);
    end
endmodule
