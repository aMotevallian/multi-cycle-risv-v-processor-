///////////////////////////////////////////////////////////////
// testbench
//
// Expect simulator to print "Simulation succeeded"
// when the value 25 (0x19) is written to address 100 (0x64)
///////////////////////////////////////////////////////////////

module testbench();

  logic        clk;
  logic        reset;

  logic [31:0] WriteData, DataAdr;
  logic        MemWrite;
  logic [31:0] hash;

  // instantiate device to be tested
  top dut(clk, reset, WriteData, DataAdr, MemWrite);
  
  // initialize test
  initial
    begin
      hash <= 0;
      reset <= 1; # 22; reset <= 0;
    end

  // generate clock to sequence tests
  always
    begin
      clk <= 1; # 5; clk <= 0; # 5;
    end

  // check results
  always @(negedge clk)
    begin
      if(MemWrite) begin
        if(DataAdr === 100 & WriteData === 25) begin
          $display("Simulation succeeded");
 	   	  $display("hash = %h", hash);
          $stop;
        end else if (DataAdr !== 96) begin
          $display("Simulation failed");
          $stop;
        end
      end
    end

  // Make 32-bit hash of instruction, PC, ALU
  always @(negedge clk)
    if (~reset) begin
      hash = hash ^ dut.rvmulti.dp.Instr ^ dut.rvmulti.dp.PC;
      if (MemWrite) hash = hash ^ WriteData;
      hash = {hash[30:0], hash[9] ^ hash[29] ^ hash[30] ^ hash[31]};
    end

endmodule

///////////////////////////////////////////////////////////////
// top
//
// Instantiates multicycle RISC-V processor and memory
///////////////////////////////////////////////////////////////

module top(input  logic        clk, reset, 
           output logic [31:0] WriteData, DataAdr, 
           output logic        MemWrite);

  logic [31:0] ReadData;
  
  // instantiate processor and memories
  riscvmulti rvmulti(clk, reset, MemWrite, DataAdr, 
                     WriteData, ReadData);
  mem mem(clk, MemWrite, DataAdr, WriteData, ReadData);
endmodule

///////////////////////////////////////////////////////////////
// mem
//
// Single-ported RAM with read and write ports
// Initialized with machine language program
///////////////////////////////////////////////////////////////

module mem(input  logic        clk, we,
           input  logic [31:0] a, wd,
           output logic [31:0] rd);

  logic [31:0] RAM[63:0];
  
  initial
      $readmemh("riscvtest.txt",RAM);

  assign rd = RAM[a[31:2]]; // word aligned

  always_ff @(posedge clk)
    if (we) RAM[a[31:2]] <= wd;
endmodule

///////////////////////////////////////////////////////////////
// riscvmulti
//
// Multicycle RISC-V microprocessor
///////////////////////////////////////////////////////////////

module riscvmulti(input  logic        clk, reset,
                  output logic        MemWrite,
                  output logic [31:0] Adr, WriteData,
                  input  logic [31:0] ReadData);

  logic [31:0] Instr;
  logic Zero;
  logic [1:0] ResultSrc;
  logic [2:0] ALUControl;
  logic [1:0] ALUSrcB;
  logic [1:0] ALUSrcA;
  logic [1:0] ImmSrc;
  logic RegWrite;
  logic PCWrite;
  logic AdrSrc;
  logic IRWrite;





  controller controller(clk, reset,  Instr[6:0], Instr[14:12], Instr[30], Zero, ImmSrc, ALUSrcA, ALUSrcB, ResultSrc, AdrSrc, ALUControl, IRWrite, PCWrite, RegWrite, MemWrite);


  datapath dp(reset, clk, ReadData, PCWrite, AdrSrc, IRWrite, ResultSrc, ALUControl, ALUSrcB, ALUSrcA, ImmSrc, RegWrite, Instr,  WriteData, Adr, Zero);



endmodule


///////////////////////////////////////////////////////////////
// Your modules go here
///////////////////////////////////////////////////////////////

// Describe your non-leaf cells structurally
// Describe your lef cells (mux, flop, alu, etc.) behaviorally
// Exactly follow the multicycle processor diagram
// Remember to declare internal signals
// Be consistent with spelling and capitalization
// Be consistent with order of signals in module declarations and instantiations


module datapath(input logic reset, input logic clk, input logic [31:0] ReadData, input logic PCWrite, input logic AdrSrc, input logic IRWrite, input logic [1:0] ResultSrc,input logic [2:0] ALUControl, input logic [1:0] ALUSrcB, input logic [1:0] ALUSrcA, input logic [1:0] ImmSrc, input logic RegWrite, output logic [31:0] Instr, output logic [31:0] WriteData, output logic [31:0] Adr, output logic Zero);
	logic [31:0] PC;
	logic [31:0] OldPC;
	logic [31:0] Data;
	logic [31:0] PCNext;
	logic [31:0] ALUResult;
	logic [31:0] ALUOut;
	logic [31:0] SrcA;
	logic [31:0] SrcB;
	logic [31:0] A;
	logic [31:0] ImmExt;
	logic [31:0] RD1;
	logic [31:0] RD2;
	logic [31:0] c4;

	assign c4 = 32'h4;



	flopenr #(32) f1(clk, reset, PCWrite, PCNext, PC);
	mux2 #(32) m1(PC, PCNext, AdrSrc, Adr);
	flopenr2 #(32) f2(clk, reset, IRWrite, PC, ReadData, OldPC, Instr);
	flopr #(32) f3(clk, reset, ReadData, Data);
	flopr2 #(32) f4(clk, reset, RD1, RD2, A, WriteData);
	mux3 #(32) m2(PC, OldPC, A, ALUSrcA, SrcA);
	mux3 #(32) m3(WriteData, ImmExt, c4, ALUSrcB, SrcB);
	flopr #(32) f5(clk, reset, ALUResult, ALUOut);
	mux3 #(32) m4(ALUOut, Data, ALUResult, ResultSrc, PCNext);


	regfile r1(clk, RegWrite, Instr[19:15], Instr[24:20], Instr[11:7], PCNext, RD1, RD2);

	extend e1(Instr[31:7], ImmSrc, ImmExt);

	alu alu(SrcA, SrcB, ALUControl, ALUResult, Zero);



endmodule



module mux3 #(parameter WIDTH = 8)
		(input logic [WIDTH-1:0] d0, d1, d2,
		input logic [1:0] s,
		output logic [WIDTH-1:0] y);
	assign y = s[1] ? d2 : (s[0] ? d1 : d0);
endmodule


module alu(input logic [31:0] SrcA,
		input logic [31:0] SrcB,
		input logic [2:0] ALUControl,
		output logic [31:0] ALUResult,
		output logic  Zero);
	
	always_comb
		begin
		case(ALUControl)
			3'b000:
				ALUResult = SrcA + SrcB;
			3'b001:
				ALUResult = SrcA - SrcB;
			3'b101: begin
				if(SrcA < SrcB)
					ALUResult = 1;
				else
					ALUResult = 0;
				end
			3'b011:
				ALUResult = SrcA | SrcB;
			3'b010:
				ALUResult = SrcA & SrcB;
			3'b111: // slli and sll
				ALUResult = SrcA << SrcB;
			3'b100: // srli and srl
				ALUResult = SrcA >> SrcB;
			3'b110:
				ALUResult = SrcA >>> SrcB;
			default: ALUResult = SrcA + SrcB;
		
		
		endcase
		if (ALUResult == 0)
			Zero = 1;
		else
			Zero = 0;
		end
		
endmodule



module extend(input logic [31:7] instr,
		input logic [1:0] immsrc,
		output logic [31:0] immext);
	always_comb
	begin
		case(immsrc)
			// I?type
			2'b00: immext = {{20{instr[31]}}, instr[31:20]};
			// S?type (stores)
			2'b01: immext = {{20{instr[31]}}, instr[31:25],instr[11:7]};
			// B?type (branches)
			2'b10: immext = {{20{instr[31]}}, instr[7],instr[30:25], instr[11:8], 1'b0};
			// J?type (jal)
			2'b11: immext = {{12{instr[31]}}, instr[19:12],instr[20], instr[30:21], 1'b0};
			default: immext = 32'bx; // undefined
		endcase
	end
endmodule


module mux2 #(parameter WIDTH = 8)
		(input logic [WIDTH-1:0] d0, d1,
		input logic s,
		output logic [WIDTH-1:0] y);
	assign y = s ? d1 : d0;
endmodule



module regfile(input logic clk,
		input logic RegWrite,
		input logic [4:0] A1,
		input logic [4:0] A2,
		input logic [4:0] A3,
		input logic [31:0] WD,
		output logic [31:0] RD1,
		output logic [31:0] RD2);

		




	reg [31:0] reg_file [0:31];
    	integer i;
	initial begin
	for(i=0;i<32;i=i+1)
	begin
        	reg_file[i]<=32'h0;
        end
	end
    	always @(negedge clk ) begin
        	if(RegWrite && (A3!=0))begin 
            	reg_file[A3] <= WD;
        	end
    	end
    	assign RD1 = reg_file[A1];
	assign RD2 = reg_file[A2];
		
endmodule


module flopr #(parameter WIDTH = 8)
		(input logic clk, reset,
		input logic [WIDTH-1:0] d,
		output logic [WIDTH-1:0] q);
	always_ff @(posedge clk, posedge reset)
		if (reset) q <= 32'b0;
		else q <= d;
endmodule


module flopenr #(parameter WIDTH = 8)
		(input logic clk, reset, en,
		input logic [WIDTH-1:0] d,
		output logic [WIDTH-1:0] q);
	always_ff @(posedge clk, posedge reset)
		if (reset) q <= 32'b0;
		else if (en) q <= d;
endmodule


module flopr2 #(parameter WIDTH = 8)
		(input logic clk, reset,
		input logic [WIDTH-1:0] d1,
		input logic [WIDTH-1:0] d2,
		output logic [WIDTH-1:0] q1,
		output logic [WIDTH-1:0] q2);
	always_ff @(posedge clk, posedge reset)
		if (reset) begin q1 <= 32'b0; q2 <= 32'b0; end 
		else begin q1 <= d1; q2 <= d2; end
endmodule

module flopenr2 #(parameter WIDTH = 8)
		(input logic clk, reset, en,
		input logic [WIDTH-1:0] d1,
		input logic [WIDTH-1:0] d2,
		output logic [WIDTH-1:0] q1,
		output logic [WIDTH-1:0] q2);
	always_ff @(posedge clk, posedge reset)
		if (reset) begin q1 <= 32'b0; q2 <= 32'b0; end 
		else if (en) begin q1 <= d1; q2 <= d2; end
endmodule

