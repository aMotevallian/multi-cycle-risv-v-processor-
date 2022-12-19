// controller.sv

// Place controller.tv in same computer directory as this file to test your multicycle controller.


typedef enum logic[6:0] {r_type_op=7'b0110011, i_type_alu_op=7'b0010011, lw_op=7'b0000011, sw_op=7'b0100011, beq_op=7'b1100011, jal_op=7'b1101111} opcodetype;



module controller(input logic clk,
            input logic reset,
            input logic [6:0] op,
            input logic [2:0] funct3,
            input logic funct7b5,
            input logic zero,
            output logic [1:0] immsrc,
            output logic [1:0] alusrca, alusrcb,
            output logic [1:0] resultsrc,
            output logic adrsrc,
            output logic [2:0] alucontrol,
            output logic irwrite, pcwrite,
            output logic regwrite, memwrite);

logic [1:0] aluop;
logic PCupdate;
logic branch ;
fsm mainfsm(clk ,reset ,op ,zero , alusrca, alusrcb, resultsrc, 
        adrsrc, aluop, irwrite, branch , PCupdate, regwrite, memwrite);

aludec alud(op[5] , funct3 ,funct7b5 , aluop , alucontrol);

instrDecoder instrd(op , immsrc);

assign pcwrite = branch && zero || PCupdate ;
endmodule 



module fsm(input logic clk ,
        input logic reset ,
        input logic [6:0] op , 
        input logic zero ,
        output logic [1:0] alusrca, alusrcb,
        output logic [1:0] resultsrc,
        output logic adrsrc,
        output logic [1:0] aluop,
        output logic irwrite, branch , PCupdate,
        output logic regwrite, memwrite);

typedef enum logic [3:0] {Fetch, Decode, Memadr, Memread, Memwb, Memwrite, Executer, Aluwb, Executei, Jal, Beq} statetype;
statetype state ,nextstate;
logic [13:0] control_signals ; 
always_ff@(posedge clk , posedge reset)
  if(reset)   state <= Fetch ; 
  else      state <= nextstate ; 

assign alusrca = control_signals[13:12];
assign alusrcb = control_signals[11:10];
assign resultsrc = control_signals[9:8];
assign adrsrc = control_signals[7];
assign aluop = control_signals[6:5];
assign irwrite = control_signals[4];
assign branch = control_signals[3];
assign PCupdate = control_signals[2];
assign regwrite = control_signals[1];
assign memwrite = control_signals[0]; 

always_comb begin
  
      case(state)
        Fetch : begin
          control_signals = 14'b00101000010100 ;
          nextstate = Decode ;
        end
      
        Decode : begin
          control_signals = 14'b01010000000000 ;
          if (op == 'b0000011 || op == 'b0100011) nextstate = Memadr;
          else if (op == 'b0110011) nextstate = Executer;
          else if (op == 'b0010011) nextstate = Executei;
          else if (op == 'b1101111) nextstate = Jal;
          else nextstate = Beq; //op == 'b1100011
        end
      
        Memadr : begin
          control_signals = 14'b10010000000000 ;
          if (op=='b0000011) nextstate = Memread ; 
          else nextstate = Memwrite ; //op == 'b0100011
        end
      
        Memread : begin
          control_signals = 14'b00000010000000 ;
          nextstate = Memwb ;
        end
      
        Memwb : begin
          control_signals = 14'b00000100000010 ;
          nextstate = Fetch ;
        end
      
        Memwrite : begin 
          control_signals = 14'b00000010000001 ;
          nextstate = Fetch ;
        end
      
        Executer : begin
          control_signals = 14'b10000001000000 ;
          nextstate = Aluwb ;
        end
      
        Aluwb : begin
          control_signals = 14'b00000000000010 ;
          nextstate = Fetch ;
        end
      
        Executei : begin
          control_signals = 14'b10010001000000 ;
          nextstate = Aluwb ;
        end
      
        Jal : begin
          control_signals = 14'b01100000000100 ;
          nextstate = Aluwb ; 
        end
      
        Beq : begin
          control_signals = 14'b10000000101000 ;
          nextstate = Fetch ;
        end
        default: nextstate = Fetch ;
      endcase
end   

endmodule





module aludec(input logic opb5,
    input logic [2:0] funct3,
    input logic funct7b5,
    input logic [1:0] ALUOp,
    output logic [2:0] ALUControl);
  logic RtypeSub;
  assign RtypeSub = funct7b5 && opb5; // TRUE for R–type subtract
  always_comb
    case(ALUOp)
      2'b00: ALUControl = 3'b000; // addition
      2'b01: ALUControl = 3'b001; // subtraction
      default: case(funct3) // R–type or I–type ALU
              3'b000: if (RtypeSub)
                      ALUControl = 3'b001; // sub
                    else
                      ALUControl = 3'b000; // add, addi
              3'b010: ALUControl = 3'b101; // slt, slti
              3'b110: ALUControl = 3'b011; // or, ori
              3'b111: ALUControl = 3'b010; // and, andi
              3'b001: ALUControl = 3'b111; // slli, sll
              3'b101:
                if (funct7b5)
                  ALUControl = 3'b110; // sra, srai
                else
                  ALUControl = 3'b100; // srl, srli
              default: ALUControl = 3'bxxx; // ???
            endcase
    endcase
endmodule


module instrDecoder(input logic [6:0] op, output logic [1:0] immsrc);
assign immsrc = (op==7'b0100011) ? 01:
           (op==7'b1100011) ? 10:
           (op==7'b1101111) ? 11:
           00;
endmodule 


module testbench();

  logic        clk;
  logic        reset;
  
  opcodetype  op;
  logic [2:0] funct3;
  logic       funct7b5;
  logic       Zero;
  logic [1:0] ImmSrc;
  logic [1:0] ALUSrcA, ALUSrcB;
  logic [1:0] ResultSrc;
  logic       AdrSrc;
  logic [2:0] ALUControl;
  logic       IRWrite, PCWrite;
  logic       RegWrite, MemWrite;
  
  logic [31:0] vectornum, errors;
  logic [39:0] testvectors[10000:0];
  
  logic        new_error;
  logic [15:0] expected;
  logic [6:0]  hash;


  // instantiate device to be tested
  controller dut(clk, reset, op, funct3, funct7b5, Zero,
                 ImmSrc, ALUSrcA, ALUSrcB, ResultSrc, AdrSrc, ALUControl, IRWrite, PCWrite, RegWrite, MemWrite);
  
  // generate clock
  always 
    begin
      clk = 1; #5; clk = 0; #5;
    end

  // at start of test, load vectors and pulse reset
  initial
    begin
      $readmemb("controller.tv", testvectors);
      vectornum = 0; errors = 0; hash = 0;
      reset = 1; #22; reset = 0;
    end
	 
  // apply test vectors on rising edge of clk
  always @(posedge clk)
    begin
      #1; {op, funct3, funct7b5, Zero, expected} = testvectors[vectornum];
    end

  // check results on falling edge of clk
  always @(negedge clk)
    if (~reset) begin // skip cycles during reset
      new_error=0; 

      if ((ImmSrc!==expected[15:14])&&(expected[15:14]!==2'bxx))  begin
        $display("   ImmSrc = %b      Expected %b", ImmSrc,     expected[15:14]);
        new_error=1;
      end
      if ((ALUSrcA!==expected[13:12])&&(expected[13:12]!==2'bxx)) begin
        $display("   ALUSrcA = %b     Expected %b", ALUSrcA,    expected[13:12]);
        new_error=1;
      end
      if ((ALUSrcB!==expected[11:10])&&(expected[11:10]!==2'bxx)) begin
        $display("   ALUSrcB = %b     Expected %b", ALUSrcB,    expected[11:10]);
        new_error=1;
      end
      if ((ResultSrc!==expected[9:8])&&(expected[9:8]!==2'bxx))   begin
        $display("   ResultSrc = %b   Expected %b", ResultSrc,  expected[9:8]);
        new_error=1;
      end
      if ((AdrSrc!==expected[7])&&(expected[7]!==1'bx))           begin
        $display("   AdrSrc = %b       Expected %b", AdrSrc,     expected[7]);
        new_error=1;
      end
      if ((ALUControl!==expected[6:4])&&(expected[6:4]!==3'bxxx)) begin
        $display("   ALUControl = %b Expected %b", ALUControl, expected[6:4]);
        new_error=1;
      end
      if ((IRWrite!==expected[3])&&(expected[3]!==1'bx))          begin
        $display("   IRWrite = %b      Expected %b", IRWrite,    expected[3]);
        new_error=1;
      end
      if ((PCWrite!==expected[2])&&(expected[2]!==1'bx))          begin
        $display("   PCWrite = %b      Expected %b", PCWrite,    expected[2]);
        new_error=1;
      end
      if ((RegWrite!==expected[1])&&(expected[1]!==1'bx))         begin
        $display("   RegWrite = %b     Expected %b", RegWrite,   expected[1]);
        new_error=1;
      end
      if ((MemWrite!==expected[0])&&(expected[0]!==1'bx))         begin
        $display("   MemWrite = %b     Expected %b", MemWrite,   expected[0]);
        new_error=1;
      end

      if (new_error) begin
        $display("Error on vector %d: inputs: op = %h funct3 = %h funct7b5 = %h", vectornum, op, funct3, funct7b5);
        errors = errors + 1;
      end
      vectornum = vectornum + 1;
      hash = hash ^ {ImmSrc&{2{expected[15:14]!==2'bxx}}, ALUSrcA&{2{expected[13:12]!==2'bxx}}} ^ {ALUSrcB&{2{expected[11:10]!==2'bxx}}, ResultSrc&{2{expected[9:8]!==2'bxx}}} ^ {AdrSrc&{expected[7]!==1'bx}, ALUControl&{3{expected[6:4]!==3'bxxx}}} ^ {IRWrite&{expected[3]!==1'bx}, PCWrite&{expected[2]!==1'bx}, RegWrite&{expected[1]!==1'bx}, MemWrite&{expected[0]!==1'bx}};
      hash = {hash[5:0], hash[6] ^ hash[5]};
      if (testvectors[vectornum] === 40'bx) begin 
        $display("%d tests completed with %d errors", vectornum, errors);
	      $display("hash = %h", hash);
        $stop;
      end
    end
endmodule

