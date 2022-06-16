// controller.sv
//
// This file is for HMC E85A Lab 5.
// Place controller.tv in same computer directory as this file to test your multicycle controller.
//
// Starter code last updated by Ben Bracker (bbracker@hmc.edu) 1/14/21
// - added opcodetype enum
// - updated testbench and hash generator to accomodate don't cares as expected outputs
// Solution code by ________ (________) ________

typedef enum logic[6:0] {r_type_op=7'b0110011, i_type_alu_op=7'b0010011, lw_op=7'b0000011, sw_op=7'b0100011, beq_op=7'b1100011, jal_op=7'b1101111} opcodetype;



module controller(input  logic       clk,
                  input  logic       reset,  
                  input  opcodetype  op,
                  input  logic [2:0] funct3,
                  input  logic       funct7b5,
                  input  logic       Zero,
                  output logic [1:0] ImmSrc,
                  output logic [1:0] ALUSrcA, ALUSrcB,
                  output logic [1:0] ResultSrc, 
                  output logic       AdrSrc,
                  output logic [2:0] ALUControl,
                  output logic       IRWrite, PCWrite, 
                  output logic       RegWrite, MemWrite);

logic [1:0] ALUOp;
logic Branch, PCUpdate, temp;

aludecoder alu_decoder(ALUOp, funct3, op[5], funct7b5, ALUControl);

immsrc instr_decorder(op, ImmSrc);

mainfsm main_fsm(clk, reset, op, Branch, PCUpdate, IRWrite, RegWrite, MemWrite, ALUSrcA, ALUSrcB, ResultSrc, AdrSrc, ALUOp);

  always_comb
      case (op)
      7'b1100011:
        case (funct3)
         3'b001:  temp = (Branch & ~Zero);
         3'b000:  temp = (Branch & Zero);
         default: temp = 0;
        endcase
        default: temp = 0;
      endcase
  
  assign PCWrite = temp | PCUpdate;		
  
						
endmodule

module mainfsm(input  logic       clk,
               input  logic       reset,
               input  opcodetype  op,
	            output logic Branch,
	            output logic PCUpdate,
               output logic       IRWrite, 
               output logic       RegWrite, MemWrite,
               output logic [1:0] ALUSrcA, ALUSrcB,
	            output logic [1:0] ResultSrc, 
               output logic       AdrSrc,
	            output logic [1:0] ALUOp);

typedef enum logic[3:0] {FetchState, DecodeState, MemAdrState, MemReadState, 
				MemWBState, MemWriteState, ExecuteRState, ALUWBState, ExecutelState, JALState,  BEQState} statetype;

statetype state, nextstate;

  always @(posedge clk, posedge reset)
    begin
    	if (reset) 	state <= FetchState;
        else		state <= nextstate;
    end

  always_comb
    case (state)
    FetchState: nextstate = DecodeState;
 
    DecodeState: if (op == lw_op || op == sw_op) nextstate = MemAdrState;
        else if (op == r_type_op) nextstate = ExecuteRState;
        else if (op == i_type_alu_op) nextstate = ExecutelState;
        else if (op == jal_op) nextstate = JALState;
        else if (op == beq_op) nextstate = BEQState;
	     else  nextstate = DecodeState;
 
    MemAdrState: if (op == lw_op) nextstate = MemReadState;
        else if (op == sw_op) nextstate = MemWriteState;
        else nextstate = MemAdrState;

    MemReadState: nextstate = MemWBState;
 
    MemWBState: nextstate = FetchState;
 
    MemWriteState: nextstate = FetchState;
 
    ExecuteRState: nextstate = ALUWBState;
 
    ALUWBState: nextstate = FetchState;
 
    ExecutelState: nextstate = ALUWBState;
 
    JALState: nextstate = ALUWBState;
 
    BEQState: nextstate = FetchState;
 
    default: nextstate = FetchState;
 
    endcase

//output logic
  assign ALUSrcA[1] = (state == MemAdrState || state == ExecuteRState || state == ExecutelState || state == BEQState);
  assign ALUSrcA[0] = (state == DecodeState || state == JALState);
  assign ALUSrcB[1] = (state == FetchState || state == JALState);
  assign ALUSrcB[0] = (state == DecodeState || state == MemAdrState  || state == ExecutelState);
  
  assign ALUOp[1] = (state == ExecuteRState  || state == ExecutelState);
  assign ALUOp[0] = (state == BEQState);
  
  assign ResultSrc[1] = (state == FetchState);
  assign ResultSrc[0] = (state == MemWBState);
  
  assign Branch = (state == BEQState);
  assign AdrSrc = (state == MemReadState || state == MemWriteState);
  assign IRWrite = (state == FetchState);
  assign PCUpdate = (state == FetchState || state == JALState);
  assign RegWrite = (state == MemWBState || state == ALUWBState);
  assign MemWrite = (state == MemWriteState);

endmodule

//done
module immsrc(input opcodetype  op,
	      output logic [1:0] ImmSrc);
always_comb
    case (op)
    sw_op: ImmSrc = 2'b01;
    beq_op: ImmSrc = 2'b10;
    jal_op: ImmSrc = 2'b11;
    default: ImmSrc = 2'b00;
    endcase
	

endmodule 

//done
module aludecoder(input  logic [1:0] ALUOp,
              input  logic [2:0] funct3,
				  input  logic       opb5,
              input  logic       funct7b5, 
              output logic [3:0] ALUControl);

  logic  RtypeSub;
  assign RtypeSub = funct7b5 & opb5;  // TRUE for R-type subtract instruction

  always_comb
    case(ALUOp)
      2'b00:                ALUControl = 4'b0000; // addition
      2'b01:                ALUControl = 4'b0001; // subtraction
      default: case(funct3) // R-type or I-type ALU
                 3'b000:  if (RtypeSub) 
                            ALUControl = 4'b0001; // sub
                          else          
                            ALUControl = 4'b0000; // add, addi
                 3'b010:    ALUControl = 4'b0101; // slt, slti
					  3'b101:	 ALUControl = 4'b1000; // sra, srai
                 3'b110:    ALUControl = 4'b0011; // or, ori
                 3'b111:    ALUControl = 4'b0010; // and, andi
                 default:   ALUControl = 4'bxxxx; // ???
               endcase
    endcase
endmodule


//ENDHERE




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
        $display("   ImmSrc = %b      Expected %b"	, ImmSrc,     expected[15:14]);
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
