// my controller code


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
      $readmemh("riscvtest.txt",RAM);                   /////////////////////////////////////// test file here ///

  assign rd = RAM[a[31:2]]; // выравнивание на слово

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
						
  logic        Zero;
  logic [1:0]  ImmSrc, ALUSrcA, ALUSrcB, ResultSrc;
  logic        AdrSrc;
  logic [3:0]  ALUControl;
  logic        IRWrite, PCWrite, RegWrite;
  logic [31:0] Instr;
  
  controller c(clk, reset, 
					opcodetype'(Instr[6:0]), Instr[14:12], Instr[30], 
					Zero, 
					ImmSrc, ALUSrcA, ALUSrcB, ResultSrc, 
					AdrSrc,
					ALUControl,
					IRWrite, PCWrite, RegWrite,
					MemWrite);
					
	datapath dp(clk, reset,
	            PCWrite, AdrSrc, IRWrite, RegWrite,
					ImmSrc, ALUSrcA, ALUSrcB, ResultSrc,
					ALUControl,
					ReadData,
					Adr, Instr, WriteData,
					Zero);
  
  
endmodule


module datapath(input  logic        clk, reset,
                input  logic        PCWrite, AdrSrc, IRWrite, RegWrite,
                input  logic [1:0]  ImmSrc, ALUSrcA, ALUSrcB, ResultSrc,
					 input  logic [3:0]  ALUControl,
					 input  logic [31:0] ReadData,
					 output logic [31:0] Adr, Instr, WriteData,
					 output logic Zero);

  logic [31:0] Data, ImmExt, RD1, RD2, A;
  logic [31:0] PC, OldPC, SrcA, SrcB;
  logic [31:0] ALUResult, ALUOut, Result;
  
  // ReadData to Data/Instr flow
  flopr            #(32) ReadDataToData(clk, reset, ReadData, Data);
  conditionalflopr #(32) ReadDataToInstr(clk, reset, IRWrite, ReadData, Instr);
  
  //Extend data flow
  extend      ext(Instr[31:7], ImmSrc, ImmExt);
  
  //register file flow
  regfile     rf(clk, RegWrite, Instr[19:15], Instr[24:20], Instr[11:7], Result, RD1, RD2);
  flopr #(32) RD1ToA(clk, reset, RD1, A);
  flopr #(32) RD2ToWriteData(clk, reset, RD2, WriteData);
 
  //ALU flow
  mux3  #(32) OldPCPCAToSrcA(PC, OldPC, A, ALUSrcA, SrcA); 
  mux3  #(32) WriteDataImmExt4ToSrcB(WriteData, ImmExt, 32'd4, ALUSrcB, SrcB);
  alu         alu(SrcA, SrcB, ALUControl, ALUResult, Zero);
  flopr #(32) ALUResultToALUOut(clk, reset, ALUResult, ALUOut);
  
  //result flow
  mux3  #(32) ALUOutDataALUResultToResult(ALUOut, Data, ALUResult, ResultSrc, Result);
  
  //PC flow
  conditionalflopr #(32) ResultToPC(clk, reset, PCWrite, Result, PC);
  mux2             #(32) PCResultToAdr(PC, Result, AdrSrc, Adr);
  conditionalflopr #(32) PCToOldPC(clk, reset, IRWrite, PC, OldPC);
endmodule


// riscvsingle.sv

module regfile(input  logic        clk, 
               input  logic        we3, 
               input  logic [ 4:0] a1, a2, a3, 
               input  logic [31:0] wd3, 
               output logic [31:0] rd1, rd2);

  logic [31:0] rf[31:0];

  // three ported register file
  // read two ports combinationally (A1/RD1, A2/RD2)
  // write third port on rising edge of clock (A3/WD3/WE3)
  // register 0 hardwired to 0

  always_ff @(posedge clk)
    if (we3) rf[a3] <= wd3;	

  assign rd1 = (a1 != 0) ? rf[a1] : 0;
  assign rd2 = (a2 != 0) ? rf[a2] : 0;
endmodule



module extend(input  logic [31:7] instr,
              input  logic [1:0]  immsrc,
              output logic [31:0] immext);
 
  always_comb
    case(immsrc) 
               // I-type 
      2'b00:   immext = {{20{instr[31]}}, instr[31:20]};  
               // S-type (stores)
      2'b01:   immext = {{20{instr[31]}}, instr[31:25], instr[11:7]}; 
               // B-type (branches)
      2'b10:   immext = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0}; 
               // J-type (jal)
      2'b11:   immext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0}; 
      default: immext = 32'bx; // undefined
    endcase             
endmodule


module flopr #(parameter WIDTH = 8)
              (input  logic             clk, reset,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else       q <= d;
endmodule


module conditionalflopr #(parameter WIDTH = 8)
                         (input  logic   clk, reset, condition,
								  input  logic [WIDTH-1:0] d,
								  output logic [WIDTH-1:0] q);
								  
  always_ff @(posedge clk, posedge reset)
    if      (reset)     q <= 0;
    else if (condition) q <= d;
	 
endmodule							  


module mux2 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, 
              input  logic             s, 
              output logic [WIDTH-1:0] y);

  assign y = s ? d1 : d0; 
endmodule

module mux3 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, d2,
              input  logic [1:0]       s, 
              output logic [WIDTH-1:0] y);

  assign y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule



module alu(input  logic signed [31:0] a, b,     // now signed to work with sra srai correctly
           input  logic [3:0]  alucontrol,
           output logic signed [31:0] result,   // now signed to work with sra srai correctly
           output logic        zero);

  logic [31:0] condinvb, sum;
  logic        v;              // overflow
  logic        isAddSub;       // true when is add or subtract operation

  assign condinvb = alucontrol[0] ? ~b : b;
  assign sum = a + condinvb + alucontrol[0];
  assign isAddSub = ~alucontrol[2] & ~alucontrol[1] |
                    ~alucontrol[1] & alucontrol[0];

  always_comb
    case (alucontrol)
      4'b0000:  result = sum;          // add
      4'b0001:  result = sum;          // subtract
      4'b0010:  result = a & b;        // and
      4'b0011:  result = a | b;        // or
      4'b0100:  result = a ^ b;        // xor
      4'b0101:  result = sum[31] ^ v;  // slt
      4'b0110:  result = a << b[4:0];  // sll
      4'b0111:  result = a >> b[4:0];  // srl
      4'b1000:  result = a >>> b[4:0]; // sra, srai
      default: result = 32'bx;
    endcase

  assign zero = (result == 32'b0);
  assign v = ~(alucontrol[0] ^ a[31] ^ b[31]) & (a[31] ^ sum[31]) & isAddSub;
  
endmodule


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