`timescale 1ns/1ns

module DataMem(clock,MemWrite,address,WriteData,ReadData);

input clock, MemWrite;
input [31:0] address;
input [31:0] WriteData;

output [31:0] ReadData;

reg [31:0] Dmem [127:0];
    
initial begin
		Dmem[0] = 0;
		Dmem[1] = 1;
		Dmem[2] = 2;
		Dmem[3] = 3;
		Dmem[4] = 4;
		Dmem[5] = 5;
		Dmem[6] = 6;
		Dmem[7] = 7;
		Dmem[8] = 8;
		Dmem[9] = 9;
		Dmem[10] = 10;
		Dmem[11] = 11;
    end
    assign ReadData = Dmem[address];
    
always @ (posedge clock) begin
	
    if (MemWrite) 
		Dmem[address] <= WriteData;
end
endmodule
//////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ns
module InstMem(ReadAddress,Instruction);

input [31:0] ReadAddress;
output [31:0] Instruction;

reg [31:0] Imem[255:0];
    
initial begin
    //Imem[0] = 32'b10001100000000010000000000000101; //LW $1,5($0);
    //Imem[1] = 32'b10001100000000100000000000000100; //LW $2,4($0);
    //Imem[2] = 32'b00000000001000100001100000100000; //add $3,$1,$2;
    //Imem[3] = 32'b00000000001000100010000000100010; //sub $4,$1,$2
    //Imem[4] = 32'b10101100000000110000000000000000; //SW $3,0($0);
    //Imem[5] = 32'b10101100000001000000000000000000; //SW $4,0($0);
    //Imem[6] = 32'b00000001001100000111000000100101; //or $5,$4,$3;
    //Imem[7] = 32'b10101100000001010000000000000000; //SW $5,0($0);
    //Imem[8] = 32'b00000000001000100011000000100100; //and $6,$1,$2;
    //Imem[9] = 32'b10101100000001100000000000000000; //SW $6,0($0);
    //Imem[10] = 32'b0001000001100100000000000000000; //beq $3,$4,0
    
    
    //Imem[0] = 32'b10001100000000010000000000000001; //LW $1,1($0);
    //Imem[1] = 32'b10001100000000100000000000000011; //LW $2,3($0);
    //Imem[2] = 32'b00000000000000000000000000000000; //nop
    //Imem[2] = 32'b00000000001000100001100000100000; //add $3,$1,$2;
    //Imem[3] = 32'b00000000001000100010000000100000; //sub $4,$3,$2;
    //Imem[4] = 32'b10101100000000110000000000000100; //SW $3,4($0); 
    //Imem[5] = 32'b10101100000001000000000000000001; //SW $4,1($0);
    //Imem[6] = 32'b00000000011001000010100000100100; //and $5,$3,$4;
    //Imem[7] = 32'b000000 00011 00100 00110 00000100101; //or $6,$3,$4;
    //Imem[8] = 32'b10001100000001110000000000000100; //LW $7 , 4($0);
    //Imem[9] = 32'b00000000111001100100000000100000; //Add $8,$7,$6;
    

    Imem[0] = 32'b10001100000000010000000000000001; //LW $1,1($0);
    Imem[1] = 32'b10001100000000100000000000000011; //LW $2,3($0);
    Imem[2] = 32'b00000000000000000000000000000000; //nop
    Imem[2] = 32'b00000000001000100001100000100000; //add $3,$1,$2;
    Imem[3] = 32'b00000000001000100001100000100010; //sub $4,$3,$2;
    Imem[4] = 32'b10101100000000110000000000000100; //SW $3,4($0); 
    Imem[5] = 32'b10101100000001000000000000000001; //SW $4,1($0);
    Imem[6] = 32'b00000000011001000010100000100100; //and $5,$3,$4;
    Imem[7] = 32'b00000000011001000011000000100101; //or $6,$3,$4;
    Imem[8] = 32'b10001100000001110000000000000111; //LW $7 , 7($0);
    Imem[9] = 32'b00000000111001100100000000100000; //Add $8,$7,$6;
    Imem[10] = 32'b00010000001000100000000000000110; //beq $1,$2,6
    Imem[11] = 32'b00010000001001000000000000000000; //beq $1,$4,0
    
end
    
  assign Instruction= Imem[ReadAddress/4]; 

endmodule
/////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ns

module ShiftLeft(input [31:0] in,output [31:0] out);
            
assign out = in << 2;

endmodule
/////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ns
module signExtend( input  [15:0] in,output [31:0] out );

    assign out = { {16{in[15]}} , in };
    
endmodule
////////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ns
module PC (clk, rst,en ,in,out);

input clk,rst;
input [31:0] in;
input en;
output reg [31:0] out;

always @ (posedge clk)

  if(rst) out<=0;
  else if (!en) out<=out;
  else out<=in;
    
endmodule
/////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ns
module ALU_Adder (a,b,y);

input [31:0] a,b;
wire [32:0] w;
output [31:0] y;
assign w[0]=0;
genvar i;
generate 
for(i=0;i<=31;i=i+1) begin:adding
        FullAdder FA(a[i],b[i],w[i],w[i+1],y[i]);
    end
endgenerate
endmodule

module FullAdder(input a, input b, input cin, output cout, output s);

assign {cout,s}=a+b+cin;

endmodule
////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ns

module ALU32bits( input [31:0] a, 
            input [31:0] b,
            input [3:0] f,
            output reg [31:0] y, 
            output reg zero);

always @ (*) begin

    case (f)
        4'b0000: y = a + b;   // ADD
        4'b0001: y = a - b;   // SUB
        4'b0010: y = a & b;   // AND
        4'b0011: y = a | b;   // OR
    endcase
         zero = (y==8'b0);
     end
endmodule
//////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ns
module Controlunit(input [5:0] Opcode, 
               input [5:0] Func,
               output reg MemtoReg,
               output reg  MemWrite,
               output reg  ALUSrc,
               output reg  RegDst,
               output reg  RegWrite,
               output reg  Jump,
               output reg Branch,
               output reg B,
               output reg  [3:0] ALUControl
               );
               
reg [7:0] temp;

always @(*) begin 

    case (Opcode) 
        6'b000000: begin                          // R-type
                    temp <= 8'b11000000;        

                    case (Func)
                    6'b100000: ALUControl <= 4'b0000;    // ADD
                    6'b100010: ALUControl <= 4'b0001;    // SUB
                    6'b100100: ALUControl <= 4'b0010;    // AND
                    6'b100101: ALUControl <= 4'b0011;    // OR
                endcase

            end

        6'b100011: begin                          // LW
                        temp <= 8'b10100100;     
                        ALUControl <= 4'b0000;
                    end

        6'b101011: begin                          // SW
                         temp <= 8'b00101000;      
                         ALUControl <= 4'b0000;
                    end  

        6'b000100: begin                          // BEQ
                         temp <= 8'b00010000;      
                        ALUControl <= 4'b0001; 
                end  
                                      
                             
        default:   temp <= 12'bxxxxxxxxxxxx;      // NOP
    endcase
    
    {RegWrite,RegDst,ALUSrc,Branch,MemWrite,MemtoReg,Jump,B} = temp;
end 
endmodule
////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ns
module RegisterFile32bits (input clock,
                    input RegWrite, 
                    input reset,
                    input [4:0] ReadRegister1,
                    input [4:0] ReadRegister2,
                    input [4:0] WriteRegister,
                    input [31:0] WriteData,
                    output [31:0] ReadData1, 
                    output [31:0] ReadData2); 
                    
                     
reg [31:0] register [31:0];

assign ReadData1 = register[ReadRegister1];
assign ReadData2 = register[ReadRegister2];

integer i;

initial begin
    for (i=1; i<32; i=i+1) begin
         register[i] <= 32'd0;
        end
    end
    
    initial 
       begin
        register[0] <= 0;
        register[1] <= 13;
        register[2] <= 14;
        register[3] <= 15;
        register[4] <= 16;
        register[5] <= 19;
        register[6] <= 21;
        register[7] <= 25;
        register[8] <= 29;
       end
    
always @(posedge clock)
begin
    register[0]=0;
    if(reset) for(i = 0; i < 32; i = i + 1) register[i] = 32'd0;
    else if (RegWrite)
        if(WriteRegister != 0) register[WriteRegister]= WriteData;
    
end

endmodule
///////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ns

module IF_ID(input clk, 
             input rst,
             input stall,   
             input [31:0]PCplus4_IF, 
             output reg [31:0]PCplus4_ID, 
             input [31:0]Instr_IF,
             output reg [31:0]Instr_ID);
  
  always@(posedge clk)
    begin
    
      if (rst) begin
        PCplus4_ID <= 0;
        Instr_ID <= 0;
      end

      else if(stall) begin
        PCplus4_ID <= PCplus4_ID;
        Instr_ID <= Instr_ID;
        end

      else begin
		PCplus4_ID <= PCplus4_IF;
        Instr_ID <= Instr_IF;
      end

    end
  
endmodule

//////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ns

module ID_EX(input clk, 
             input rst,
             input[31:0] dataone_ID,
             output reg [31:0] dataone_Ex,
             input[31:0] WriteData_ID,
             output reg [31:0] WriteData_Ex,
             input[31:0] extendedimm_ID,
             output reg [31:0] extendedimm_Ex,
             input [31:0] Instr_ID,
             output reg [31:0] Instr_Ex,
             input RegWrite_ID,
             output reg RegWrite_Ex,
             input MemtoReg_ID, 
             output reg MemtoReg_Ex,
             input MemWrite_ID, 
             output reg MemWrite_Ex, 
             input [3:0]ALUControl_ID,
             output reg [3:0]ALUControl_Ex,
             input ALUSrc_ID,
             output reg ALUSrc_Ex,  
             input RegDst_ID,
             output reg RegDst_Ex);
  
  always@(posedge clk)
    begin
      if (rst) begin

        dataone_Ex <= 0;
        WriteData_Ex <= 0;
        extendedimm_Ex <= 0;
        Instr_Ex <= 0;
        RegWrite_Ex <= 0;
        MemtoReg_Ex <= 0;
        MemWrite_Ex <= 0;
        ALUControl_Ex <= 0;
        ALUSrc_Ex <= 0;
        RegDst_Ex <= 0;
		 
      end

    else   begin

        dataone_Ex <= dataone_ID;
        WriteData_Ex <= WriteData_ID;
        extendedimm_Ex <= extendedimm_ID;
        Instr_Ex <= Instr_ID;
        RegWrite_Ex <= RegWrite_ID;
        MemtoReg_Ex <= MemtoReg_ID;
        MemWrite_Ex <= MemWrite_ID;
        ALUControl_Ex <= ALUControl_ID;
        ALUSrc_Ex <= ALUSrc_ID;
        RegDst_Ex <= RegDst_ID;

      end
    end
  
endmodule

//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ns

module EX_M(input clk,
            input rst,
            input [31:0] ALUResult_Ex,
            output reg [31:0] ALUResult_M,
            input [31:0] WriteData_Ex,
            output reg [31:0] WriteData_M,
            input [4:0] writereg_Ex,
            output reg [4:0] writereg_M,
            input RegWrite_Ex,
            output reg RegWrite_M,
            input MemtoReg_Ex, 
            output reg MemtoReg_M,
            input MemWrite_Ex, 
            output reg MemWrite_M);
  
  always@(posedge clk)
    begin
      if (rst)  begin
        ALUResult_M <= 0;
        WriteData_M <= 0;
        writereg_M <= 0;
        RegWrite_M <= 0;
        MemtoReg_M <= 0;
        MemWrite_M <= 0;
        end

      else begin
        ALUResult_M <= ALUResult_Ex;
        WriteData_M <= WriteData_Ex;
        writereg_M  <= writereg_Ex;
        RegWrite_M  <= RegWrite_Ex;
        MemtoReg_M  <= MemtoReg_Ex;
        MemWrite_M  <= MemWrite_Ex;
      end
    end
  
endmodule

////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ns

module M_WB(input clk,
            input rst,
            input [31:0] ReadData_M,
            output reg [31:0] ReadData_WB,
            input [31:0] ALUResult_M, 
            output reg [31:0]ALUResult_WB,
            input [4:0]writereg_M,
            output reg [4:0]writereg_WB,
            input RegWrite_M,
            output reg RegWrite_WB,
            input MemtoReg_M, 
            output reg MemtoReg_WB);
  
  always@(posedge clk )
    begin
      if (rst) begin

		ReadData_WB <= 0;
        ALUResult_WB <= 0;
        writereg_WB <= 0;

        RegWrite_WB <= 0;
        MemtoReg_WB <= 0;
      end
      
      else begin

		ReadData_WB <= ReadData_M;
        ALUResult_WB <= ALUResult_M;
        writereg_WB <= writereg_M;

        RegWrite_WB  <= RegWrite_M;
        MemtoReg_WB  <= MemtoReg_M;

      end
    end
endmodule

///////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ns
module mux2bits (d0,d1,s,y);
input [31:0] d0;
input [31:0] d1;
input s;
output [31:0] y;

assign y = s ? d1 : d0;

endmodule
//////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ns
module mux3bits (d0,d1,d2,s,y);

input [31:0] d0,d1,d2;
input [1:0] s;
output reg [31:0] y;

always @* begin

    case(s)
        2'b00: y<=d0;
        2'b01: y<=d1;
        2'b10: y<=d2;
    endcase
    
end

endmodule
//////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ns

module hazardunit(      input [4:0] Rt_EX,            
                        input [4:0] Rs_D,
                        input [4:0] Rt_D,
                        input [4:0] writereg_M,
                        input [4:0] writereg_EX,
                        input MemtoReg_E,
                        input MemtoReg_M,
                        input RegWrite_EX,
                        input Branch_ID, 
                        output reg stall_IF_ID,
                        output reg stall_ID_EX,
                        output reg flush_EX_Mem);
reg lwstall, branchstall;
always @(*) begin

    lwstall= ((Rs_D==Rt_EX) || (Rt_D==Rt_EX)) && MemtoReg_E;

    branchstall =Branch_ID &
            (RegWrite_EX &
            (writereg_EX == Rs_D | writereg_EX == Rt_D) |
             MemtoReg_M &
            (writereg_M == Rs_D | writereg_M == Rt_D));



    stall_ID_EX = lwstall || branchstall ;
    stall_IF_ID = lwstall || branchstall ;
    flush_EX_Mem = lwstall || branchstall ;

    end

endmodule 

/////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ns

module forwardingunit(  input [4:0] Rs_EX,            
                      	input [4:0] Rt_EX,
                        input [4:0] Rs_ID,            
                      	input [4:0] Rt_ID,            
                      	input [4:0] writereg_M,       
                      	input [4:0] writereg_WB,      
                        input RegWrite_M,       
                        input RegWrite_WB,      
                        output reg[1:0] ForwardAE,  
                        output reg[1:0] ForwardBE,
                        output reg ForwardAD,  
                        output reg ForwardBD );

always @(*)
    begin
        
        if (RegWrite_M
            && (writereg_M != 0)
            && (writereg_M == Rs_EX))
            ForwardAE = 2'b10;
        
        else if (RegWrite_WB
            && (writereg_WB != 0)
            && (writereg_WB == Rs_EX))
            ForwardAE = 2'b01;
        
        else
            ForwardAE = 2'b00;

        
        if (RegWrite_M
            && (writereg_M != 0)
            && (writereg_M == Rt_EX))
            ForwardBE = 2'b10;
            
        else if (RegWrite_WB
            && (writereg_WB != 0)
            && (writereg_WB == Rt_EX))
            ForwardBE = 2'b01;
            
        else
            ForwardBE = 2'b00;

        ForwardAD = (writereg_M !=0) && (Rs_ID == writereg_M) && RegWrite_M;
        ForwardBD = (writereg_M !=0) && (Rt_ID == writereg_M) && RegWrite_M;
    end

endmodule 

/////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ns
module Datapath(input clk,
                input reset,
                input RegDst_ID,
                input RegWrite_ID,
                input ALUSrc_ID,
                input B,
                input MemtoReg_ID,
                input MemWrite_ID,
                input Branch_ID,
                input [3:0] ALUControl_ID,
                input [31:0] ReadData_M,
                input [31:0] Instr_IF,
                output MemWrite_M,
                output [31:0] Instr_ID,
                output [31:0] PC_IF,
                output [31:0] WriteData_M, 
                output [31:0] ALUResult_M);


wire [31:0] PCNEXT_IF, PCplus4_IF, PCplus4_ID;
wire [31:0] PCBranch_ID,PCbeforeBranch;
wire [31:0] extendedimm_ID, extendedimm_Ex, extendedimmafter;
wire [31:0] dataone_ID ,dataone_Ex;
wire [31:0] datatwo_ID, datatwo_Ex;
wire [31:0] ALUResult_Ex, ALUResult_WB, ALUResult_Mem;
wire [31:0] MUXresult_WB,  aluop2, SrcA_EX, SrcB_EX;
wire [31:0] ReadData_WB;
wire [4:0]  writereg_Ex, writereg_M, writereg_WB ;
wire ZeroFlag_Ex;
wire [31:0] Instr_Ex;
wire RegWrite_Ex, RegWrite_M, RegWrite_WB;
wire MemtoReg_Ex, MemtoReg_M, MemtoReg_WB;
wire MemWrite_Ex;
wire [3:0] ALUControl_Ex;
wire ALUSrc_Ex;
wire RegDst_Ex;
wire [1:0] ForwardAE,ForwardBE;
wire ForwardAD, ForwardBD;
wire Flush_Ex, Stall_IF, Stall_ID; 
wire BranchMUXselect, Equal_ID;
wire [31:0] equalone,equaltwo;


PC programcounter(clk, reset,!Stall_IF ,PCNEXT_IF, PC_IF);
ALU_Adder  pcadd4(PC_IF, 32'd4 , PCplus4_IF);

assign BranchMUXselect = ((B ^ Equal_ID ) & Branch_ID);
//mux before pc
mux2bits branchmux(PCplus4_IF , PCBranch_ID, BranchMUXselect , PCNEXT_IF);
  
IF_ID IfId(clk,reset | BranchMUXselect ,
Stall_ID,PCplus4_IF,PCplus4_ID,Instr_IF,Instr_ID);


signExtend SE(Instr_ID[15:0],extendedimm_ID);
ShiftLeft STL2(extendedimm_ID,extendedimmafter);
RegisterFile32bits RF(clk,RegWrite_WB, reset, Instr_ID[25:21], Instr_ID[20:16],
 writereg_WB, MUXresult_WB, dataone_ID,datatwo_ID); 
 
//comparator
mux2bits equalonemux(dataone_ID,ALUResult_Mem,ForwardAD,equalone);
mux2bits equaltwomux(datatwo_ID,ALUResult_Mem,ForwardBD,equaltwo);
assign Equal_ID = (equalone==equaltwo);

ALU_Adder pcaddsigned(extendedimmafter, PCplus4_ID, PCBranch_ID);

ID_EX IdEx(clk, reset , dataone_ID, dataone_Ex,datatwo_ID,
datatwo_Ex, extendedimm_ID,extendedimm_Ex, Instr_ID,Instr_Ex, RegWrite_ID,
RegWrite_Ex, MemtoReg_ID, MemtoReg_Ex, MemWrite_ID,MemWrite_Ex, ALUControl_ID,
ALUControl_Ex, ALUSrc_ID, ALUSrc_Ex,RegDst_ID, RegDst_Ex);


mux2bits writeopmux(Instr_Ex[20:16],Instr_Ex[15:11],RegDst_Ex, writereg_Ex);
mux3bits forwardmuxA (dataone_Ex, MUXresult_WB, ALUResult_Mem, ForwardAE, SrcA_EX);
mux3bits forwardmuxB (datatwo_Ex, MUXresult_WB, ALUResult_Mem, ForwardBE, aluop2);
ALU32bits alucomp(SrcA_EX, SrcB_EX, ALUControl_Ex, ALUResult_Ex, ZeroFlag_Ex);
mux2bits aluop2sel(aluop2,extendedimm_Ex, ALUSrc_Ex, SrcB_EX);


EX_M ExMem(clk, reset, ALUResult_Ex, ALUResult_Mem,
aluop2, WriteData_M, writereg_Ex, writereg_M,
RegWrite_Ex, RegWrite_M, MemtoReg_Ex, MemtoReg_M,
MemWrite_Ex, MemWrite_M );

assign ALUResult_M = ALUResult_Mem;

forwardingunit Forward_Unit( Instr_Ex [25:21], Instr_Ex [20:16],
Instr_ID [25:21], Instr_ID [20:16], writereg_M, writereg_WB, 
RegWrite_M, RegWrite_WB, ForwardAE, ForwardBE, ForwardAD, ForwardBD);
  
hazardunit hazard_unit(Instr_Ex [20:16], Instr_ID [25:21], Instr_ID [20:16]
, writereg_M,writereg_Ex,MemtoReg_Ex,MemtoReg_M,RegWrite_Ex,Branch_ID,
Stall_IF,Stall_ID,Flush_Ex );


M_WB MemWb(clk,reset, ReadData_M, ReadData_WB, ALUResult_M,
ALUResult_WB, writereg_M, writereg_WB,RegWrite_M, RegWrite_WB,
MemtoReg_M, MemtoReg_WB);


mux2bits resultmux(ALUResult_WB, ReadData_WB, MemtoReg_WB, MUXresult_WB); 

endmodule


///////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ns
module MIPS_PP(input clk,input reset);
                
wire [31:0] PC, Instr_IF, Instr_ID, ReadData, WriteData, ALUResult;
wire RegDst,RegWrite, ALUSrc, Jump, MemtoReg, B , MemWrite_ID, MemWrite_M, Branch;
wire [3:0] ALUControl;

Datapath datapath(clk, reset, RegDst,RegWrite, ALUSrc, B,MemtoReg ,
MemWrite_ID,Branch , ALUControl,ReadData, Instr_IF, MemWrite_M, Instr_ID ,PC, 
WriteData,ALUResult);


Controlunit controller(Instr_ID[31:26], Instr_ID[5:0],MemtoReg,MemWrite_ID,
ALUSrc, RegDst, RegWrite, Jump, Branch, B, ALUControl);


DataMem dmem(clk,MemWrite_M,ALUResult, WriteData, ReadData);

InstMem imem(PC,Instr_IF);

endmodule

////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ns
module MIPS_PP_tb;

	reg clk;
   reg reset;

	MIPS_PP mpp (clk,reset);

   always #50 clk=!clk;
	initial begin
	   clk=0;
	   reset=1;
	   #100; //cycle 1
      repeat(13) begin
	      reset=0;
	      #100;
      end
      $finish;
  	end
 endmodule