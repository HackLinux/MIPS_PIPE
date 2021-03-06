/*
 *=========================================================================================
 * Caminho de dados processador pipeline
 * input: 
 *	iCLK - Clock
 *	iRST - Reset
 *  output:
 *	none
 *
 *=========================================================================================
 *
 * Conside��es sobre o Pipeline
 *
 *	- Tamanho dos registradores entre est�gios:
 *
 *		@ IF/ID: PC+4 (32 bits), Instru��o (32 bits) ========= Total de 64 bits
 *
 *		@ ID/EX: Opcode (6 bits),  RegWB (3 bits), RegM (2 bits),  RegEX (6 bits) , PC+4 (32 bits), DadosLeitura1 (32 bits), DadosLeitura2 (32 bits), ImmExtSinal (32 bits), ImmExtZero (32 bits), 
 *		ImmConcatZero (32 bits), Shamt (5 bits),  NumRs (5 bits), NumRt (5 bits), NumRd (5 bits), ========== Total de 229 bits
 *
 *		@ EX/MEM: RegWB (3 bits), RegM (2 bits), PC+4 (32 bits), ResultALU (32 bits), ResultFowardB (32 bits), RegDestino (5 bits) ============ Total de 106 bits
 *
 *		@ MEM/WB:RegWB(3 bits), PC+4 (32 bits), DadosLeituraMem (32 bits), ResultALU (32 bits), RegDestino (5 bits) ============ Total de 104 bits
 *
 *		*** Cada um dos bits presentes nos registradores de est�gio est�o especificados na figura regs_estado.png
 *
 *	- Estou fundindo os MUX em s�rie em um s�:
 *
 *		- Os dois mux antes do PC ser�o controlados pelo sinal OrigPC: (MODIFICADO!!)
 *			- 000: PC+4
 *			- 001: Endere�o do Branch
 *			- 010: Endere�o do Jump
 *			- 011: Nada
 *		    - 100: 0x4000 .ktext
		    - 111: wPC 
		    
 *		- Os dois mux da parte debaixo da etapa EX ser�o controlados pelo sinal RegDst:
 *			- 00: Rt
 *			- 01: Rd
 *			- 10: 31 ($Ra)
 *			- 11: Nada
 *
 *		- Os dois mux do final da etapa WB ser�o controlados pelo sinal MemParaReg:
 *			- 00: Dados da MD
 *			- 01: Resultado da ALU
 *			- 10: PC+4
 *			- 11: Nada
 *
 */
 //adicionado os sinais jump, branch e jr vindos do controle
//os wXXXX da entrada sao na verdade oXXXX
module MIPS (
	iCLK, //OK
	iCLKMem, //OK
	iCLK50, //OK
	iRST, //OK
	wPC, //OK
	wCRegDst, //OK
	wCOrigALU,  //OK
	wCMem2Reg,  //OK
	wCRegWrite,  //OK
	wCMemRead,  //OK
	wCMemWrite,  //OK
	wCOrigPC, //OK
	wCJump, //OK
	wCBranch, //OK
	wCJr, //OK
	wOpcode, //OK
	wFunct, //OK
	wRegDispSelect, //OK
	wiRegA0, //OK
	wCInputA0En, //OK
	wRegDisp, //OK
	wCALUOp, //OK
	woInstr, //OK
	wDebug, //OK
	wMEM_ResultALU, 
	wMEM_ResultFowardB,
	wMEM_MemWrite,
	iwAudioCodecData
);

/* Padrao de nomeclatura
 *
 * XXXXX - registrador XXXX
 * wXXXX - wire XXXX
 * wCXXX - wire do sinal de controle XXX
 * memXX - memoria XXXX
 * Xunit - unidade funcional X
 * iXXXX - sinal de entrada/input
 * oXXXX - sinal de saida/output
 */

//********************* AS DEFINI��ES DOS EST�GIOS DO PIPELINE COME�AM AQUI *********************//
input wire	iCLK, iCLKMem, iCLK50, iRST;//clocks e o reset

input wire [4:0] wRegDispSelect;//seleciona o registrador que vai ser mostrado no display da placa

//fios para debug - BEGIN
input wire [31:0] wiRegA0;
input wire wCInputA0En;//permite que o a0 seja modificado pelas chaves da placa
output wire [31:0] wDebug;
//fios para debug - END

output wire [31:0] wPC, //fios do PC
					wRegDisp,//fios com os bits que ser�o mostrados no display da placa quando selecionado um registrador
					woInstr;//fios da instru��o a ser mostarda no display

//Sinais do bloco controlador - BEGIN
output wire  wCRegWrite, wCMemRead, wCMemWrite, wCJump,wCBranch, wCJr;
output wire [1:0] wCALUOp, wCOrigALU, wCRegDst, wCMem2Reg;
output wire [2:0] wCOrigPC;
//Sinais do bloco controlador - END

//Sinais da unidade de foward
wire [1:0] wFU_FowardA, wFU_FowardB;

//================ Estruturas do Est�gio IF - BEGIN ===================//
reg[31:0] PC; //registrador do PC
wire [31:0] wiPC;//fio do PC
wire [31:0] wPC4; //fio do PC+4
wire [31:0] wInstr;//fio da Instru��o

reg[63:0] RegIFID;//registrador do est�gio IF/ID
//================ Estruturas do Est�gio IF - END ===================//


//================ Estruturas do Est�gio ID - BEGIN ===================//
wire [31:0] wID_PC4;//PC+4 do IF/ID

wire [4:0] wNumRs, //N�mero do registrador Rs
			wNumRt, //N�mero do registrador Rt 
			wNumRd, //N�mero do registrador Rd
			//wRegDst, //N�mero do registrador que ser� escrito no Banco de Registradores N�O PRECISO DISSO POR ENQUANTO, REMOVER SE POSS�VEL
			wShamt; //bits [10:6]  Shift amount

output wire [5:0] wOpcode, wFunct;//declarados no cabe�alho, pois s�o outputs

wire [31:0] wResultJr;//resultado do mux Jr
wire [31:0] wResultFowardJr;//resultado do mux de foward do Jr
//wire [31:0] wResultFowardPC4;//resultado do mux de foward do PC4 que fica depois do mux de foward do Jr
wire [31:0] wJrAddr;//endere�o do Addr
wire [31:0] wJumpAddr;//Endere�o do Jump
wire [31:0] wBranchPC;//Endere�o do Branch
wire [15:0] wImm;//Imediato
wire [31:0] wExtSigImm;//Imediato com extens�o de sinal
wire [31:0] wExtZeroImm;//Imediato com extens�o com zeros
wire [31:0] wConcatZeroImm;//Imediato concatenado com zeros, para o lui
wire [31:0] wRead1, wRead2;//Dados de Leitura 1 e 2 do Banco de registradores

wire wEqual;//indica se os dois registradores lidos s�o iguais

wire wHU_BlockPC, wHU_BlockIFID, wHU_FlushControl, wHU_FowardJr, wHU_FowardPC4;

wire wIFID_Flush;

reg [31:0] Ra_Aux;
wire [31:0] wRa_Aux;

reg[228:0] RegIDEX;//registrador do est�gio ID/EX
//================ Estruturas do Est�gio ID - END ===================//

//================ Estruturas do Est�gio EX - BEGIN ===================//
wire [31:0] wEX_PC4;//PC+4 do ID/EX
wire [31:0] wEX_Read1, wEX_Read2;//Dados de leitura 1 e 2 do Banco de Registradores
wire [31:0] wEX_ExtSigImm, wEX_ExtZeroImm, wEX_ConcatZeroImm;
wire [31:0] wEX_ResultALU;//resultado na sa�da da ALU
wire [31:0] wEX_ResultFowardA;//resultado do mux FowardA
wire [31:0] wEX_ResultFowardB;//resultado do mux FowardB
wire [31:0] wEX_ResultOrigALU;//resultado do mux controlado por OrigALU

wire [5:0] wEX_Opcode;
wire [4:0] wEX_NumRs, wEX_NumRt, wEX_NumRd, wEX_Shamt;
wire [3:0] wALUControl;//fio que sai da ALUControl e entra na ULA

wire [2:0] wEX_WB;//cont�m os sinais EscreveReg, MemParaReg
wire [1:0] wEX_M;//cont�m os sinais EscreveMem, LeMem
wire [1:0] wEX_RegDst;//fio do sinal de controle RegDst
wire [4:0] wEX_RegDestino;//n�mero do registrador de destino 
wire [1:0] wEX_ALUOp, wEX_OrigALU;

wire wEX_Zero, wEX_Overflow;

wire wEX_MemRead;

wire [1:0] wEX_FowardA, wEX_FowardB;//fios que controlam os mux de fowarding: ESSES FIOS TAMB�M EST�O ATUALMENTE DESCONECTADOS!


reg[105:0] RegEXMEM;//registrador do est�gio EX/MEM
//================ Estruturas do Est�gio EX - END ===================//


//================ Estruturas do Est�gio MEM - BEGIN ===================//
input wire [31:0] iwAudioCodecData;
wire [31:0] wMEM_PC4;//PC+4 do EX/MEM

output wire [31:0] wMEM_ResultALU;//resultado na sa�da da ALU
output wire [31:0] wMEM_ResultFowardB;//resultado do mux FowardB
wire [31:0] wMEM_DataFromMem;//fio que sai da Mem�ria de Dados ATUALMENTE DESCONECTADO!

wire [2:0] wMEM_WB;//cont�m os sinais EscreveReg, MemParaReg
wire wMEM_MemRead, wMEM_RegWrite;
output wire wMEM_MemWrite;

wire [4:0] wMEM_RegDestino;//n�mero do registrador de destino 

reg[103:0] RegMEMWB;//registrador do est�gio MEM/WB
//================ Estruturas do Est�gio MEM - END ===================//

//================ Estruturas do Est�gio WB - BEGIN ===================//
wire [31:0] wWB_PC4;//PC+4 do MEM/WB

wire [31:0] wWB_DataFromMem;//fio que sai da Mem�ria de Dados
wire [31:0] wWB_ResultALU;//resultado na sa�da da ALU
wire [31:0] wWB_WriteData;//dados que v�o ser escritos no BR 

wire [4:0] wWB_RegDestino;//n�mero do registrador de destino 

wire [1:0] wWB_Mem2Reg;
wire wWB_RegWrite;
//================ Estruturas do Est�gio WB - END ===================//


initial
begin
	PC <= 32'b0;
	wiPC <= 32'b0;
	RegIFID <= 64'b0;
	wCRegWrite <= 1'b0;
	wCMemRead <= 1'b0;
	wCMemWrite <= 1'b0;
	wCJump <= 1'b0;
	wCBranch <= 1'b0;
	wCnBranch <= 1'b0;
	wCJr <= 1'b0;
	wCALUOp <= 2'b0;
	wCOrigALU <= 2'b0;
	wCRegDst <= 2'b0;
	wCMem2Reg <= 2'b0;
	wCOrigPC <= 3'b0;
	RegIDEX <= 229'b0;
	RegEXMEM <= 106'b0;
	RegMEMWB <= 104'b0;
	wFU_FowardA <= 2'b0;
	wFU_FowardB <= 2'b0;
	wHU_BlockPC <= 1'b0;
	wHU_BlockIFID <= 1'b0;
	wHU_FlushControl <= 1'b0;
	wIFID_Flush <= 1'b0;
	wHU_FowardJr <= 1'b0;
	wHU_FowardPC4 <= 1'b0;
end


//================ Assigns do Est�gio IF - BEGIN ===================//
assign wPC = PC;
assign wPC4	= wPC + 32'h4;  /* Calculo PC+4 */
// assign RegIFID[31:0] = wInstr;
// assign RegIFID[63:32] = wPC4;
assign woInstr = wInstr;
//================ Assigns do Est�gio IF - END ===================//

//================ Assigns do Est�gio ID - BEGIN ===================//
assign wID_PC4 = RegIFID[63:32];

assign wOpcode = RegIFID[31:26];
assign wNumRs = RegIFID[25:21];
assign wNumRt = RegIFID[20:16];
assign wNumRd = RegIFID[15:11];
assign wShamt  = RegIFID[10:6];
assign wFunct = RegIFID[5:0];

assign wImm = RegIFID[15:0];
assign wExtSigImm = {{16{wImm[15]}},wImm};
assign wExtZeroImm = {{16'b0},wImm};
assign wConcatZeroImm = {wImm,{16'b0}};

assign wJumpAddr = {wID_PC4[31:28],RegIFID[25:0],{2'b00}};
assign wBranchPC = wID_PC4 + {wExtSigImm[29:0],{2'b00}};
assign wJrAddr = wRead1;

assign wEqual = (wRead1==wRead2) ? 1'b1 : 1'b0;

assign wIFID_Flush = ((wCJump) || ((wCBranch) && (wEqual)) || ((wCnBranch) && (~wEqual))) ? 1'b1 : 1'b0; // colocado | a mais

assign wRa_Aux = Ra_Aux;

// assign RegIDEX[4:0] = wNumRd;
// assign RegIDEX[9:5] = wNumRt;
// assign RegIDEX[14:10] = wNumRs;
// assign RegIDEX[19:15] = wShamt;
// assign RegIDEX[51:20] = wConcatZeroImm;
// assign RegIDEX[83:52] = wExtZeroImm;
// assign RegIDEX[115:84] = wExtSigImm;
// assign RegIDEX[147:116] = wRead2;
// assign RegIDEX[179:148] = wRead1;
// assign RegIDEX[211:180] = wID_PC4;
// assign RegIDEX[213:212] = wCALUOp;
// assign RegIDEX[215:214] = wCOrigALU;
// assign RegIDEX[217:216] = wCRegDst;
// assign RegIDEX[218] = wCMemRead;
// assign RegIDEX[219] = wCMemWrite;
// assign RegIDEX[221:220] = wCMem2Reg;
// assign RegIDEX[222] = wCRegWrite;
// assign RegIDEX[228:223] = wOpcode;
//================ Assigns do Est�gio ID - END ===================//

//================ Assigns do Est�gio EX - BEGIN ===================//
assign wEX_PC4 = RegIDEX[211:180];

assign wEX_Opcode = RegIDEX[228:223];

assign wEX_WB = RegIDEX[222:220];
assign wEX_M = RegIDEX[219:218];

assign wEX_RegDst = RegIDEX[217:216];
assign wEX_OrigALU = RegIDEX[215:214];
assign wEX_ALUOp = RegIDEX[213:212];
assign wEX_Read1 = RegIDEX[179:148];
assign wEX_Read2 = RegIDEX[147:116];
assign wEX_ExtSigImm = RegIDEX[115:84];
assign wEX_ExtZeroImm = RegIDEX[83:52];
assign wEX_ConcatZeroImm = RegIDEX[51:20];
assign wEX_Shamt = RegIDEX[19:15];
assign wEX_NumRs = RegIDEX[14:10];
assign wEX_NumRt = RegIDEX[9:5];
assign wEX_NumRd = RegIDEX[4:0];

assign wEX_MemRead = RegIDEX[218];

//assign wEX_ResultFowardB = wEX_Read2;//enquanto n�o tem a unidade de fowarding****************

//Precisa concectar fios do ExtSigImm na ula (os 6 bits do funct)

// assign RegEXMEM[105:103] = wEX_WB;
// assign RegEXMEM[102:101] = wEX_M;
// assign RegEXMEM[100:69] = wEX_PC4;
// assign RegEXMEM[68:37] = wEX_ResultALU;
// assign RegEXMEM[36:5] = wEX_ResultFowardB;
// assign RegEXMEM[4:0] = wEX_RegDestino;
//================ Assigns do Est�gio EX - END ===================//

//================ Assigns do Est�gio MEM - BEGIN ===================//
assign wMEM_PC4 = RegEXMEM[100:69];

assign wMEM_ResultALU = RegEXMEM[68:37];
assign wMEM_ResultFowardB = RegEXMEM[36:5];
assign wMEM_WB = RegEXMEM[105:103];

assign wMEM_RegWrite = RegEXMEM[105];
assign wMEM_MemWrite = RegEXMEM[102];
assign wMEM_MemRead = RegEXMEM[101];

assign wMEM_RegDestino = RegEXMEM[4:0];

// assign RegMEMWB[103:101] = wMEM_WB;
// assign RegMEMWB[100:69] = wMEM_PC4;
// assign RegMEMWB[68:37] = wMEM_DataFromMem;
// assign RegMEMWB[36:5] = wMEM_ResultALU;
// assign RegMEMWB[4:0] = wMEM_RegDestino;
//================ Assigns do Est�gio MEM - END ===================//

//================ Assigns do Est�gio WB - BEGIN ===================//
assign wWB_PC4 = RegMEMWB[100:69];

assign wWB_DataFromMem = RegMEMWB[68:37];
assign wWB_ResultALU = RegMEMWB[36:5];
assign wWB_RegDestino = RegMEMWB[4:0];

assign wWB_RegWrite = RegMEMWB[103];
assign wWB_Mem2Reg = RegMEMWB[102:101];
//================ Assigns do Est�gio WB - END ===================//

/* Assigns para debug */
assign wDebug = wiRegA0;

//============================= BLOCOS ADICIONAIS - BEGIN =============================//

/* Mem�ria de Instru��es */
CodeMemory memInstr(
	.iCLK(iCLK),
	.iCLKMem(iCLK50),
	.iByteEnable(4'b1111),
	.iAddress(wPC),
	.iWriteData(ZERO),
	.iMemRead(ON),
	.iMemWrite(OFF),
	.oMemData(wInstr),
	.iwAudioCodecData()
);

/* Banco de Registradores */
Registers memReg(
	.iCLK(iCLK),
	.iCLR(iRST),
	.iReadRegister1(wNumRs),
	.iReadRegister2(wNumRt),
	.iWriteRegister(wWB_RegDestino),
	.iWriteData(wWB_WriteData), 
	.iRegWrite(wWB_RegWrite),
	.oReadData1(wRead1),
	.oReadData2(wRead2),
	.iRegDispSelect(wRegDispSelect),
	.oRegDisp(wRegDisp),
	.iRegA0(wiRegA0),
	.iA0en(wCInputA0En)
 );
 
 /* ALU CTRL */
ALUControl ALUControlunit (
	.iFunct(wEX_ExtSigImm[5:0]), 
	.iOpcode(wEX_Opcode), 
	.iALUOp(wEX_ALUOp), 
	.oControlSignal(wALUControl)
);

/* ALU */
ALU ALUunit(
	.iCLK(iCLK),
	.iRST(iRST),
	.iControlSignal(wALUControl),
	.iA(wEX_ResultFowardA), 
	.iB(wEX_ResultOrigALU),
	.iShamt(wEX_Shamt),
	.oALUresult(wEX_ResultALU),
	.oZero(wEX_Zero),
	.oOverflow(wEX_Overflow)
);

//unidade de foward
FowardUnit fUnit(
	.iEX_NumRs(wEX_NumRs),
	.iEX_NumRt(wEX_NumRt),
	.iMEM_NumRd(wMEM_RegDestino),
	.iMEM_RegWrite(wMEM_RegWrite),
	.iWB_NumRd(wWB_RegDestino),
	.iWB_RegWrite(wWB_RegWrite),
	.oFowardA(wFU_FowardA),
	.oFowardB(wFU_FowardB)
);

/* memoria de dados */
DataMemory memData(
	.iCLK(iCLK),
	.iCLKMem(iCLK50),
	.iByteEnable(4'b1111), 
	.iAddress(wMEM_ResultALU), 
	.iWriteData(wMEM_ResultFowardB),
	.iMemRead(wMEM_MemRead), 
	.iMemWrite(wMEM_MemWrite),
	.oMemData(wMEM_DataFromMem),
	.iwAudioCodecData(iwAudioCodecData)
);

//unidade de hazard
HazardUnit hUnit (
	.iNumRs(wNumRs), 
	.iNumRt(wNumRt), 
	.iEX_NumRt(wEX_NumRt), 
	.iEX_MemRead(wEX_MemRead), 
	.iCJr(wCJr),
	.iEX_RegDestino(wEX_RegDestino),
	.iMEM_RegDestino(wMEM_RegDestino),
	.oBlockPC(wHU_BlockPC), 
	.oBlockIFID(wHU_BlockIFID),
	.oFlushControl(wHU_FlushControl),
	.oFowardJr(wHU_FowardJr),
	.oFowardPC4(wHU_FowardPC4)
);

Control Controlunit (
	.iCLK(iCLK),
	.iOp(wOpcode),
	.iFunct(wFunct),
	.oRegDst(wCRegDst),
	.oOrigALU(wCOrigALU),
	.oMemparaReg(wCMem2Reg),
	.oEscreveReg(wCRegWrite),
	.oLeMem(wCMemRead),
	.oEscreveMem(wCMemWrite),
	.oOpALU(wCALUOp),
	.oOrigPC(wCOrigPC),
	.oJump(wCJump),
	.oBranch(wCBranch),
	.onBranch(wCnBranch),
	.oJr(wCJr)
);
//============================= BLOCOS ADICIONAIS - END =============================//

//============================= MUXS - BEGIN =============================//

always @(wCJr)
begin
	case(wCJr)
		1'b0:
			wResultJr <= wJumpAddr;
		1'b1:
			wResultJr <= wJrAddr;
	endcase
end

always @(wHU_FowardJr)
begin
	case(wHU_FowardJr)
		1'b0:
			wResultFowardJr <= wResultJr;
		1'b1:
			wResultFowardJr <= wEX_ResultALU;
	endcase
end

always @(negedge iCLK)
begin
	if (wHU_FowardPC4) begin
		Ra_Aux <= wMEM_PC4; 
	end
end 


always @(wFU_FowardA)
begin
	case(wFU_FowardA)
		2'b00:
			wEX_ResultFowardA <= wEX_Read1;
		2'b01:
			wEX_ResultFowardA <= wWB_WriteData;
		2'b10:
			wEX_ResultFowardA <= /*(wEX_Opcode==6'h2B) ? wEX_Read1 :*/ wMEM_ResultALU;//se a instru��o for diferente de SW pode fazer o foward!
		default:
			wEX_ResultFowardA <= wEX_Read1;
	endcase
end

always @(wFU_FowardB)
begin
	case(wFU_FowardB)
		2'b00:
			wEX_ResultFowardB <= wEX_Read2;
		2'b01:
			wEX_ResultFowardB <= wWB_WriteData;
		2'b10:
			wEX_ResultFowardB <= wMEM_ResultALU;
		default:
			wEX_ResultFowardB <= wEX_Read2;
	endcase
end

always @(wCOrigPC,iCLK)
begin
	case(wCOrigPC)
		3'b000:
			wiPC <= wPC4;
		3'b001:
			wiPC <= wEqual ? wBranchPC : wPC4;
		3'b010:
			wiPC <= (wHU_FowardPC4) ? wRa_Aux : wResultFowardJr;			
//		3'b011:
//			wiPC <= wEX_ResultFowardA; /////////
		3'b100 :
			wiPC <= 32'h00004000; //.ktext
		3'b101:
			wiPC <= ~wEqual ? wBranchPC : wPC4;
		default:
			wiPC <= wPC;
	endcase
end

always @(wEX_RegDst)
begin
	case(wEX_RegDst)
		2'b00:
			wEX_RegDestino <= wEX_NumRt;
		2'b01:
			wEX_RegDestino <= wEX_NumRd;
		2'b10:
			wEX_RegDestino <= 5'd31;
		default:
			wEX_RegDestino <= 5'd0;
	endcase
end

always @(wEX_OrigALU)
begin
	case(wEX_OrigALU)
		2'b00:
			wEX_ResultOrigALU <= wEX_ResultFowardB;
		2'b01:
			wEX_ResultOrigALU <= wEX_ExtSigImm;
		2'b10:
			wEX_ResultOrigALU <= wEX_ExtZeroImm;
		2'b11:
			wEX_ResultOrigALU <= wEX_ConcatZeroImm;
	endcase
end

always @(wWB_Mem2Reg)
begin
	case(wWB_Mem2Reg)
		2'b00:
			wWB_WriteData <= wWB_DataFromMem;
		2'b01:
			wWB_WriteData <= wWB_ResultALU;
		2'b10:
			wWB_WriteData <= wWB_PC4;
		2'b11:
			wWB_WriteData <= wConcatZeroImm;
	endcase
end

//============================= MUXS - END =============================//

always @(posedge iCLK)
begin
	if(iRST) begin
		PC <= 32'b0;
		//wPC <= PC;
		//wPC4 <= 32'b0;
		RegIFID <= 64'b0;	
		RegIDEX <= 229'b0;
		RegEXMEM <= 106'b0;
		RegMEMWB <= 104'b0;
	end
	else begin
		// Est�gio IF //
		if (!wHU_BlockPC) begin
			PC <= wiPC;
		end
		if (!wHU_BlockIFID) begin//ATEN��O SE ESTIVER BLOQUEADO N�O D� FLUSH!
			if (wIFID_Flush) begin
				RegIFID[31:0] <= 32'b0;
				RegIFID[63:32] <= 32'b0;
			end
			else begin
				RegIFID[31:0] <= wInstr; 
				RegIFID[63:32] <= wPC4; //wPC + 32'h4;
			end
		end
		// Est�gio IF //
		
		// Est�gio ID //
		RegIDEX[4:0] <= wNumRd;
		RegIDEX[9:5] <= wNumRt;
		RegIDEX[14:10] <= wNumRs;
		RegIDEX[19:15] <= wShamt;
		RegIDEX[51:20] <= wConcatZeroImm;
		RegIDEX[83:52] <= wExtZeroImm;
		RegIDEX[115:84] <= wExtSigImm;
		RegIDEX[147:116] <= wRead2;
		RegIDEX[179:148] <= wRead1;
		RegIDEX[211:180] <= wID_PC4;
		if (wHU_FlushControl) begin
			RegIDEX[213:212] <= 2'b0;
			RegIDEX[215:214] <= 2'b0;
			RegIDEX[217:216] <= 2'b0;
			RegIDEX[218] <= 1'b0;
			RegIDEX[219] <= 1'b0;
			RegIDEX[221:220] <= 2'b0;
			RegIDEX[222] <= 1'b0;
		end
		else begin
			RegIDEX[213:212] <= wCALUOp;
			RegIDEX[215:214] <= wCOrigALU;
			RegIDEX[217:216] <= wCRegDst;
			RegIDEX[218] <= wCMemRead;
			RegIDEX[219] <= wCMemWrite;
			RegIDEX[221:220] <= wCMem2Reg;
			RegIDEX[222] <= wCRegWrite;
		end
		RegIDEX[228:223] <= wOpcode;
		// Est�gio ID //
		
		// Est�gio EX //		
		RegEXMEM[105:103] <= wEX_WB;
		RegEXMEM[102:101] <= wEX_M;
		RegEXMEM[100:69] <= wEX_PC4;
		RegEXMEM[68:37] <= wEX_ResultALU;
		RegEXMEM[36:5] <= wEX_ResultFowardB;
		RegEXMEM[4:0] <= wEX_RegDestino;
		// Est�gio EX //
		
		// Est�gio MEM //
		RegMEMWB[103:101] <= wMEM_WB;
		RegMEMWB[100:69] <= wMEM_PC4;
		RegMEMWB[68:37] <= wMEM_DataFromMem;
		RegMEMWB[36:5] <= wMEM_ResultALU;
		RegMEMWB[4:0] <= wMEM_RegDestino;
		// Est�gio MEM //
		
		// Est�gio WB //
		// Est�gio WB //
	end
		
end

endmodule
