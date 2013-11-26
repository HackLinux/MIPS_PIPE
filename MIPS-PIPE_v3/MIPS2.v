/*
 * Caminho de dados processador pipeline
 *
 *	- Tamanho dos registradores entre estagios:
 *
 *		@ IF/ID: PC+4 (32), Instrucao (32) = Total de 64 bits
 *		@ ID/EX: Opcode (6), RegWB (3), RegM (2), RegEX (6), PC+4 (32), DadosLeitura1 (32), DadosLeitura2 (32), ImmExtSinal (32), ImmExtZero (32), 
 *		ImmConcatZero (32), Shamt (5),  NumRs (5), NumRt (5), NumRd (5), = Total de 229 bits
 *		@ EX/MEM: RegWB (3), RegM (2), PC+4 (32), ResultALU (32), ResultForwardB (32), RegDestino (5) = Total de 106 bits
 *		@ MEM/WB: RegWB (3), PC+4 (32), DadosLeituraMem (32), ResultALU (32), RegDestino (5) = Total de 104 bits
 *
 *	- Estou fundindo os MUX em serie em um so:
 *		- Os dois mux antes do PC serao controlados pelo sinal OrigPC: (MODIFICADO!!)
 *			- 000: PC+4
 *			- 001: Endereco do Branch
 *			- 010: Endereco do Jump
 *			- 011: Nada
 *		    - 100: 0x4000 .ktext
		    - 111: wPC 	    
 *		- Os dois mux da parte debaixo da etapa EX serao controlados pelo sinal RegDst:
 *			- 00: Rt
 *			- 01: Rd
 *			- 10: 31 ($Ra)
 *			- 11: Nada
 *		- Os dois mux do final da etapa WB serao controlados pelo sinal MemParaReg:
 *			- 00: Dados da MD
 *			- 01: Resultado da ALU
 *			- 10: PC+4
 *			- 11: Nada
 */
 //adicionado os sinais jump, branch e jr vindos do controle
//os wXXXX da entrada sao na verdade oXXXX
module MIPS2 (
	iCLK,
	iCLK50,
	iRST,
	woPC,
	woCRegDst,
	woCOrigALU,
	woCMem2Reg,
	woCRegWrite,
	woCMemRead,
	woCMemWrite,
	woCOrigPC,
	woCJump,
	woCBranch,
	woCJr,
	woOpcode,
	woFunct,
	iwRegDispSelect,
	iwRegA0,
	iwCInputA0En,
	woRegDisp,
	woCALUOp,
	woInstr,
	woDebug,
	woMemAddress,
	woMemWriteData,
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

// Clocks
input wire	iCLK, iCLK50, iRST;//clocks e o reset

// Definicoes dos fios de entrada
input wire [4:0] iwRegDispSelect;
input wire [31:0] iwRegA0;
input wire iwCInputA0En;
input wire [31:0] iwAudioCodecData;
output wire [31:0] woMemAddress, woMemWriteData ;

// Definicoes dos fios de saida
output wire [31:0]  woDebug, //Porta para Debug
					woPC, //fios do PC
					woRegDisp,//fios com os bits que serao mostrados no display da placa quando selecionado um registrador
					woInstr;//fios da instrucao a ser mostarda no display
output wire [5:0] woOpcode, woFunct;

//Sinais do bloco controlador
output wire  woCRegWrite, woCMemRead, woCMemWrite, woCJump,woCBranch, woCJr;
output wire [1:0] woCALUOp, woCOrigALU, woCRegDst, woCMem2Reg;
output wire [2:0] woCOrigPC;

/* Assigns para debug */
//RETIRAR
//assign woDebug = iwRegA0;
assign woDebug = {29'b0, wHU_BlockPC, wHU_BlockIFID, wHU_FlushControl};

/* Assigns para sinais de saida LEDs */
assign woPC        = PC;
assign woCRegDst   = wID_CRegDst;
assign woCOrigALU  = wID_COrigALU;
assign woCMem2Reg  = wID_CMem2Reg;
assign woCOrigPC   = wID_COrigPC;
assign woOpcode    = wID_Opcode;
assign woFunct     = wID_Funct;
assign woCRegWrite = wID_CRegWrite;
assign woCMemRead  = wID_CMemRead;
// assign woCMemWrite = wID_CMemWrite; // FUCKER!!!
assign woCMemWrite = wMEM_MemWrite; // CORRETO!
assign woCJump     = wID_CJump;
assign woCBranch   = wID_CBranch;
assign woCJr       = wID_CJr;

initial
begin
	PC       <= 32'b0;
	RegIFID  <= 64'b0;
	RegIDEX  <= 229'b0;
	RegEXMEM <= 106'b0;
	RegMEMWB <= 104'b0;
end

//================ Estruturas do Estagio IF - BEGIN ===================//
reg  [31:0] PC; //registrador do PC
reg  [31:0] wIF_iPC;//fios entrada do PC
wire [31:0] wIF_PC4; //fios do PC+4
wire [31:0] wIF_Instr;//fio da Instrucao

reg  [63:0] RegIFID;//registrador do estagio IF/ID

assign wIF_PC4 = PC + 32'h4;  /* Calculo PC+4 */
assign woInstr = wIF_Instr;

// Mux OrigemPC
//always @(wID_COrigPC, iCLK)
always @(*)
begin
	case(wID_COrigPC)
		3'b000:
			wIF_iPC = wIF_PC4;
		3'b001:
			wIF_iPC = wID_Equal ? wID_BranchPC : wID_PC4 + 32'h4;
		3'b010:
			wIF_iPC = (wHU_ForwardPC4) ? wRa_Aux : wID_ResultForwardJr;			
//		3'b011:
//			wiPC <= wEX_ResultForwardA; /////////
		3'b100 :
			wIF_iPC = 32'h00004000; //.ktext
		3'b101:
			wIF_iPC = ~wID_Equal ? wID_BranchPC : wID_PC4 + 32'h4;
		default:
			wIF_iPC = PC;
	endcase
end

/* Memoria de Instrucoes */
CodeMemory memInstr(
	.iCLK(iCLK),
	.iCLKMem(iCLK50),
	.iByteEnable(4'b1111),
	.iAddress(PC),
	.iWriteData(ZERO),
	.iMemRead(ON),
	.iMemWrite(OFF),
	.oMemData(wIF_Instr),
	.iwAudioCodecData()
);

//================ Estruturas do Estagio ID - BEGIN ===================//

wire [31:0] wID_PC4;//PC+4 do IF/ID

wire [4:0] wID_NumRs, //Numero do registrador Rs
			wID_NumRt, //Numero do registrador Rt 
			wID_NumRd, //Numero do registrador Rd
			wID_Shamt; //bits [10:6]  Shift amount

wire [31:0] wID_ResultJr;//resultado do mux Jr
wire [31:0] wID_ResultForwardJr;//resultado do mux de Forward do Jr
//wire [31:0] wResultForwardPC4;//resultado do mux de Forward do PC4 que fica depois do mux de Forward do Jr
wire [31:0] wID_JrAddr;//endereco do Addr
wire [31:0] wID_JumpAddr;//Endereco do Jump
wire [31:0] wID_BranchPC;//Endereco do Branch
wire [15:0] wID_Imm;//Imediato
wire [31:0] wID_ExtSigImm;//Imediato com extensao de sinal
wire [31:0] wID_ExtZeroImm;//Imediato com extensao com zeros
wire [31:0] wID_ConcatZeroImm;//Imediato concatenado com zeros, para o lui
wire [31:0] wID_Read1, wID_Read2;//Dados de Leitura 1 e 2 do Banco de registradores
wire [31:0] wID_ResultRead1, wID_ResultRead2;

wire [5:0] wID_Opcode,wID_Funct;
wire  wID_CRegWrite, wID_CMemRead, wID_CMemWrite, wID_CJump,wID_CBranch, wID_CnBranch, wID_CJr;
wire [1:0] wID_CALUOp, wID_COrigALU, wID_CRegDst, wID_CMem2Reg;
wire [2:0] wID_COrigPC;

wire wID_Equal;//indica se os dois registradores lidos sao iguais

wire wHU_BlockPC, wHU_BlockIFID, wHU_FlushControl, wHU_ForwardJr, wHU_ForwardPC4;

wire wIFID_Flush;

wire wHU_Branch;

// Teste
wire wFU_ForwardBranchRs, wFU_ForwardBranchRt;

reg [31:0] Ra_Aux;
wire [31:0] wRa_Aux;

reg [244:0] RegIDEX;//registrador do estagio ID/EX

assign wID_PC4    = RegIFID[63:32];
assign wID_Opcode = RegIFID[31:26];
assign wID_NumRs  = RegIFID[25:21];
assign wID_NumRt  = RegIFID[20:16];
assign wID_NumRd  = RegIFID[15:11];
assign wID_Shamt  = RegIFID[10:6];
assign wID_Funct  = RegIFID[5:0];
assign wID_Imm    = RegIFID[15:0];

assign wID_ExtSigImm     = {{16{wID_Imm[15]}},wID_Imm};
assign wID_ExtZeroImm    = {{16'b0},wID_Imm};
assign wID_ConcatZeroImm = {wID_Imm,{16'b0}};

assign wID_JumpAddr = {wID_PC4[31:28], RegIFID[25:0], {2'b00}};
assign wID_BranchPC = wID_PC4 + {wID_ExtSigImm[29:0], {2'b00}};
assign wID_JrAddr   = wID_Read1;

assign wHU_Branch = wID_CBranch | wID_CnBranch;

assign wID_Equal = (wID_ResultRead1 == wID_ResultRead2) ? 1'b1 : 1'b0;

/* Flush para desvios */
assign wIFID_Flush = ((wID_CJump) ||
					((wID_CBranch) && (wID_Equal)) || 
					((wID_CnBranch) && (~wID_Equal))) ? 
					1'b1 : 1'b0;

assign wRa_Aux = Ra_Aux;

always @(*)
begin
  wID_ResultJr = (wID_CJr) ? wID_JrAddr : wID_JumpAddr;
end

always @(*)
begin
  wID_ResultForwardJr = (wHU_ForwardJr) ? wEX_ResultALU : wID_ResultJr;
end

always @(*)
begin
  wID_ResultRead1 = (wFU_ForwardBranchRs) ? wMEM_ResultALU : wID_Read1;
end

always @(*)
begin
  wID_ResultRead2 = (wFU_ForwardBranchRt) ? wMEM_ResultALU : wID_Read2;
end

Registers memReg(
	.iCLK(iCLK),
	.iCLR(iRST),
	.iReadRegister1(wID_NumRs),
	.iReadRegister2(wID_NumRt),
	.iWriteRegister(wWB_RegDestino),
	.iWriteData(wWB_WriteData), 
	.iRegWrite(wWB_RegWrite),
	.oReadData1(wID_Read1),
	.oReadData2(wID_Read2),
	// debug
	.iRegDispSelect(iwRegDispSelect),
	.oRegDisp(woRegDisp),
	.iRegA0(iwRegA0),
	.iA0en(iwCInputA0En)
 );

Control Controlunit (
	.iCLK(iCLK),
	.iOp(wID_Opcode),
	.iFunct(wID_Funct),
	.oRegDst(wID_CRegDst),
	.oOrigALU(wID_COrigALU),
	.oMemparaReg(wID_CMem2Reg),
	.oEscreveReg(wID_CRegWrite),
	.oLeMem(wID_CMemRead),
	.oEscreveMem(wID_CMemWrite),
	.oOpALU(wID_CALUOp),
	.oOrigPC(wID_COrigPC),
	.oJump(wID_CJump),
	.oBranch(wID_CBranch),
	.onBranch(wID_CnBranch),
	.oJr(wID_CJr)
);

HazardUnit hUnit (
	.iID_NumRs(wID_NumRs), 
	.iID_NumRt(wID_NumRt), 
	.iEX_NumRt(wEX_NumRt), 
	.iEX_MemRead(wEX_MemRead), 
	.iCJr(wID_CJr),
	.iEX_RegDestino(wEX_RegDestino),
	.iMEM_MemRead(wMEM_MemRead),
	.iMEM_RegDestino(wMEM_RegDestino),
	.iBranch(wHU_Branch),
	.oBlockPC(wHU_BlockPC), 
	.oBlockIFID(wHU_BlockIFID),
	.oFlushControl(wHU_FlushControl),
	.oForwardJr(wHU_ForwardJr),
	.oForwardPC4(wHU_ForwardPC4)
);

//================ Estruturas do Estagio EX - BEGIN ===================//
wire [31:0] wEX_PC4;//PC+4 do ID/EX
wire [31:0] wEX_Read1, wEX_Read2;//Dados de leitura 1 e 2 do Banco de Registradores
wire [31:0] wEX_ExtSigImm, wEX_ExtZeroImm, wEX_ConcatZeroImm;
wire [31:0] wEX_ResultALU;//resultado na saida da ALU
wire [31:0] wEX_ResultForwardA;//resultado do mux ForwardA
wire [31:0] wEX_ResultForwardB;//resultado do mux ForwardB
wire [31:0] wEX_ResultOrigALU;//resultado do mux controlado por OrigALU
wire [15:0] wEX_Imm;
wire [5:0] wEX_Opcode;
wire [4:0] wEX_NumRs, wEX_NumRt, wEX_NumRd, wEX_Shamt;
wire [3:0] wEX_ALUControl;//fio que sai da ALUControl e entra na ULA

wire [2:0] wEX_WB;//contem os sinais EscreveReg, MemParaReg
wire [1:0] wEX_M;//contem os sinais EscreveMem, LeMem
wire [1:0] wEX_RegDst;//fio do sinal de controle RegDst
wire [4:0] wEX_RegDestino;//numero do registrador de destino 
wire [1:0] wEX_ALUOp, wEX_OrigALU;

wire wEX_Zero, wEX_Overflow;
wire wEX_MemRead;

wire [1:0] wFU_ForwardA, wFU_ForwardB;

reg [121:0] RegEXMEM;

assign wEX_Imm           = RegIDEX[244:229];
assign wEX_Opcode        = RegIDEX[228:223];
assign wEX_WB            = RegIDEX[222:220];
assign wEX_M             = RegIDEX[219:218];
assign wEX_MemRead       = RegIDEX[218];
assign wEX_RegDst        = RegIDEX[217:216];
assign wEX_OrigALU       = RegIDEX[215:214];
assign wEX_ALUOp         = RegIDEX[213:212];
assign wEX_PC4           = RegIDEX[211:180];
assign wEX_Read1         = RegIDEX[179:148];
assign wEX_Read2         = RegIDEX[147:116];
assign wEX_ExtSigImm     = RegIDEX[115:84];
assign wEX_ExtZeroImm    = RegIDEX[83:52];
assign wEX_ConcatZeroImm = RegIDEX[51:20];
assign wEX_Shamt         = RegIDEX[19:15];
assign wEX_NumRs         = RegIDEX[14:10];
assign wEX_NumRt         = RegIDEX[9:5];
assign wEX_NumRd         = RegIDEX[4:0];

//assign wEX_ResultForwardA = wEX_Read1;//enquanto nao tem a unidade de Forwarding****************
//assign wEX_ResultForwardB = wEX_ResultOrigALU;//enquanto nao tem a unidade de Forwarding****************

// Mux Forward A
//always @(wFU_ForwardA)
always @( * )
begin
	case(wFU_ForwardA)
		2'b00:   wEX_ResultForwardA = wEX_Read1;
		2'b01:   wEX_ResultForwardA = wWB_WriteData;
		2'b10:   wEX_ResultForwardA = wMEM_ResultALU;
		default: wEX_ResultForwardA = wEX_Read1;
	endcase
end

// Mux Forward B
//always @(wFU_ForwardB)
always @( * )
begin
	case(wFU_ForwardB)
		2'b00:   wEX_ResultForwardB = wEX_Read2;
		2'b01:   wEX_ResultForwardB = wWB_WriteData;
		2'b10:   wEX_ResultForwardB = wMEM_ResultALU;
		default: wEX_ResultForwardB = wEX_Read2;
	endcase
end

// MuX OriALU
//always @(wEX_OrigALU)
always @( * )
begin
	case(wEX_OrigALU)
		2'b00: wEX_ResultOrigALU = wEX_ResultForwardB;
		2'b01: wEX_ResultOrigALU = wEX_ExtSigImm;
		2'b10: wEX_ResultOrigALU = wEX_ExtZeroImm;
		2'b11: wEX_ResultOrigALU = wEX_ConcatZeroImm;
	endcase
end

//always @(wEX_RegDst)
always @( * )
begin
	case(wEX_RegDst)
		2'b00:   wEX_RegDestino = wEX_NumRt;
		2'b01:   wEX_RegDestino = wEX_NumRd;
		2'b10:   wEX_RegDestino = 5'd31;
		default: wEX_RegDestino = 5'd0;
	endcase
end

ALUControl ALUControlunit (
	.iFunct(wEX_ExtSigImm[5:0]), 
	.iOpcode(wEX_Opcode), 
	.iALUOp(wEX_ALUOp), 
	.oControlSignal(wEX_ALUControl)
);

ALU ALUunit(
	.iCLK(iCLK),
	.iRST(iRST),
	.iControlSignal(wEX_ALUControl),
	.iA(wEX_ResultForwardA), 
	.iB(wEX_ResultOrigALU),
	.iShamt(wEX_Shamt),
	.oALUresult(wEX_ResultALU),
	.oZero(wEX_Zero),
	.oOverflow(wEX_Overflow)
);

ForwardUnit fUnit(
	.iID_NumRs(wID_NumRs),
	.iID_NumRt(wID_NumRt),
	.iEX_NumRs(wEX_NumRs),
	.iEX_NumRt(wEX_NumRt),
	.iMEM_NumRd(wMEM_RegDestino),
	.iMEM_RegWrite(wMEM_RegWrite),
	.iWB_NumRd(wWB_RegDestino),
	.iWB_RegWrite(wWB_RegWrite),
	.iWB_MemRead(wMEM_MemRead),
	.oFwdA(wFU_ForwardA),
	.oFwdB(wFU_ForwardB),
	.oFwdBranchRs(wFU_ForwardBranchRs),
	.oFwdBranchRt(wFU_ForwardBranchRt)
);

//================ Estruturas do Estagio MEM - BEGIN ===================//

wire [31:0] wMEM_PC4;//PC+4 do EX/MEM
wire [31:0] wMEM_DataFromMem;//fio que sai da Memoria de Dados ATUALMENTE DESCONECTADO!
wire [2:0]  wMEM_WB;//contem os sinais EscreveReg, MemParaReg
wire wMEM_MemRead, wMEM_MemWrite, wMEM_RegWrite;

wire [4:0]  wMEM_RegDestino;//numero do registrador de destino 
wire [31:0] wMEM_ResultALU;//resultado na saida da ALU
wire [31:0] wMEM_ResultForwardB;//resultado do mux ForwardB
wire [15:0] wMEM_Imm;

reg [119:0] RegMEMWB;//registrador do estagio MEM/WB

assign wMEM_RegDestino     = RegEXMEM[4:0];
assign wMEM_ResultForwardB = RegEXMEM[36:5];
assign wMEM_ResultALU      = RegEXMEM[68:37];
assign wMEM_PC4            = RegEXMEM[100:69];
assign wMEM_MemRead        = RegEXMEM[101];
assign wMEM_MemWrite       = RegEXMEM[102];
assign wMEM_WB             = RegEXMEM[105:103];
assign wMEM_RegWrite       = RegEXMEM[105];
assign wMEM_Imm            = RegEXMEM[121:106];

assign woMemAddress   = wMEM_ResultALU;
assign woMemWriteData = wMEM_ResultForwardB;
	
/* memoria de dados */
DataMemory memData(
	.iCLK(iCLK),
	.iCLKMem(iCLK50),
	.iByteEnable(4'b1111), 
	.iAddress(woMemAddress), 
	.iWriteData(woMemWriteData),
	.iMemRead(wMEM_MemRead), 
	.iMemWrite(wMEM_MemWrite),
	.oMemData(wMEM_DataFromMem),
	.iwAudioCodecData(iwAudioCodecData)
);

//================ Estruturas do Estagio WB - BEGIN ===================//
wire [31:0] wWB_PC4;//PC+4 do MEM/WB

wire [31:0] wWB_DataFromMem;//fio que sai da Memoria de Dados
wire [31:0] wWB_ResultALU;//resultado na saida da ALU
wire [31:0] wWB_WriteData;//dados que vao ser escritos no BR 
wire [15:0] wWB_Imm; //Imediato para LUI
wire [4:0] wWB_RegDestino;//numero do registrador de destino 

wire [1:0] wWB_Mem2Reg;
wire wWB_RegWrite;

assign wWB_PC4 = RegMEMWB[100:69];

assign wWB_DataFromMem = RegMEMWB[68:37];
assign wWB_ResultALU = RegMEMWB[36:5];
assign wWB_RegDestino = RegMEMWB[4:0];

assign wWB_Mem2Reg = RegMEMWB[102:101];
assign wWB_RegWrite = RegMEMWB[103];
assign wWB_Imm = RegMEMWB[119:104];

//Mux MemtoReg
always @(wWB_Mem2Reg)
begin
	case(wWB_Mem2Reg)
		2'b00:
			wWB_WriteData <= wWB_DataFromMem; //LW
		2'b01:
			wWB_WriteData <= wWB_ResultALU; // TipoR e I
		2'b10:
			wWB_WriteData <= wWB_PC4;  // Syscall, JAL
		2'b11:
			wWB_WriteData <= {wWB_Imm,16'b0};  //LUI
	endcase
end

//============================= Sincronizacao =============================//

//ainda nao sei para que serve
always @(negedge iCLK)
begin
	if (wHU_ForwardPC4) begin
		Ra_Aux <= wMEM_PC4; 
	end
end 

// Escrita dos registradores de cada estagio
always @(posedge iCLK)
begin
	if(iRST) begin
		PC       <= 32'b0;
		RegIFID  <= 64'b0;	
		RegIDEX  <= 229'b0;
		RegEXMEM <= 106'b0;
		RegMEMWB <= 104'b0;
	end
	else begin
	
		// Estagio IF
		if (!wHU_BlockPC) begin
			PC <= wIF_iPC;
		end
		if (!wHU_BlockIFID) begin // Se estiver bloqueado, nao da flush
			if (wIFID_Flush) begin
				RegIFID[31:0]  <= 32'b0;
				RegIFID[63:32] <= 32'b0;
			end
			else begin
				RegIFID[31:0]  <= wIF_Instr; 
				RegIFID[63:32] <= wIF_PC4; //wPC + 32'h4;
			end
		end
		
		// Estagio ID
		/*
		RegIDEX[4:0]     <= wID_NumRd;
		RegIDEX[9:5]     <= wID_NumRt;
		RegIDEX[14:10]   <= wID_NumRs;
		RegIDEX[19:15]   <= wID_Shamt;
		RegIDEX[51:20]   <= wID_ConcatZeroImm;
		RegIDEX[83:52]   <= wID_ExtZeroImm;
		RegIDEX[115:84]  <= wID_ExtSigImm;
		RegIDEX[147:116] <= wID_Read2;
		RegIDEX[179:148] <= wID_Read1;
		RegIDEX[211:180] <= wID_PC4;
		
		if (wHU_FlushControl) begin
			RegIDEX[222:212] <= 11'b0;
		end
		else begin
			RegIDEX[213:212] <= wID_CALUOp;
			RegIDEX[215:214] <= wID_COrigALU;
			RegIDEX[217:216] <= wID_CRegDst;
			RegIDEX[218]     <= wID_CMemRead;
			RegIDEX[219]     <= wID_CMemWrite;
			RegIDEX[221:220] <= wID_CMem2Reg;
			RegIDEX[222]     <= wID_CRegWrite;
		end
		
		RegIDEX[228:223] <= wID_Opcode;
		RegIDEX[244:229] <= wID_Imm;
		*/
		
		if (wHU_FlushControl) begin
			RegIDEX[244:0] <= 245'b0;
		end
		else begin
		RegIDEX[4:0]     <= wID_NumRd;
		RegIDEX[9:5]     <= wID_NumRt;
		RegIDEX[14:10]   <= wID_NumRs;
		RegIDEX[19:15]   <= wID_Shamt;
		RegIDEX[51:20]   <= wID_ConcatZeroImm;
		RegIDEX[83:52]   <= wID_ExtZeroImm;
		RegIDEX[115:84]  <= wID_ExtSigImm;
		RegIDEX[147:116] <= wID_Read2;
		RegIDEX[179:148] <= wID_Read1;
		RegIDEX[211:180] <= wID_PC4;
		RegIDEX[213:212] <= wID_CALUOp;
		RegIDEX[215:214] <= wID_COrigALU;
		RegIDEX[217:216] <= wID_CRegDst;
		RegIDEX[218]     <= wID_CMemRead;
		RegIDEX[219]     <= wID_CMemWrite;
		RegIDEX[221:220] <= wID_CMem2Reg;
		RegIDEX[222]     <= wID_CRegWrite;
		RegIDEX[228:223] <= wID_Opcode;
		RegIDEX[244:229] <= wID_Imm;
		end
		
		// Estagio EX
		RegEXMEM[121:106] <= wEX_Imm;		
		RegEXMEM[105:103] <= wEX_WB;
		RegEXMEM[102:101] <= wEX_M;
		RegEXMEM[100:69]  <= wEX_PC4;
		RegEXMEM[68:37]   <= wEX_ResultALU;
		RegEXMEM[36:5]    <= wEX_ResultForwardB;
		RegEXMEM[4:0]     <= wEX_RegDestino;
		
		// Estagio MEM
		RegMEMWB[119:104] <= wMEM_Imm;
		RegMEMWB[103:101] <= wMEM_WB;
		RegMEMWB[100:69]  <= wMEM_PC4;
		RegMEMWB[68:37]   <= wMEM_DataFromMem;
		RegMEMWB[36:5]    <= wMEM_ResultALU;
		RegMEMWB[4:0]     <= wMEM_RegDestino;
		
	end
		
end

endmodule