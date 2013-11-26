/*
 TopDE
 
 Top Level para processador MIPS PIPELINE v1 baseado no processador 

Desenvolvido por  
André Figueira Lourenço 	09/89525
José Chaves Júnior 	08/40122
Hugo Marello 	10/29444
em 2010/2 na disciplina OAC

Melhorado por
Emerson Grzeidak 09/93514
Gabriel Calache Cozendey 09/47946
Glauco Medeiros Volpe 10/25091
Luiz Henrique Dias Navarro 10/00748
Waldez Azevedo Gomes Junior 10/08617
em 2011/1 na disciplina OAC

Top Level para processador MIPS UNICICLO v2 baseado no processador desenvolvido por 
Antonio Martino Neto 09/89886
Bruno de Matos Bertasso 08/25590
Carolina S. R. de Oliveira 07/45006
Herman Ferreira M. de Asevedo 09/96319
Renata Cristina 09/0130600
em 2012/1 na disciplina OAC

Top Level para processador MIPS UNICICLO v3 baseado no processador desenvolvido por 
Antonio Martino Neto 09/89886
em 2013/1 na disciplina TG1

 Adaptado para a placa de desenvolvimento DE2-70.
 Prof. Marcus Vinicius Lamar   2013/1
 */
 
module TopDE (iCLK_50, iCLK_28, 
			  iKEY, 
			  oHEX0_D, oHEX0_DP, 
			  oHEX1_D, oHEX1_DP, 
			  oHEX2_D, oHEX2_DP,
			  oHEX3_D, oHEX3_DP,
			  oHEX4_D, oHEX4_DP,
			  oHEX5_D, oHEX5_DP,
			  oHEX6_D, oHEX6_DP,
			  oHEX7_D, oHEX7_DP,
			  oLEDG, 
			  oLEDR, 
			  iSW,
			  oVGA_CLOCK, oVGA_HS, oVGA_VS, oVGA_BLANK_N, oVGA_SYNC_N,
			  oVGA_R, oVGA_G, oVGA_B,
			  GPIO_0, 
			  oLCD_ON, oLCD_BLON, LCD_D, oLCD_RW, oLCD_EN, oLCD_RS,
			  oTD1_RESET_N, 
			  I2C_SDAT, oI2C_SCLK, 
			  AUD_ADCLRCK, iAUD_ADCDAT, AUD_DACLRCK, oAUD_DACDAT, AUD_BCLK, oAUD_XCK,
			  PS2_KBCLK, PS2_KBDAT,
			  oOUTPUT, oCLK, oPC, owInstr);

/* I/O type definition */
input iCLK_50, iCLK_28;
input [3:0] iKEY;
input [17:0] iSW;
output [8:0] oLEDG;
output [17:0] oLEDR;
output [6:0] oHEX0_D, oHEX1_D, oHEX2_D, oHEX3_D, oHEX4_D, oHEX5_D, oHEX6_D, oHEX7_D;
output oHEX0_DP, oHEX1_DP, oHEX2_DP, oHEX3_DP, oHEX4_DP, oHEX5_DP, oHEX6_DP, oHEX7_DP;

// GPIO_0
input [31:0] GPIO_0 ;

//VGA interface
output oVGA_CLOCK, oVGA_HS, oVGA_VS, oVGA_BLANK_N, oVGA_SYNC_N;
output [9:0] oVGA_R, oVGA_G, oVGA_B;

// TV Decoder
output oTD1_RESET_N; // TV Decoder Reset

// I2C
inout  I2C_SDAT; 	// I2C Data
output oI2C_SCLK; 	// I2C Clock

// Audio CODEC
inout  AUD_ADCLRCK; 	// Audio CODEC ADC LR Clock
input  iAUD_ADCDAT; 	// Audio CODEC ADC Data
inout  AUD_DACLRCK; 	// Audio CODEC DAC LR Clock
output oAUD_DACDAT;  	// Audio CODEC DAC Data
inout  AUD_BCLK;    	// Audio CODEC Bit-Stream Clock
output oAUD_XCK;     	// Audio CODEC Chip Clock

// PS2 Keyborad
inout PS2_KBCLK;
inout PS2_KBDAT;

//	Modulo LCD 16X2
inout	[7:0]	LCD_D;			//	LCD Data bus 8 bits
output			oLCD_ON;		//	LCD Power ON/OFF
output			oLCD_BLON;		//	LCD Back Light ON/OFF
output			oLCD_RW;		//	LCD Read/Write Select, 0 = Write, 1 = Read
output			oLCD_EN;		//	LCD Enable
output			oLCD_RS;		//	LCD Command/Data Select, 0 = Command, 1 = Data

//para simulacao
output [31:0] oOUTPUT,oPC,owInstr;
output oCLK;

assign oOUTPUT = wOutput;
assign oCLK = CLK;
assign oPC = PC;
assign owInstr=wInstr;

/* Local Clock signals */
reg CLKManual, CLKAutoSlow, CLKSelectAuto, CLKSelectFast, CLKAutoFast;
wire CLK, clock50_ctrl;
reg [3:0] CLKCount2;
reg [25:0] CLKCount;

wire [7:0] wcount;
wire [25:0] counts;
wire [3:0] countf;
assign counts = 26'h200000*wcount;
assign countf = wcount;
assign wcount = iSW[7:0];  // usado para o divisor de frequencia 

 
 // Reset Sincrono com o Clock
wire Reset;
always @(posedge CLK)
	Reset <= ~iKEY[0];
 
 
/* Local wires */
wire [31:0] PC, wRegDisp, wRegA0, wMemAddress, wMemWriteData, extOpcode, extFunct, wOutput, wInstr, wDebug, wMemReadVGA;
wire [1:0] ALUOp,OrigALU, RegDst, Mem2Reg;
wire [2:0] OrigPC;
wire MemWrite, MemRead, RegWrite;
wire [4:0] wRegDispSelect;
wire [5:0] wOpcode, wFunct; 
 
/* LEDs sinais de controle */
assign oLEDG[7:0] =	PC[9:2];
assign oLEDG[8] =	CLK;
assign oLEDR[1:0] =	Mem2Reg;
assign oLEDR[3:2] =	OrigALU;
assign oLEDR[5:4] =	RegDst;
assign oLEDR[8:6] =	OrigPC;
assign oLEDR[10:9] =	ALUOp;

assign oLEDR[11] = RegWrite;
assign oLEDR[12] = MemWrite;
assign oLEDR[13] = MemRead;

assign oLEDR[17:14] = 4'b0; // Nao utilizados atualmente
		
/* para apresentacao nos displays */
assign extOpcode = {26'b0,wOpcode};
assign extFunct = {26'b0,wFunct};

/* 7 segment display register content selection */
assign wRegDispSelect =	iSW[17:13];


/* $a0 initial content, with signal extention */
assign wRegA0 = {{24{iSW[7]}},iSW[7:0]};


assign wOutput	= iSW[12] ?
				(iSW[17] ?
					PC :
					(iSW[16] ?
						wInstr :
						(iSW[15] ?
							extOpcode :
							(iSW[14] ?
								extFunct :
								(iSW[13]?
								wMemWriteData:
								//wDebug:
								wDebug)
								//wMemAddress)
								//32'h08888880)
							)
						)
					)
				) :
				wRegDisp;
				

/* Clocks */
/* Clocks escolha */
always @(posedge iCLK_50) //clock50_ctrl) //
	CLK <= CLKSelectAuto?(CLKSelectFast?CLKAutoFast:CLKAutoSlow):CLKManual;
	

/* Clock inicializacao */
initial
begin
	CLKManual	<= 1'b0;
	CLKAutoSlow	<= 1'b0;
	CLKAutoFast	<= 1'b0;
	CLKSelectAuto<= 1'b0;
	CLKSelectFast<= 1'b0;
	CLKCount2<=4'b0;
	CLKCount<=26'b0;
end

always @(posedge iKEY[3])
begin
	CLKManual <= ~CLKManual;       // Manual
end

always @(posedge iKEY[2])
begin
	CLKSelectAuto <= ~CLKSelectAuto;
end

always @(posedge iKEY[1])
begin
	CLKSelectFast <= ~CLKSelectFast;
end

always @(posedge clock50_ctrl)
begin

	if (CLKCount == counts) //Clock Slow
	begin
		CLKAutoSlow <= ~CLKAutoSlow;
		CLKCount <= 26'b0;
	end
	else
		CLKCount <= CLKCount + 1'b1;
	
	if (CLKCount2 == countf) //Clock Fast
	begin
		CLKAutoFast <= ~CLKAutoFast;
		CLKCount2 <= 4'b0;
	end
	else
		CLKCount2 <= CLKCount2 + 1'b1;
	
end
	


/* Mono estavel 10 segundos */
mono Mono1 (iCLK_50,~iSW[10],clock50_ctrl,Reset);


/* MIPS Processor instantiation */

MIPS2 Processor0 (
	.iCLK(CLK),
	.iCLK50(iCLK_50),
	.iRST(Reset),
	.iwRegA0(wRegA0),
	.iwCInputA0En(iSW[8]),
	.woPC(PC),
	.woCALUOp(ALUOp),
	.woCMemWrite(MemWrite), // NAO VEM DA CONTROL UNIT, VEM DO ESTAGIO MEM!!! D:<
	.woCMemRead(MemRead),
	.woCRegWrite(RegWrite),
	.woCRegDst(RegDst),
	.iwRegDispSelect(wRegDispSelect),
	.woRegDisp(wRegDisp),
	.woOpcode(wOpcode),
	.woFunct(wFunct),
	.woInstr(wInstr),
	.woCOrigALU(OrigALU),
	.woCMem2Reg(Mem2Reg),
	.woCOrigPC(OrigPC),
	.woDebug(wDebug),
	.woMemAddress(wMemAddress),
	.woMemWriteData(wMemWriteData),
	.iwAudioCodecData(wAudioCodecData)
	);

	
/* 7 segment display instantiations */

assign oHEX0_DP = 1'b1;
assign oHEX1_DP = 1'b1;
assign oHEX2_DP = 1'b1;
assign oHEX3_DP = 1'b1;
assign oHEX4_DP = 1'b1;
assign oHEX5_DP = 1'b1;
assign oHEX6_DP = 1'b1;
assign oHEX7_DP = 1'b1;

Decoder7 Dec0 (
	.In(wOutput[3:0]),
	.Out(oHEX0_D)
	);

Decoder7 Dec1 (
	.In(wOutput[7:4]),
	.Out(oHEX1_D)
	);

Decoder7 Dec2 (
	.In(wOutput[11:8]),
	.Out(oHEX2_D)
	);

Decoder7 Dec3 (
	.In(wOutput[15:12]),
	.Out(oHEX3_D)
	);

Decoder7 Dec4 (
	.In(wOutput[19:16]),
	.Out(oHEX4_D)
	);

Decoder7 Dec5 (
	.In(wOutput[23:20]),
	.Out(oHEX5_D)
	);

Decoder7 Dec6 (
	.In(wOutput[27:24]),
	.Out(oHEX6_D)
	);

Decoder7 Dec7 (
	.In(wOutput[31:28]),
	.Out(oHEX7_D)
	);


// VGA Interface

parameter VGAADDRESS = 32'h80000000; //em bytes	
	
VgaAdapterInterface VGAAI0 (
	.iRST(~Reset),
	.iCLK_50(iCLK_50),
	.iCLK(CLK),
	.iMemWrite(MemWrite),
	.iwMemAddress(wMemAddress),
	.iwMemWriteData(wMemWriteData),
	.oMemReadData(wMemReadVGA),
	.oVGA_R(oVGA_R),
	.oVGA_G(oVGA_G),
	.oVGA_B(oVGA_B),
	.oVGA_HS(oVGA_HS),
	.oVGA_VS(oVGA_VS),
	.oVGA_BLANK(oVGA_BLANK_N),
	.oVGA_SYNC(oVGA_SYNC_N),
	.oVGA_CLK(oVGA_CLOCK));
	
	
	//  Audio In/Out Interface

// reset delay gives some time for peripherals to initialize
wire DLY_RST;
Reset_Delay r0(	.iCLK(iCLK_50),.oRESET(DLY_RST) );

assign	oTD1_RESET_N = 1'b1;  // Enable 27 MHz

wire AUD_CTRL_CLK;

VGA_Audio_PLL 	p1 (	
	.areset(~DLY_RST),
	.inclk0(iCLK_28),
	.c0(),
	.c1(AUD_CTRL_CLK),
	.c2()
);

I2C_AV_Config u3(	
//	Host Side
  .iCLK(iCLK_50),
  .iRST_N(~Reset),
//	I2C Side
  .I2C_SCLK(oI2C_SCLK),
  .I2C_SDAT(I2C_SDAT)	
);

assign	AUD_ADCLRCK	=	AUD_DACLRCK;
assign	oAUD_XCK	=	AUD_CTRL_CLK;

audio_clock u4(	
//	Audio Side
   .oAUD_BCK(AUD_BCLK),
   .oAUD_LRCK(AUD_DACLRCK),
//	Control Signals
   .iCLK_18_4(AUD_CTRL_CLK),
   .iRST_N(DLY_RST)	
);


 /* CODEC AUDIO */

audio_converter u5(
	// Audio side
	.AUD_BCK(AUD_BCLK),       // Audio bit clock
	.AUD_LRCK(AUD_DACLRCK), // left-right clock
	.AUD_ADCDAT(iAUD_ADCDAT),
	.AUD_DATA(oAUD_DACDAT),
	// Controller side
	.iRST_N(DLY_RST),  // reset
	.AUD_outL(audio_outL),
	.AUD_outR(audio_outR),

	.AUD_inL(audio_inL),
	.AUD_inR(audio_inR)
);

wire [15:0] audio_inL, audio_inR;
reg [15:0] audio_outL,audio_outR;
reg [31:0] wAudioCodecData;

reg [31:0] waudio_inL ,waudio_inR;
reg [31:0] waudio_outL, waudio_outR;
reg [31:0] Ctrl1,Ctrl2;
 
 /* Endereco dos registradores do CODEC na memoria*/
parameter 	INRDATA=32'h40000000,  INLDATA=32'h40000004,
			OUTRDATA=32'h40000008, OUTLDATA=32'h4000000c,
			CTRL1=32'h40000010,    CTRL2=32'h40000014;
initial
	begin
		waudio_inL<=32'b0;
		waudio_inR<=32'b0;
		waudio_outL<=32'b0;
		waudio_outR<=32'b0;
		Ctrl1<=32'b0;
		Ctrl2<=32'b0;
	end

always @(negedge AUD_DACLRCK)
	begin
		if(Ctrl2[0]==0)
			begin
				waudio_inR<= {16'b0,audio_inR};
				audio_outR = waudio_outR[15:0];
				Ctrl1[0]<=1'b1;
			end
		else
			Ctrl1[0]<=1'b0;
	end
	
always @(posedge AUD_DACLRCK)
	begin
		if(Ctrl2[1]==0)
			begin
				waudio_inL = {16'b0,audio_inL};
				audio_outL = waudio_outL[15:0];
				Ctrl1[1]<=1'b1;
			end
		else
			Ctrl1[1]<=1'b0;
	end


always @(posedge CLK)			
		if(MemWrite) //Escrita no dispositivo de ï¿½udio
			begin
				case (wMemAddress)
					OUTRDATA: waudio_outR <= wMemWriteData;
					OUTLDATA: waudio_outL <= wMemWriteData;  
					CTRL2:    Ctrl2 <= wMemWriteData;
				endcase
			end	




// Teclado PS2

wire [7:0] PS2scan_code;
reg [7:0] PS2history[1:8]; // buffer de 8 bytes
wire PS2read, PS2scan_ready;


 /* Enderecos na memï¿½ria do Buffer de leitura do Teclado */
parameter 	BUFFER0=32'h40000020,  
			BUFFER1=32'h40000024;

oneshot pulser(
   .pulse_out(PS2read),
   .trigger_in(PS2scan_ready),
   .clk(iCLK_50)
);

keyboard kbd(
  .keyboard_clk(PS2_KBCLK),
  .keyboard_data(PS2_KBDAT),
  .clock50(iCLK_50),
  .reset(Reset),
  .read(PS2read),
  .scan_ready(PS2scan_ready),
  .scan_code(PS2scan_code)
);

always @(posedge PS2scan_ready, posedge Reset)
begin
	if(Reset)
		begin
		PS2history[8] <= 0;
		PS2history[7] <= 0;
		PS2history[6] <= 0;
		PS2history[5] <= 0;
		PS2history[4] <= 0;
		PS2history[3] <= 0;
		PS2history[2] <= 0;
		PS2history[1] <= 0;
		end
	else
		begin
		PS2history[8] = PS2history[7];
		PS2history[7] = PS2history[6];
		PS2history[6] = PS2history[5];
		PS2history[5] = PS2history[4];
		PS2history[4] = PS2history[3];
		PS2history[3] = PS2history[2];
		PS2history[2] = PS2history[1];
		PS2history[1] = PS2scan_code;
		end
end


// LCD

parameter LIMPA  = 32'h70000020;  //Endereco de limpar o display
parameter LINHA1 = 32'h70000000;
parameter LINHA2 = 32'h70000010;

/*	LCD ON */
assign	oLCD_ON		=	1'b1;
assign	oLCD_BLON	=	1'b1;

wire [7:0] oLeituraLCD;
	
LCDStateMachine LCDSM0 (
	.iCLK(iCLK_50),
	.iRST(Reset),
	.LCD_DATA(LCD_D),
	.LCD_RW(oLCD_RW),
	.LCD_EN(oLCD_EN),
	.LCD_RS(oLCD_RS),
	.iMemAddress(wMemAddress),
	.iMemWriteData(wMemWriteData),
	.iMemWrite(MemWrite),
	.oLeitura(oLeituraLCD)
	);



/* acesso para leitura dos endereï¿½os da MMIO
 a gravaï¿½ï¿½o eh feita no proprio dispositivo acima */


always @(*)
begin
		if(MemRead)  //Leitura dos dispositivos
			if(wMemAddress>=VGAADDRESS)
				//VGA
				wAudioCodecData <= wMemReadVGA;
			else
			if(wMemAddress>=LINHA1 && wMemAddress <LIMPA)
				//LCD
				wAudioCodecData<={24'b0,oLeituraLCD};
			else
			begin
				case (wMemAddress)
					//Audio
					INRDATA:  wAudioCodecData <= waudio_inR;
					OUTRDATA: wAudioCodecData <= waudio_outR;
					INLDATA:  wAudioCodecData <= waudio_inL;
					OUTLDATA: wAudioCodecData <= waudio_outL;
					CTRL1:    wAudioCodecData <= Ctrl1;
					CTRL2:    wAudioCodecData <= Ctrl2;
					//PS2
					BUFFER0:  wAudioCodecData <= {PS2history[4],PS2history[3],PS2history[2],PS2history[1]};
					BUFFER1:  wAudioCodecData <= {PS2history[8],PS2history[7],PS2history[6],PS2history[5]};
					default:  wAudioCodecData <= 32'b0;  
				endcase
			end
end

endmodule
