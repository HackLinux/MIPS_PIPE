/*
 * Controle do Pipeline
 * DETALHE: ESTOU USANDO MAIS SINAIS DE 1 BIT para simplificar o entendimento, mas talvez seja mais rapido juntar alguns desses sinais como o OrigPc.. Ja mudei um pouco isso!
 * A marcacao (pra nada) indica que a escolha feita nao tem justificativa, tal escolha nao afeta em nada a execucao da instrucao

	oRegDst: registrador de destino
		00 - rt
		01 - rd
		10 - $ra
	oOrigALU:
		00 - 
		01 - 
		10 - 
		11 - 
	oMemparaReg
	oEscreveReg
	oLeMem: indica leitura da memoria (lw)
	oEscreveMem: indica escrita na memoria (sw)
	oOpALU
		00 - add
		01 - sub
		10 - campo funct
		11 - campo opcode
	oOrigPC
	oJump
	oBranch
	onBranch
	oJr: indica operacao jr
 
 */
module Control(
	iCLK, //OK
	iOp, //OK
	iFunct, //OK
	oRegDst, //OK
	oOrigALU, //OK
	oMemparaReg, //OK
	oEscreveReg, //OK
	oLeMem, //OK
	oEscreveMem, //OK
	oOpALU, //OK
	oOrigPC, //OK
	oJump, //OK
	oBranch, //OK
	onBranch, 
	oJr //OK
);

input wire iCLK;
input wire [5:0] iOp, iFunct;
output reg oEscreveReg, oLeMem, oEscreveMem, oJump, oBranch, onBranch, oJr;
output reg [1:0] oOpALU, oOrigALU, oRegDst, oMemparaReg;
output reg [2:0] oOrigPC;

initial
begin
  oRegDst     <= 2'b00;
  oOrigALU    <= 2'b00;
  oMemparaReg <= 2'b00;
  oEscreveReg <= 1'b0;
  oLeMem      <= 1'b0;
  oEscreveMem <= 1'b0;
  oOpALU      <= 2'b00;
  oOrigPC     <= 3'b000;
  oJump       <= 1'b0;
  oBranch     <= 1'b0;
  onBranch    <= 1'b0;
  oJr         <= 1'b0;
end

always @(iOp,iFunct)
begin
	case(iOp)
		OPCLW://OK
			begin
				oRegDst     <= 2'b00;//seleciona o Rt
				oOrigALU    <= 2'b01;//seleciona o imediato
				oMemparaReg <= 2'b00;//seleciona o resultado da MD
				oEscreveReg <= 1'b1;//ativa EscreveReg
				oLeMem      <= 1'b1;//ativa LeMem
				oEscreveMem <= 1'b0;//desativa EscreveMem
				oOrigPC     <= 3'b000;//seleciona PC+4
				oOpALU      <= 2'b00;//realiza ADD
				oJump       <= 1'b0;//desativa jump
				oBranch     <= 1'b0;//desativa branch
				onBranch    <= 1'b0; // desativa BNE
				oJr         <= 1'b0;//desativa o Jr
			end
			
		OPCSW://OK
			begin
				oRegDst     <= 2'b00;//seleciona o Rt
				oOrigALU    <= 2'b01;//seleciona o imediato
				oMemparaReg <= 2'b00;//seleciona o resultado da MD (pra nada)
				oEscreveReg <= 1'b0;//desativa EscreveReg
				oLeMem      <= 1'b0;//desativa LeMem
				oEscreveMem <= 1'b1;//ativa EscreveMem
				oOrigPC     <= 3'b000;//seleciona PC+4			 
				oOpALU      <= 2'b00;//realiza ADD
				oJump       <= 1'b0;//desativa jump
				oBranch     <= 1'b0;//desativa branch
				onBranch    <=1'b0; // desativa BNE
				oJr         <= 1'b0;//desativa o Jr
			end
			
		OPCBEQ://OK
			begin
				oRegDst     <= 2'b00;//seleciona o Rt (pra nada)
				oOrigALU    <= 2'b00;//seleciona o resultado do fowardB (pra nada)
				oMemparaReg <= 2'b00;//seleciona o resultado da MD (pra nada)
				oEscreveReg <= 1'b0;//desativa EscreveReg
				oLeMem      <= 1'b0;//desativa LeMem
				oEscreveMem <= 1'b0;//desativa EscreveMem
				oOrigPC     <= 3'b001;//seleciona o endere�o do branch
				oOpALU      <= 2'b01;//seleciona subtra��o (pra nada)
				oJump       <= 1'b0;//desativa jump
				oBranch     <= 1'b1;//ativa branch
				onBranch    <= 1'b0; // desativa BNE
				oJr         <= 1'b0;//desativa o Jr
			end

		OPCBNE://OK
			begin
				oRegDst     <= 2'b00;//seleciona o Rt (pra nada)
				oOrigALU    <= 2'b00;//seleciona o resultado do fowardB (pra nada)
				oMemparaReg <= 2'b00;//seleciona o resultado da MD (pra nada)
				oEscreveReg <= 1'b0;//desativa EscreveReg
				oLeMem      <= 1'b0;//desativa LeMem
				oEscreveMem <= 1'b0;//desativa EscreveMem
				oOrigPC     <= 3'b101;//seleciona o endere�o do branch
				oOpALU      <= 2'b01;//seleciona subtra��o (pra nada)
				oJump       <= 1'b0;//desativa jump
				oBranch     <= 1'b0;//desativa branch
				onBranch    <= 1'b1; // ativa BNE
				oJr         <= 1'b0;//desativa o Jr
			end
		
		OPCRFMT:    
			begin
				case (iFunct) 
				FUNJR://OK
					begin
						oRegDst     <= 2'b00;//seleciona o Rt (pra nada)
						oOrigALU    <= 2'b00;//seleciona o resultado do fowardB (pra nada)
						oMemparaReg <= 2'b00;//seleciona o resultado da MD (pra nada)
						oEscreveReg <= 1'b0;//desativa EscreveReg
						oLeMem      <= 1'b0;//desativa LeMem
						oEscreveMem <= 1'b0;//desativa EscreveMem
						oOrigPC     <= 3'b010;//seleciona resultado do MUX Jr
						oOpALU      <= 2'b10;//seleciona campo funct
						oJump       <= 1'b1;//ativa jump (mesmo que um jr n�o seja tipo J)
						oBranch     <= 1'b0;//desativa branch
						onBranch    <= 1'b0; // desativa BNE
						oJr         <= 1'b1;//ativa o Jr
					end
					
				FUNSYS:
					begin
						oRegDst     <= 2'b10; // salva em $ra o end de retorno
						oOrigALU    <= 2'b00;
						oMemparaReg <= 2'b10;
						oEscreveReg <= 1'b1;
						oLeMem      <= 1'b0;
						oEscreveMem <= 1'b0;
						oOrigPC     <= 3'b100;  // Endereco do .ktext
						oOpALU      <= 2'b10;
						oJump       <= 1'b1;//ativa jump (mesmo que um syscall n�o seja tipo J)
						oBranch     <= 1'b0;//desativa branch
						onBranch    <= 1'b0; // desativa BNE
						oJr         <= 1'b0;//desativa o Jr
					end
				//TIPO R
				default://OK
					begin
						oRegDst     <= 2'b01;//seleciona o Rd
						oOrigALU    <= 2'b00;//seleciona o resultado do fowardB
						oMemparaReg <= 2'b01;//seleciona o resultado da ALU
						oEscreveReg <= 1'b1;//ativa EscreveReg
						oLeMem      <= 1'b0;//desativa LeMem
						oEscreveMem <= 1'b0;//desativa EscreveMem
						oOrigPC     <= 3'b000;//seleciona PC+4						 
						oOpALU      <= 2'b10;//funct determina a opera��o da ALU
						oJump       <= 1'b0;//desativa jump
						oBranch     <= 1'b0;//desativa branch
						onBranch    <= 1'b0; // desativa BNE
						oJr         <= 1'b0;//desativa o Jr
					end
				endcase
			end

		OPCJMP://OK
			begin
				oRegDst     <= 2'b00;//seleciona o Rt (pra nada)
				oOrigALU    <= 2'b00;//seleciona o resultado do fowardB (pra nada)
				oMemparaReg <= 2'b00;//seleciona o resultado da MD (pra nada)
				oEscreveReg <= 1'b0;//desativa EscreveReg
				oLeMem      <= 1'b0;//desativa LeMem
				oEscreveMem <= 1'b0;//desativa EscreveMem
				oOrigPC     <= 3'b010;//seleciona resultado do MUX Jr
				oOpALU      <= 2'b00;//seleciona ADD (pra nada)
				oJump       <= 1'b1;//ativa jump
				oBranch     <= 1'b0;//desativa branch
				onBranch    <= 1'b0; // desativa BNE
				oJr         <= 1'b0;//desativa o Jr
			end

		OPCADDI,
		OPCADDIU://OK
			begin
				oRegDst     <= 2'b00;//seleciona o Rt
				oOrigALU    <= 2'b01;//seleciona o imediato com extens�o de sinal
				oMemparaReg <= 2'b01;//seleciona o resultado da ALU
				oEscreveReg <= 1'b1;//ativa EscreveReg
				oLeMem      <= 1'b0;//desativa LeMem
				oEscreveMem <= 1'b0;//desativa EscreveMem
				oOrigPC     <= 3'b000;//seleciona PC+4		 
				oOpALU      <= 2'b11;//opcode determina opera��o da ALU
				oJump       <= 1'b0;//desativa jump
				oBranch     <= 1'b0;//desativa branch
				onBranch    <= 1'b0; // desativa BNE
				oJr         <= 1'b0;//desativa o Jr
			end
			
		OPCANDI://OK
			begin
				oRegDst     <= 2'b00;//seleciona o Rt
				oOrigALU    <= 2'b01;//seleciona o imediato com extens�o de sinal
				oMemparaReg <= 2'b01;//seleciona o resultado da ALU
				oEscreveReg <= 1'b1;//ativa EscreveReg
				oLeMem      <= 1'b0;//desativa LeMem
				oEscreveMem <= 1'b0;//desativa EscreveMem
				oOrigPC     <= 3'b000;//seleciona PC+4		 
				oOpALU      <= 2'b11;//opcode determina opera��o da ALU
				oJump       <= 1'b0;//desxativa jump
				oBranch     <= 1'b0;//desativa branch
				onBranch    <= 1'b0; // desativa BNE
				oJr         <= 1'b0;//desativa o Jr
			end			
			
		OPCXORI://OK
			begin
				oRegDst     <= 2'b00;//seleciona o Rt
				oOrigALU    <= 2'b01;//seleciona o imediato com extens�o de sinal
				oMemparaReg <= 2'b01;//seleciona o resultado da ALU
				oEscreveReg <= 1'b1;//ativa EscreveReg
				oLeMem      <= 1'b0;//desativa LeMem
				oEscreveMem <= 1'b0;//desativa EscreveMem
				oOrigPC     <= 3'b000;//seleciona PC+4		 
				oOpALU      <= 2'b11;//opcode determina opera��o da ALU
				oJump       <= 1'b0;//desxativa jump
				oBranch     <= 1'b0;//desativa branch
				onBranch    <= 1'b0; // desativa BNE 
				oJr         <= 1'b0;//desativa o Jr
			end
			
		OPCORI://OK
			begin
				oRegDst     <= 2'b00;//seleciona o Rt
				oOrigALU    <= 2'b10;//seleciona o imediato com extens�o com zeros
				oMemparaReg <= 2'b01;//seleciona o resultado da ALU
				oEscreveReg <= 1'b1;//ativa EscreveReg
				oLeMem      <= 1'b0;//desativa LeMem
				oEscreveMem <= 1'b0;//desativa EscreveMem
				oOrigPC     <= 3'b000;//seleciona PC+4		 
				oOpALU      <= 2'b11;//opcode determina opera��o da ALU
				oJump       <= 1'b0;//desativa jump
				oBranch     <= 1'b0;//desativa branch
				onBranch    <= 1'b0; // desativa BNE		
				oJr         <= 1'b0;//desativa o Jr		
			end					
			
		OPCJAL://OK
			begin
				oRegDst     <= 2'b10;//Seleciona 31 (ra)
				oOrigALU    <= 2'b00;//seleciona o resultado do fowardB (pra nada)
				oMemparaReg <= 2'b10;//seleciona o PC+4
				oEscreveReg <= 1'b1;//ativa EscreveReg
				oLeMem      <= 1'b0;//desativa LeMem
				oEscreveMem <= 1'b0;//desativa EscreveMem
				oOrigPC     <= 3'b010;//seleciona o resultado do MUX Jr	 
				oOpALU      <= 2'b00;//seleciona ADD (pra nada)
				oJump       <= 1'b1;//ativa jump
				oBranch     <= 1'b0;//desativa branch
				onBranch    <=1'b0; // desativa BNE	
				oJr         <= 1'b0;//desativa o Jr
			end
			
		OPCLUI://OK
			begin
				oRegDst     <= 2'b00;//Seleciona o Rt
				oOrigALU    <= 2'b11;//seleciona o imediato concatenado com 16 zeros
				oMemparaReg <= 2'b11;//seleciona o Imediato com zeros
				oEscreveReg <= 1'b1;//ativa EscreveReg
				oLeMem      <= 1'b0;//desativa LeMem
				oEscreveMem <= 1'b0;//desativa EscreveMem
				oOrigPC     <= 3'b000;//seleciona PC+4	 
				oOpALU      <= 2'b00;//seleciona ADD (pra nada)
				oJump       <= 1'b0;//desativa jump
				oBranch     <= 1'b0;//desativa branch
				onBranch    <= 1'b0; // desativa BNE	
				oJr         <= 1'b0;//desativa o Jr
			end	
			
		default: // Instrucao Nao reconhecida
			begin
				oRegDst     <= 2'b00;
				oOrigALU    <= 2'b00;
				oMemparaReg <= 2'b00;
				oEscreveReg <= 1'b0;
				oLeMem      <= 1'b0;
				oEscreveMem <= 1'b0;
				oOrigPC     <= 3'b111;// looop inf				 
				oOpALU      <= 2'b00; 
				oJump       <= 1'b0;//desativa jump
				oBranch     <= 1'b0;//desativa branch
				onBranch    <=1'b0; // desativa BNE	
				oJr         <= 1'b0;//desativa o Jr
			end
			
	endcase
end

endmodule
