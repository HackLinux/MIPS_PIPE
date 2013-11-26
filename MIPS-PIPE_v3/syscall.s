.kdata
.word 0x00000000, 0x00000000, 0x10101010, 0x00100010, 0x00002828, 0x00000000, 0x28FE2828, 0x002828FE, 0x38503C10, 0x00107814, 0x10686400, 0x00004C2C, 0x28102818, 0x003A4446, 0x00001010, 0x00000000, 0x20201008, 0x00081020, 0x08081020, 0x00201008, 0x38549210, 0x00109254, 0xFE101010, 0x00101010, 0x00000000, 0x10081818, 0xFE000000, 0x00000000, 0x00000000, 0x18180000, 0x10080402, 0x00804020, 0x54444438, 0x00384444, 0x10103010, 0x00381010, 0x08044438, 0x007C2010, 0x18044438, 0x00384404, 0x7C482818, 0x001C0808, 0x7840407C, 0x00384404, 0x78404438, 0x00384444, 0x1008047C, 0x00202020, 0x38444438, 0x00384444, 0x3C444438, 0x00384404, 0x00181800, 0x00001818, 0x00181800, 0x10081818, 0x20100804, 0x00040810, 0x00FE0000, 0x000000FE, 0x04081020, 0x00201008, 0x08044438, 0x00100010, 0x545C4438, 0x0038405C, 0x7C444438, 0x00444444, 0x78444478, 0x00784444, 0x40404438, 0x00384440, 0x44444478, 0x00784444, 0x7840407C, 0x007C4040, 0x7C40407C, 0x00404040, 0x5C404438, 0x00384444, 0x7C444444, 0x00444444, 0x10101038, 0x00381010, 0x0808081C, 0x00304848, 0x70484444, 0x00444448, 0x20202020, 0x003C2020, 0x92AAC682, 0x00828282, 0x54546444, 0x0044444C, 0x44444438, 0x00384444, 0x38242438, 0x00202020, 0x44444438, 0x0C384444, 0x78444478, 0x00444850, 0x38404438, 0x00384404, 0x1010107C, 0x00101010, 0x44444444, 0x00384444, 0x28444444, 0x00101028, 0x54828282, 0x00282854, 0x10284444, 0x00444428, 0x10284444, 0x00101010, 0x1008047C, 0x007C4020, 0x20202038, 0x00382020, 0x10204080, 0x00020408, 0x08080838, 0x00380808, 0x00442810, 0x00000000, 0x00000000, 0xFE000000, 0x00000810, 0x00000000, 0x3C043800, 0x003A4444, 0x24382020, 0x00582424, 0x201C0000, 0x001C2020, 0x48380808, 0x00344848, 0x44380000, 0x0038407C, 0x70202418, 0x00202020, 0x443A0000, 0x38043C44, 0x64584040, 0x00444444, 0x10001000, 0x00101010, 0x10001000, 0x60101010, 0x28242020, 0x00242830, 0x08080818, 0x00080808, 0x49B60000, 0x00414149, 0x24580000, 0x00242424, 0x44380000, 0x00384444, 0x24580000, 0x20203824, 0x48340000, 0x08083848, 0x302C0000, 0x00202020, 0x201C0000, 0x00380418, 0x10381000, 0x00101010, 0x48480000, 0x00344848, 0x44440000, 0x00102844, 0x82820000, 0x0044AA92, 0x28440000, 0x00442810, 0x24240000, 0x38041C24, 0x043C0000, 0x003C1008, 0x2010100C, 0x000C1010, 0x10101010, 0x00101010, 0x04080830, 0x00300808, 0x92600000, 0x0000000C, 0x243C1818, 0xA55A7E3C, 0x99FF5A81, 0x99663CFF, 0x10280000, 0x00000028, 0x10081020, 0x00081020
	   
	   	   
.ktext

# Usar off set de endereço 0x00000000 e offset para jumps de 0x00004000 para exportar MIF com CodeMemory de 4096 words;

funcSyscall: addi $sp, $sp, -44   			# Salva $ra e $ts na pilha
			 sw $ra, 0($sp)
			 sw $t0, 4($sp)
			 sw $t1, 8($sp)
			 sw $t2, 12($sp)
			 sw $t3, 16($sp)
			 sw $t4, 20($sp)
			 sw $t5, 24($sp)
			 sw $t6, 28($sp)
			 sw $t7, 32($sp)
			 sw $t8, 36($sp)
			 sw $t9, 40($sp)


			addi $t0, $zero, 1				# sycall 1 = print int
			beq $t0, $v0, goToPrintInt

			addi $t0, $zero, 4				# syscall 4 = print string
			beq $t0, $v0, goToPrintString

			addi $t0, $zero, 11				# syscall 11 = print char
			beq $t0, $v0, goToPrintChar

endSyscall: lw $ra, 0($sp)					# syscall 30 = time     syscall 32 = sleep    syscall 41 = randon
			 lw $t0, 4($sp)
			 lw $t1, 8($sp)
			 lw $t2, 12($sp)
			 lw $t3, 16($sp)
			 lw $t4, 20($sp)
			 lw $t5, 24($sp)
			 lw $t6, 28($sp)
			 lw $t7, 32($sp)
			 lw $t8, 36($sp)
			 lw $t9, 40($sp)
			addi $sp, $sp, 44				# avaliam $v0 por hardware
			jr $ra

goToPrintInt: jal printInt					# chama printInt
			j endSyscall

goToPrintString: jal printString			# chama printString
			j endSyscall			

goToPrintChar: jal printChar				#chama printChar
			j endSyscall



############################
#  PrintInt				   #
#  $a0	=	valor inteiro  #
#  $a1	=	x			   #
#  $a2	=	y			   #
############################

printInt: addi $sp, $sp, -4   				# salva $ra
		sw $ra, 0($sp)

		beq $a0, $zero, printZero	
		j printNotZero						# chama printNotZero

printZero: addi $a0, $a0, 48				# Imprime 0
		jal printChar

		lw $ra, 0($sp)						#retorna
		addi $sp, $sp, 4
		jr $ra

printNotZero: add $t0, $zero, $a0			# $t0 contem o valor do inteiro a ser impresso
		addi $t1, $zero, 10					# $t1 eh uma constante 10
		slt $t9, $t0, $zero					# $t0 < 0 ?
		beq $t9, $zero, PrintIntContinue	# verifica se o valor eh negativo. 

		addi $a0, $zero, 45					# Negativo, imprime um '-' na tela

		addi $sp, $sp, -12
		sw $t0, 0($sp)						# salva regs
		sw $t1, 4($sp)
		sw $ra, 8($sp)

		jal printChar						# imprime ASCII 45

		lw $ra, 8($sp)						# recupera regs
		lw $t1, 4($sp)
		lw $t0, 0($sp)
		addi $sp, $sp, 12

		sub $t0, $zero, $t0					# Torna $t0 positivo
		addi $a1, $a1, 8					# incrementa a coluna
		add $t3, $zero, $zero				# $t3=0

PrintIntContinue: beq $t0, $zero, PrintIntPop		# se $t0 é zero, nao há mais digitos para imprimir

		div $t0, $t1					# divide o valor por 10
		mflo $t0						# $t0 contem o valor dividido por 10
		mfhi $t2						# $t2 contem o ultimo digito a ser impresso

		addi $sp, $sp, -4
		sw $t2, 0($sp)					# empilha $t2

		addi $t3, $t3, 1				# conta quantos elementos (digitos) estão na pilha
		j PrintIntContinue				# volta para ser dividido e empilhado de novo

PrintIntPop: beq $t3, $zero, endPrintInt	# ultimo digito endPrintInt

		lw $a0, 0($sp)					# le valor da pilha e coloca em $a0
		addi $sp, $sp, 4

		addi $a0, $a0, 48				# código ASCII do dígito = numero + 48

		addi $sp, $sp, -8				# salva regs
		sw $t3, 0($sp)
		sw $ra, 4($sp)

		jal printChar					# imprime digito

		lw $ra, 4($sp)					# recupera regs
		lw $t3, 0($sp)
		addi $sp, $sp, 8

		addi $a1, $a1, 8				# incrementa a coluna
		addi $t3, $t3, -1				# decrementa contador
		j PrintIntPop					# volta

endPrintInt: lw $ra, 0($sp)				# recupera $ra
		addi $sp, $sp, 4
		jr $ra							# fim printInt



#################################
#  PrintSring			  		#
#  $a0	=	endereco da string  #
#  $a1	=	x			   		#
#  $a2	=	y			   		#
#################################

printString: addi $sp, $sp, -4			# salva $ra
		sw $ra, 0($sp)

		move $t0, $a0					# $t0=endereco da string

ForPrintString:	lw $a0, 0($t0)			# le em $a0 o caracter a ser impresso
		beq $a0, $zero, EndForPrintString	# string ASCIIZ termina com NULL

		addi $sp, $sp, -4				# salva $t0
		sw $t0, 0($sp)

		jal printChar					# imprime char

		lw $t0, 0($sp)					# recupera $t0					
		addi $sp, $sp, 4

		addi $a1, $a1, 8				# incrementa a coluna
		addi $t0, $t0, 4				# cada caractere esta armazenado em uma word na memoria, logo incrementa 4 noo endereco
		j ForPrintString				# loop

EndForPrintString: lw $ra, 0($sp)		# recupera $ra
		addi $sp, $sp, 4
		jr $ra							# fim printString



#################################
#  PrintChar			  		#
#  $a0	=	char(ASCII)		    #
#  $a1	=	x			   		#
#  $a2	=	y			   		#

#$t0	=	i
#$t1	=	j
#$t2	=	endereco do char na memoria
#$t3	=	metade do char (2ï¿½ e depois 1ï¿½)
#$t4	=	endereco para impressao
#$t5	=	background color
#$t6	=	foreground color
#$t7	=	2


#################################


printChar: li $t7, 2					# iniciando $t7=2
		andi $t5,$a3,0x07				# cor fundo
		andi $t6,$a3,0x38				# cor frente
		srl $t6,$t6,3
#		li $t5, 0						# cor preta 000 = RGB
#		li $t6, 2						# cor verde 010 = RGB

		addi $t4, $a2, 0				# t4 = y
		sll $t4, $t4, 8					# t4 = 256(y)
		add $t4, $t4, $a1				# t4 = 256(y) + x
		addi $t4, $t4, 7				# t4 = 256(y) + (x+7)
		li $t8, 0x80000000				# Endereco de inicio da memoria VGA
		add $t4, $t4, $t8				# t4 = endereco de impressao do ultimo pixel da primeira linha do char

		addi $t2, $a0, -32				# indice do char na memoria
		sll $t2, $t2, 3					# offset em bytes em relacao ao endereco inicial
		addi $t2, $t2, 0x00004b40		# endereco do char na memoria

		lw $t3, 0($t2)					# carrega a primeira word do char

		addi $t0, $zero, 4				# i = 4

forChar1I: beq $t0, $zero, endForChar1I	# if(i == 0) end for i
		addi $t1, $zero, 8				# j = 8

	forChar1J: beq $t1, $zero, endForChar1J	# if(j == 0) end for j

		div $t3, $t7
		srl $t3, $t3, 1				# t3 = t3/2  ???????????????????
		mfhi $t9					# t9 = t3%2
		beq $t9, $zero, printCharPixelbg1
		sw $t6, 0($t4)
		j endCharPixel1
	
		printCharPixelbg1:	sw $t5, 0($t4)
			endCharPixel1:  addi $t1, $t1, -1				# j--
							addi $t4, $t4, -1				# t4 aponta um pixel para a esquerda
		j forChar1J

	endForChar1J:  addi $t0, $t0, -1				# i--
					addi $t4, $t4, 264				# t4 = t4 + 8 + 256 (t4 aponta para o ultimo pixel da linha de baixo)
	j forChar1I

endForChar1I: lw $t3, 4($t2)					# carrega a segunda word do char

	addi $t0, $zero, 4				# i = 4

forChar2I: beq $t0, $zero, endForChar2I	# if(i == 0) end for i
		addi $t1, $zero, 8				# j = 8

	forChar2J: beq $t1, $zero, endForChar2J	# if(j == 0) end for j

			div $t3, $t7
			srl $t3, $t3, 1					# t3 = t3/2
			mfhi $t9						# t9 = t3%2
			beq $t9, $zero, printCharPixelbg2
			sw $t6, 0($t4)
			j endCharPixel2
	
		printCharPixelbg2: sw $t5, 0($t4)
		
 		endCharPixel2:	addi $t1, $t1, -1				# j--
						addi $t4, $t4, -1				# t4 aponta um pixel para a esquerda
		j forChar2J
	
	endForChar2J:	addi $t0, $t0, -1				# i--
					addi $t4, $t4, 264				# t4 = t4 + 8 + 256 (t4 aponta para o ultimo pixel da linha de baixo)
	j forChar2I

endForChar2I: jr $ra
