/*
 * Registers.v
 *
 * Main processor register bank testbench.
 * Stores information in 32-bit registers. 31 registers are available for
 * writing and 32 are available for reading.
 * Also allows for two simultaneous data reads, has a write enable signal
 * input, is clocked and has an asynchronous reset signal input.
 * Write is syncronous with top-down clock edge
 */
module Registers (iCLK, iCLR, iReadRegister1, iReadRegister2, iWriteRegister,
	iWriteData, iRegWrite, oReadData1, oReadData2, iRegDispSelect, oRegDisp,iRegA0,iA0en);

/* I/O type definition */
input wire [4:0] iReadRegister1, iReadRegister2, iWriteRegister, iRegDispSelect;
input wire [31:0] iWriteData, iRegA0;
input wire iCLK, iCLR, iRegWrite, iA0en;
output wire [31:0] oReadData1, oReadData2, oRegDisp;

/* Local register bank */
reg [31:0] registers[31:0];

parameter SPR=5'd29,			// $SP
		  SPADR=32'd16380;		// Endereco em bytes da pilha - ultimo da memuser

integer i;

initial
begin
	for (i = 0; i <= 31; i = i + 1)
	begin
		registers[i] = 32'b0;
	end
	registers[SPR] = SPADR;  // $sp = Maximo - 33
end

/* Output definition */
assign oReadData1 =	registers[iReadRegister1];
assign oReadData2 =	registers[iReadRegister2];
assign oRegDisp =	registers[iRegDispSelect];



/* Main block for writing and reseting */
always @(negedge iCLK)   //cuidar banco de reg escrito no meio do ciclo
begin
	if (iCLR)
	begin
		for (i = 1; i <= 31; i = i + 1)
		begin
			registers[i] = 32'b0;
		end
		registers[SPR] = SPADR;  // $SP
	end
	else if (~iCLK && iRegWrite)
	begin
		if (iWriteRegister != 5'b0)
		begin
			registers[iWriteRegister] =	iWriteData;
		end
	end

	/* Writing contents of iRegA0 into $a0 */
	if(iA0en)
		registers[5'd4] = iRegA0;
end

endmodule

