`timescale 1ns / 1ps
// Data width of Wishbone slave port can be can be toggled between 64-bit, 32-bit and 16-bit.
// Address width of Wishbone slave port can be can be modified by changing parameter adr_wb_nb.

// Wishbone compliant
// Work as Wishbone slave, support Classic standard SINGLE/BLOCK READ/WRITE Cycle

// registers or wires
// [15:0]kp,ki,kd,sp;	can be both read and written 

`include "wb_bitwidth_define.v"
module PID_Controller#(
`ifdef wb_16bit
parameter	wb_nb=16,
`endif
`ifdef wb_32bit
parameter	wb_nb=32,
`endif
`ifdef wb_64bit
parameter	wb_nb=64,
`endif
adr_wb_nb	=	16,
kp_adr		=	0,
ki_adr		=	1,
kd_adr		=	2,
sv_adr		=	3
)
(//Syscon port
input	i_clk,
input	i_rst,	//reset when low
//Wishbone Slave port
input	i_wb_cyc,
input	i_wb_stb,
input	i_wb_we,

input	[adr_wb_nb-1:0]i_wb_adr, //16 bit address
input	[wb_nb-1:0]i_wb_data,    //Write port

output	o_wb_ack,
output	[wb_nb-1:0]o_wb_data,    //Read port

//Direct input & output
input [31:0]i_pv, //Input: present value
output[31:0]o_mv  //Output: manipulation value
);
    reg	[15:0]kp,ki,kd,sv;
    PID PID_core_wb(.clk(i_clk),.rst(i_rst),.Kp_in(kp),.Ki_in(ki),.Kd_in(kd),.SV_in(sv),.PV_in(i_pv),.MV(o_mv),.of());
    
    reg	wack;	//write acknowledgement
    reg rack;   //read  acknowledgement
    wire	we;	// write enable
    assign	we=i_wb_cyc&i_wb_we&i_wb_stb;
    wire	re;	//read enable
    assign	re=i_wb_cyc&(~i_wb_we)&i_wb_stb;
	reg [wb_nb-1:0]rdata_reg;

	wire	[wb_nb-1:0]rdata[0:3];	//wishbone read data array
	`ifdef	wb_16bit
	assign	rdata[0]=kp;
	assign	rdata[1]=ki;
	assign	rdata[2]=kd;
	assign	rdata[3]=sv;
	`endif

	`ifdef	wb_32bit
	assign	rdata[0]={{16{kp[15]}},kp};
	assign	rdata[1]={{16{ki[15]}},ki};
	assign	rdata[2]={{16{kd[15]}},kd};
	assign	rdata[3]={{16{sv[15]}},sv};
	`endif

	`ifdef	wb_64bit
	assign	rdata[0]={{48{kp[15]}},kp};
	assign	rdata[1]={{48{ki[15]}},ki};
	assign	rdata[2]={{48{kd[15]}},kd};
	assign	rdata[3]={{48{sv[15]}},sv};
	`endif

    wire	[1:0]adr; // address for write & read
    `ifdef wb_16bit
    assign	adr=i_wb_adr[2:1];
    `endif
    `ifdef wb_32bit
    assign	adr=i_wb_adr[3:2];
    `endif
    `ifdef wb_64bit
    assign	adr=i_wb_adr[4:3];
    `endif

    wire	adr_check;	// A '1' means address is within the range of adr
    `ifdef wb_16bit
    assign	adr_check=i_wb_adr[adr_wb_nb-1:3]==0&&i_wb_adr[0]==0;
    `endif
    `ifdef wb_32bit
    assign	adr_check=i_wb_adr[adr_wb_nb-1:4]==0&&i_wb_adr[1:0]==0;
    `endif
    `ifdef wb_64bit
     assign	adr_check=i_wb_adr[adr_wb_nb-1:5]==0&&i_wb_adr[2:0]==0;
    `endif
    
    reg [3:0] state;  //state machine
    parameter Idle = 4'b0001;
    parameter Write = 4'b0010;
    parameter Read = 4'b0100;
	parameter Done = 4'b1000;

    always@(posedge i_clk)
	if(!i_rst)begin
		state<=Idle;
		wack<=0;
		rack<=0;
		kp<=0;
		ki<=0;
		kd<=0;
		sv<=0;
	end
	else begin
		case(state)
		Idle:begin
			if(we)
                state<=Write;
			if(re)
                state<=Read;
		end
		Read:begin
			if(adr_check) begin //Check if the address is a legal address
				rdata_reg<=rdata[adr];
				rack<=1;
				state<=Done;
			end
			else begin //If the address is illegal, read outcome is 0
				rdata_reg<=0;
				rack<=1;
				state<=Done;
			end
		end
		Write:begin
			if(adr_check)begin //Check if the address is an illegal address
				wack<=1;
				state<=Done;
				case(adr)
					0:	begin
						kp<=i_wb_data[15:0];
					end
					1:	begin
						ki<=i_wb_data[15:0];
					end
					2:	begin
						kd<=i_wb_data[15:0];
					end
					3:	begin
						sv<=i_wb_data[15:0];
					end
				endcase
			end
			else begin
				wack<=1;
				state<=Done;
			end
		end
		Done:begin
			if(!i_wb_stb) begin
				if(wack) wack<=0;
				if(rack) rack<=0;
				state<=Idle;
			end
		end
		endcase
	end
	assign	o_wb_ack=(wack|rack)&i_wb_stb;
	assign	o_wb_data=rdata_reg;
endmodule
