`timescale 1ns/10ps
//By Group - 4
//Compile and Simulate the test_top file


interface intfc(input bit clock);

	logic [15:0] pc, Instr_dout,Data_dout, Data_din, Data_addr;
	logic reset, instrmem_rd, complete_instr, complete_data, I_macc, D_macc, Data_rd;

	clocking cb @(posedge clock);
    		default input #1 output #0;
		
		output Instr_dout;
		output Data_din;
		output Data_addr;
		output  complete_instr;
		output complete_data;
		output Data_dout;
		//output reset;
		input pc;
		input instrmem_rd;
		input I_macc;
		input D_macc;
		input Data_rd;
	endclocking

	
endinterface

interface fetch_inf(input bit clock);

	logic [15:0] npc;
	logic [15:0] pc;
	logic instrmem_rd;
	logic reset;
	logic enable_fetch;
	logic enable_updatePC;
	logic [15:0] pcout;
	logic br_taken;
	
	clocking cb @ (posedge clock);
		default input #1 output #0;

		output enable_fetch, enable_updatePC;
		//output reset;
		input pc;
		//input npc;
		//input instrmem_rd;
	endclocking

endinterface


interface decode_inf(input bit clock);

	 logic [15:0] IR;
	 logic [15:0] npc_out;
	 logic [5:0] E_Control;
	 logic [1:0] W_Control;
	 logic reset;
	 logic [15:0] npc_in;
	 logic [15:0] instr_dout;
	 logic enable_decode;
	 logic [3:0] psr;
	 logic Mem_Control;
	
	clocking cb @ (posedge clock);
		default input #1 output #0;

		output enable_decode;
		output npc_in, instr_dout;
		//output reset;
		output psr;
		input IR,npc_out;
		input E_Control;
		input W_Control;
	endclocking

endinterface

interface execute_inf(input bit clock);

	logic [15:0] pcout;
	logic [15:0] aluout;
	
	
	logic [1:0] W_Control_out;
	logic [2:0] sr1, sr2, dr, NZP;
	logic reset;
	logic [1:0] W_Control_in;
	logic enable_execute;
	logic [15:0] npc_in;
	logic [15:0] IR;
	logic [15:0] VSR1;
	logic [15:0] VSR2;
	logic [5:0] E_Control;
	logic Mem_Control_in;
	logic Mem_Control_out;
	logic [15:0]M_Data;
	
	clocking cb @ (posedge clock);
		default input #1 output #0;

		output E_Control;
		output W_Control_in;
		output enable_execute;
		output npc_in, IR;
		//output VSR1, VSR2;
		//output reset;
		input pcout, aluout;
		input W_Control_out;
		input dr, NZP;
		//input sr1, sr2;

	endclocking

endinterface



interface writeback_inf(input bit clock);

	 logic [15:0] VSR1, VSR2;
	 logic [2:0] psr;
	 logic reset;
	 logic enable_writeback;
	 logic [15:0] pcout, aluout;
	 logic [2:0] dr;
	 logic [2:0] sr1, sr2;
	 logic [1:0] W_Control;
	 logic [15:0] memout, npc;
	 logic [15:0] dr_val, DR_in;
	 logic [2:0] dr_wb;
	 logic [15:0] r0,r1,r2,r3,r4,r5,r6,r7;
	
	clocking cb @ (posedge clock);
		default input #1 output #0;

		output pcout, aluout;
		output dr;
		//output sr1, sr2;
		output W_Control;
		//output reset;
		output enable_writeback;
		//output dr_wb;
		//input VSR1, VSR2;
		input psr;
		//input dr_val;

	endclocking

endinterface

interface controller_inf(input bit clock);

	logic enable_fetch, enable_decode, enable_execute, enable_writeback, enable_updatePC;
	logic reset;
	logic [2:0] psr, NZP;
	logic complete_instr;
	logic [1:0] mem_state;
	logic complete_data, br_taken;
	logic [15:0] IR;
	
	clocking cb @ (posedge clock);
		default input #1 output #0;

		output psr, NZP;
		output complete_instr;
		output reset;
		input enable_fetch;
		input enable_decode;
		input enable_execute;
		input enable_writeback;
		input enable_updatePC;

	endclocking

endinterface

//interfaces defined for all the sub_DUTs and the main DUT




class packet;

	rand logic [15:0] instr_dout;
	rand logic complete_instr;
	rand logic complete_data;
	rand logic br_taken;
	logic reset;
	constraint c1 {
		instr_dout[15:12] inside {1,5,9,14};
		complete_instr inside {1};
		br_taken inside {0};
		complete_data inside {1};
	}

	
	logic fet_enable_updatePC;
	logic fet_enable_fetch;

	
	logic [15:0] dec_npc_in;
	logic dec_enable_decode;
	logic [15:0] dec_instr_dout;
	logic [2:0] dec_psr;

	
	logic exe_enable_execute;
	logic [15:0] exe_npc_in;
	logic [15:0] exe_IR;
	logic [15:0] exe_VSR1;
	logic [15:0] exe_VSR2;
	logic [5:0] exe_E_Control ;
	logic [1:0] exe_W_Control_in;

	
	logic  wb_enable_writeback;
	logic [15:0] wb_pcout;
	logic [15:0] wb_aluout;
	logic [1:0] wb_W_Control;
	logic [2:0] wb_sr1;
	logic [2:0] wb_sr2;
	logic [2:0] wb_dr;

	
	logic cntrl_complete_instr;
	logic [2:0] cntrl_psr;
	logic [2:0] cntrl_NZP;

endclass




class outpacket;

	logic [15:0] PC;
	logic instrmem_rd;
	//fetch
	logic [15:0] npc_fet,pc_fet;
	logic instrmem_rd_fet;
	//decode
	logic [15:0] IR_dec, npc_dec;
	logic [5:0] E_Control_dec;
	logic [1:0] W_Control_dec;
	
	//execute
	logic [15:0] aluout_exe, pcout_exe;
	logic [2:0] sr1_exe,sr2_exe,dr_exe;
	logic [1:0] W_Control_exe;
	//wb
	logic [15:0] VSR1_wb, VSR2_wb;
	logic [2:0] psr_wb;

	logic enable_fetch, enable_decode, enable_execute, enable_writeback, enable_updatePC;


endclass




class Generator;
		
	packet  pkt2send;	
	
    typedef mailbox #(packet) in_box_type;
	in_box_type in_box;
	int flag;
	int	packet_number;
	int 	number_packets;
	extern function new(int number_packets);
	extern virtual task gen();
	extern virtual task start();
endclass

function Generator::new(int number_packets);
	
	this.pkt2send = new();
	this.in_box = new;
	this.packet_number = 0;
	this.number_packets = number_packets;
	this.flag = 0;
endfunction

task Generator::gen();
	  
	if (!pkt2send.randomize()) 
	begin
		$display(" \n%m\n[ERROR]%0d gen(): Randomization Failed!\n", $time);
		$finish;	
	end
	if(!flag) begin
		pkt2send.reset = 1;
		flag =1;
	end
	else begin
		pkt2send.reset = 0;
	end
	
endtask



task Generator::start();
	  $display ($time, "ns:  [GENERATOR] Generator Started\n");
	  fork
		
		  for (int i=0; i<number_packets ; i++) 
		  begin
			  gen();
			
			  begin 
			      packet pkt = new pkt2send; 
				  in_box.put(pkt); 
			
			  end
		  end
		  $display($time, "ns:  [GENERATOR] Generation Finished Creating %d Packets  \n", number_packets);
      join_none
endtask


class DriverBase;
	
	virtual intfc.TB intf;
	virtual fetch_inf fet;	
	virtual decode_inf dec;
	virtual execute_inf exe;
	virtual writeback_inf wb;
	virtual controller_inf cntrl;	
		
	packet    pkt2send;	

	logic [15:0] payload_instr_dout;
	logic payload_complete_instr;
	logic payload_complete_data;
	logic payload_br_taken;
	logic payload_reset;


	extern function new(virtual intfc.TB intf, virtual fetch_inf fet, virtual decode_inf dec, virtual execute_inf exe, 
				virtual writeback_inf wb, virtual controller_inf cntrl);
	extern virtual task send();

	
endclass

function DriverBase::new( virtual intfc.TB intf, virtual fetch_inf fet, virtual decode_inf dec, virtual execute_inf exe, 
				virtual writeback_inf wb, virtual controller_inf cntrl);
	this.intf = intf;
	this.fet = fet;
	this.dec = dec;
	this.exe = exe;
	this.wb = wb;
	this.cntrl = cntrl;
	pkt2send = new();
endfunction

task DriverBase::send();
	$display($time, "ns:  [DRIVER] Sending Payload Begin\n");
	
	intf.complete_data <= payload_complete_data;
	cntrl.complete_instr <= payload_complete_instr;
	cntrl.reset <= payload_reset;
	dec.instr_dout <= payload_instr_dout;
	fet.br_taken <=payload_br_taken;
	fet.reset <= payload_reset;
	dec.reset <= payload_reset;
	exe.reset <= payload_reset;
	wb.reset <= payload_reset;
	
	
endtask



class Driver extends DriverBase;
  //mailbox in_box;	// Generator mailbox 
  typedef mailbox #(packet) in_box_type;
  in_box_type in_box = new;
  //mailbox out_box;	// Scoreboard mailbox
  typedef mailbox #(packet) out_box_type;
  out_box_type out_box = new;
 

  extern function new( in_box_type in_box, out_box_type out_box, virtual intfc.TB intf, virtual fetch_inf fet, virtual decode_inf dec, virtual execute_inf exe, 
				virtual writeback_inf wb, virtual controller_inf cntrl);
  extern virtual task start();
endclass

function Driver::new( in_box_type in_box, out_box_type out_box, virtual intfc.TB intf, virtual fetch_inf fet, virtual decode_inf dec, virtual execute_inf exe, 
				virtual writeback_inf wb, virtual controller_inf cntrl);
  super.new(intf, fet, dec, exe, wb , cntrl);
  this.in_box = in_box;
  this.out_box = out_box;
  this.intf = intf;
  this.fet = fet;
  this.dec = dec; 
  this.exe = exe;
  this.wb = wb;
  this.cntrl = cntrl;
endfunction

task Driver::start(); 
	int packets_sent = 0;
	$display ($time, "ns:  [DRIVER] Driver Started\n");
    fork
	    forever
	    begin
	      	in_box.get(pkt2send); 
			packets_sent++;
		  	
			$display ($time, "[DRIVER] Sending in new packet BEGIN\n");
		  	this.payload_instr_dout = pkt2send.instr_dout;
			this.payload_complete_data = pkt2send.complete_data;
			this.payload_complete_instr = pkt2send.complete_instr;
			this.payload_reset = pkt2send.reset;
			this.payload_br_taken = pkt2send.br_taken;
			
 	     		send();
	
			$display ($time, "ns:  [DRIVER] Sending in new packet END\n");
			$display ($time, "ns:  [DRIVER] Number of packets sent = %d\n", packets_sent);
	     		out_box.put(pkt2send);
			$display ($time,  "ns:  [DRIVER] The number of Packets in the Generator Mailbox = %d\n", in_box.num());
			if(in_box.num() == 0)
			begin
				break;
			end
			#100;
			 
		  	
	    end
	join_none
endtask


class Scoreboard;
	
	virtual intfc.TB intf;
	virtual fetch_inf fet;	
	virtual decode_inf dec;
	virtual execute_inf exe;
	virtual writeback_inf wb;
	virtual controller_inf cntrl;

			
	packet pkt_sent = new();	// Packet object from Driver
	
  	
	typedef mailbox #(packet) out_box_type;
  	out_box_type driver_mbox;		// mailbox for Packet objects from Drivers

  	
	
	
	extern function new(virtual intfc.TB intf, virtual fetch_inf fet, virtual decode_inf dec, virtual execute_inf exe, 
				virtual writeback_inf wb, virtual controller_inf cntrl, out_box_type driver_mbox = null);
	extern virtual task start();
	
	extern virtual task send_control();
	extern virtual task send_fetch();
	extern virtual task send_decode();
	extern virtual task send_execute();

	extern virtual task check_fetch_updatePC();
	extern virtual task check_decode();
	extern virtual task check_execute();
	extern virtual task check_writeback();
	extern virtual task check_controller();
endclass
// variables defined outside the class, so that it is visible in the wave.
	logic [15:0] PC_chk;
	logic instrmem_rd_chk;
	
	//fetch
	logic [15:0] npc_fet_chk,pc_fet_chk;
	logic instrmem_rd_fet_chk;
	
	//decode
	logic [15:0] IR_dec_chk, npc_dec_chk;
	logic [5:0] E_Control_dec_chk;
	logic [1:0] W_Control_dec_chk;
	
	//execute
	logic [15:0] aluout_exe_chk, pcout_exe_chk;
	logic [2:0] sr1_exe_chk, sr2_exe_chk, dr_exe_chk,NZP_exe_chk;
	logic [1:0] W_Control_exe_chk;
	logic [15:0] m_data_chk;	
	logic mem_control_out_chk;
	
	//wb
	logic [15:0] VSR1_wb_chk, VSR2_wb_chk;
	logic [2:0] psr_wb_chk;
	logic [15:0]tram[7:0];
	
	//control
	logic enable_fetch_chk, enable_decode_chk, enable_execute_chk, enable_writeback_chk, enable_updatePC_chk;

	typedef enum{idle, fetch, decode, execute, writeback, updatePC} states;
		states curr_state;
	int flag;

function Scoreboard::new(virtual intfc.TB intf, virtual fetch_inf fet, virtual decode_inf dec, virtual execute_inf exe, 
	virtual writeback_inf wb, virtual controller_inf cntrl ,out_box_type driver_mbox = null);
	
	if (driver_mbox == null) 
		driver_mbox = new();
	
	this.driver_mbox = driver_mbox;
	
	this.intf = intf;
	this.fet = fet;
	this.dec = dec;
	this.exe = exe;
	this.wb = wb;
	this.cntrl = cntrl;
endfunction

		
task Scoreboard::start();
	$display ($time, "ns:  [SCOREBOARD] Scoreboard Started\n");
	PC_chk=0;
	instrmem_rd_chk=0;
	
	//decode
	IR_dec_chk=0;
	npc_dec_chk=0;
	E_Control_dec_chk=0;
	W_Control_dec_chk=0;
	
	//execute
	aluout_exe_chk=0;
	pcout_exe_chk=0;
	dr_exe_chk=0;
	W_Control_exe_chk=0;
	m_data_chk = 0;	
	mem_control_out_chk = 0;
	
	flag = 0;

	//initialsising ram
	for(int i = 0; i<8; i++) begin
		tram[i]=i;
	end

//controller running forever generating the control signals in seperate thread,
//while packet is acquired from mailbox at posedge of clock in seperate thread and evaluating and checking the values.
	fork
		forever check_controller();
		forever 
		begin
				$display ($time, "ns:  [SCOREBOARD] Grabbing Data From  Driver \n");
				@ (posedge intf.clock)
				driver_mbox.get(pkt_sent); 	
		end
	join_none
	$display ($time, "[SCOREBOARD] Forking of Process Finished\n");
endtask


// these tasks send the required correct values(input) to the DUT's interface when called.
task Scoreboard::send_control();

fet.enable_fetch<=enable_fetch_chk;
fet.enable_updatePC<=enable_updatePC_chk;
dec.enable_decode<=enable_decode_chk;
exe.enable_execute<=enable_execute_chk;
wb.enable_writeback<=enable_writeback_chk;


wb.sr1 <=sr1_exe_chk;
wb.sr2 <=sr2_exe_chk;

exe.VSR1 <= VSR1_wb_chk;
exe.VSR2 <= VSR2_wb_chk;

endtask


task Scoreboard::send_fetch();
dec.npc_in <= npc_fet_chk;
endtask

	
task Scoreboard::send_decode();
exe.npc_in <=npc_dec_chk;
exe.IR <= IR_dec_chk;
exe.W_Control_in <= W_Control_dec_chk;
exe.E_Control <= E_Control_dec_chk;

endtask

	
task Scoreboard::send_execute();
wb.aluout <= aluout_exe_chk;
wb.pcout <= pcout_exe_chk;
wb.dr <=dr_exe_chk;
wb.W_Control <=W_Control_exe_chk;
endtask


//these tasks check the output of DUT coming from it's interface with the values generated in them(GOLDEN MODEL values).
task Scoreboard::check_fetch_updatePC();


		if(fet.reset) begin
			pc_fet_chk = 16'h3000;
			npc_fet_chk = 16'h3001;
				
		end	

		if( enable_updatePC_chk ) begin
			pc_fet_chk=npc_fet_chk;
			npc_fet_chk=npc_fet_chk+1;			
		end

send_fetch();

		#1
		assert(pc_fet_chk == fet.pc) 
		else begin
			$display("---------FETCH: ERROR PC-------------");
			$display("%0t Inputs:  EnableUpdatePC= %b, EnableFetch= %b", $time, enable_updatePC_chk, enable_fetch_chk);
			$display("%0t  BUG PC: Expected Value %h, Observed value = %h\n" ,$time,  pc_fet_chk, fet.pc);
		end

		assert(npc_fet_chk == fet.npc) 
		else begin
			$display("---------FETCH: ERROR NPC-------------");
			$display("%0t Inputs:  EnableUpdatePC= %b, EnableFetch= %b", $time, enable_updatePC_chk, enable_fetch_chk);
			$display("%0t BUG NPC: Expected Value %h, Observed value = %h\n" , $time, npc_fet_chk, fet.npc);
		end

endtask

task Scoreboard::check_decode();

		if(dec.reset) begin
			IR_dec_chk=0;
			npc_dec_chk=0;
			E_Control_dec_chk=0;
			W_Control_dec_chk=0;
		end
		else if(enable_decode_chk ) begin
			IR_dec_chk = pkt_sent.instr_dout;
			npc_dec_chk = npc_fet_chk;
			casex({IR_dec_chk[15:12],IR_dec_chk[5]}) 
				5'b00010: begin W_Control_dec_chk=2'b00; E_Control_dec_chk=6'b00???1; end
			 	5'b00011: begin W_Control_dec_chk=2'b00; E_Control_dec_chk=6'b00???0; end
				5'b01010: begin W_Control_dec_chk=2'b00; E_Control_dec_chk=6'b01???1; end
				5'b01011: begin W_Control_dec_chk=2'b00; E_Control_dec_chk=6'b01???0; end
				5'b1001?: begin W_Control_dec_chk=2'b00; E_Control_dec_chk=6'b10????; end
				5'b1110?: begin W_Control_dec_chk=2'b10; E_Control_dec_chk=6'b??011?; end
				default: begin W_Control_dec_chk=2'b11; E_Control_dec_chk=6'b111111; end
			
			endcase		
		end
	

	
	assert(pkt_sent.instr_dout==?dec.IR) 
	else begin
		$display("---------DECODE: ERROR IR-------------");
		$display("%0t Inputs: Instr_Dout= %h, NPC_in= %h, EnableDecode = %b, Imm5 = %b", $time, pkt_sent.instr_dout, npc_fet_chk, enable_decode_chk, pkt_sent.instr_dout[5]);
		$display("%0t EXPECTED IR: %h , OBSERVED IR: %h\n", $time, pkt_sent.instr_dout, dec.IR);	
	end
	assert(npc_dec_chk==?dec.npc_out) 
	else begin
		$display("---------DECODE: ERROR NPC_OUT-------------");
		$display("%0t Inputs: Instr_Dout= %h, NPC_in= %h, EnableDecode = %b, Imm5 = %b", $time, pkt_sent.instr_dout, npc_fet_chk, enable_decode_chk, pkt_sent.instr_dout[5]);
		$display("%0t EXPECTED NPC: %h , OBSERVED NPC: %h\n", $time, npc_dec_chk, dec.npc_out);
	end
	assert(W_Control_dec_chk==?dec.W_Control) 
	else begin
		$display("---------DECODE: ERROR W_CONTROL-------------");
		$display("%0t Inputs: Instr_Dout= %h, NPC_in= %h, EnableDecode = %b, Imm5 = %b", $time, pkt_sent.instr_dout, npc_fet_chk, enable_decode_chk, pkt_sent.instr_dout[5]);
		$display("%0t EXPECTED W_CONTROL: %h , OBSERVED W_CONTROL: %h\n", $time, W_Control_dec_chk, dec.W_Control);
	end
	assert(E_Control_dec_chk==?dec.E_Control) 
	else begin
		$display("---------DECODE: ERROR E_Control-------------");
		$display("%0t Inputs: Instr_Dout= %h, NPC_in= %h, EnableDecode = %b, Imm5 = %b", $time, pkt_sent.instr_dout, npc_fet_chk, enable_decode_chk, pkt_sent.instr_dout[5]);
		$display("%0t EXPECTED E_CONTROL: %h , OBSERVED E_CONTROL: %h\n", $time, E_Control_dec_chk, dec.E_Control);
	end
send_decode();
endtask	


logic [15:0] offset11, offset9, offset6, imm5;
logic [1:0] pcselect1, alu_control;
logic pcselect2, op2select;
logic [15:0] pcout_temp;
logic [15:0] aluin1, aluin2;
 

task Scoreboard:: check_execute();


	if(exe.reset) begin
		aluout_exe_chk=0;
		pcout_exe_chk=0;
		W_Control_exe_chk=0;
		dr_exe_chk=0;
		NZP_exe_chk = 0;	
		m_data_chk = 0;	
		mem_control_out_chk = 0;	
	end
	
	else if(enable_execute_chk ) begin
		dr_exe_chk = pkt_sent.instr_dout[11:9];
		W_Control_exe_chk = W_Control_dec_chk;
		NZP_exe_chk = 0;	
		m_data_chk = 0;	
		mem_control_out_chk = 0;

		op2select = E_Control_dec_chk[0];
		pcselect2 = E_Control_dec_chk[1];
		pcselect1 = E_Control_dec_chk[3:2];
		alu_control = E_Control_dec_chk[5:4];

		imm5 = {{11{pkt_sent.instr_dout[4]}},pkt_sent.instr_dout[4:0]};
		offset6 = {{10{pkt_sent.instr_dout[5]}}, pkt_sent.instr_dout[5:0]};
		offset9 = {{7{pkt_sent.instr_dout[8]}}, pkt_sent.instr_dout[8:0]};
		offset11 = {{5{pkt_sent.instr_dout[10]}}, pkt_sent.instr_dout[10:0]};

		aluin1 = VSR1_wb_chk;
		aluin2 = (op2select?VSR2_wb_chk:imm5);
		

		case(alu_control)
			0: aluout_exe_chk = aluin1 + aluin2;
			1: aluout_exe_chk = aluin1 & aluin2;
			2: aluout_exe_chk = ~aluin1;
		endcase

		case(pcselect1)
			0: pcout_temp = offset11;
			1: pcout_temp = offset9;
			2: pcout_temp = offset6;
			3: pcout_temp = 16'h0000;
		endcase
		
		pcout_exe_chk = (pcselect2?npc_dec_chk:VSR1_wb_chk) + pcout_temp;
	end

	assert(aluout_exe_chk==exe.aluout ) 
	else begin
		$display("---------EXECUTE: ERROR ALUOUT-------------");
		$display("%0t Inputs:  IR= %h, NPC= %h, E_Control = %b, W_Control = %b, VSR1 = %h, VSR2 = %h", $time, pkt_sent.instr_dout, npc_dec_chk, E_Control_dec_chk, W_Control_dec_chk, VSR1_wb_chk, VSR2_wb_chk);
		$display("EXPECTED aluout: %h , OBSERVED aluout: %h\n",  aluout_exe_chk, exe.aluout);	
		
		
	end
	assert(pcout_exe_chk==exe.pcout ) 
	else begin
		$display("---------EXECUTE: ERROR PCOUT-------------");
		$display("%0t Inputs:  IR= %h, NPC= %h, E_Control = %b, W_Control = %b, VSR1 = %h, VSR2 = %h", $time, pkt_sent.instr_dout, npc_dec_chk, E_Control_dec_chk, W_Control_dec_chk, VSR1_wb_chk, VSR2_wb_chk);
		$display("EXPECTED pcout: %h , OBSERVED pcout: %h\n",  pcout_exe_chk, exe.pcout);	
		
	end
	assert (W_Control_exe_chk==exe.W_Control_out) 
	else begin
		$display("---------EXECUTE: ERROR W_CONTROL-------------");
		$display("%0t Inputs:  IR= %h, NPC= %h, E_Control = %b, W_Control = %b, VSR1 = %h, VSR2 = %h", $time, pkt_sent.instr_dout, npc_dec_chk, E_Control_dec_chk, W_Control_dec_chk, VSR1_wb_chk, VSR2_wb_chk);
		$display("EXPECTED W_Control: %h , OBSERVED W_Control: %h\n", W_Control_exe_chk, exe.W_Control_out);	
	end
	
	assert (dr_exe_chk==exe.dr) 
	else begin
		$display("---------EXECUTE: ERROR DR------------");
		$display("%0t Inputs:  IR= %h, NPC= %h, E_Control = %b, W_Control = %b, VSR1 = %h, VSR2 = %h", $time, pkt_sent.instr_dout, npc_dec_chk, E_Control_dec_chk, W_Control_dec_chk, VSR1_wb_chk, VSR2_wb_chk);
		$display("EXPECTED DR: %b , OBSERVED DR: %b\n", dr_exe_chk, exe.dr);	
	end

	
			

send_execute();
 endtask


 logic [15:0] DR_in;

task Scoreboard::check_writeback();
		
	if(wb.reset) begin
		
		VSR1_wb_chk = tram[sr1_exe_chk];
		VSR2_wb_chk = tram[sr2_exe_chk];
		
		tram[0]=16'hxxxx;
  		tram[1]=16'hxxxx;  
  		tram[2]=16'hxxxx;
  		tram[3]=16'hxxxx;  
  		tram[4]=16'hxxxx;
  		tram[5]=16'hxxxx;  
  		tram[6]=16'hxxxx;
  		tram[7]=16'hxxxx;
	end


	else if(enable_writeback_chk) begin
		
		case(W_Control_exe_chk)
			2'b00: DR_in=aluout_exe_chk;
			//2'b01: DR_in=memout_exe_chk;
			2'b10: DR_in=pcout_exe_chk;
		endcase
		
		if(DR_in == 16'h0000)
			psr_wb_chk = 3'b010;
		else if(DR_in[15] == 0)
			psr_wb_chk = 3'b001;
		else if(DR_in[15] == 1)
			psr_wb_chk = 3'b100;


		tram[dr_exe_chk] = DR_in; //write opr
		wb.dr_wb <= dr_exe_chk;
		wb.dr <= dr_exe_chk;
		
		
	end

	assert (wb.psr==psr_wb_chk) 
	else begin
		$display("---------WRITEBACK: ERROR PSR-------------");
		$display("DUT RAM: R0: %h, R1: %h, R2: %h, R3: %h, R4: %h, R5: %h, R6: %h, R7: %h", 
			test_top.WB.RF.ram[0],  test_top.WB.RF.ram[1], test_top.WB.RF.ram[2], test_top.WB.RF.ram[3], test_top.WB.RF.ram[4],test_top.WB.RF.ram[5], test_top.WB.RF.ram[6] , test_top.WB.RF.ram[7]);
		$display("TEST RAM: R0: %h, R1: %h, R2: %h, R3: %h, R4: %h, R5: %h, R6: %h, R7: %h", tram[0],  tram[1], tram[2], tram[3], tram[4], tram[5], tram[6], tram[7]);
		$display("%0t Inputs: Reset = %b, ALUOUT = %h,  PCOUT = %h, W_Control = %b, DR_test = %h, DR_dut = %h, dr = %h, sr1 = %h, sr2 = %h", $time, wb.reset, aluout_exe_chk,  pcout_exe_chk, W_Control_exe_chk, DR_in, wb.DR_in, dr_exe_chk, sr1_exe_chk, sr2_exe_chk);
		$display("EXPECTED PSR: %h , OBSERVED PSR: %h\n", psr_wb_chk, wb.psr);	

	end

	assert (test_top.WB.RF.ram[dr_exe_chk] == tram[dr_exe_chk]) 
	else begin
		$display("----------WRITEBACK: ERROR WRITE-------------");
		$display("DUT RAM: R0: %h, R1: %h, R2: %h, R3: %h, R4: %h, R5: %h, R6: %h, R7: %h",
		 test_top.WB.RF.ram[0],  test_top.WB.RF.ram[1], test_top.WB.RF.ram[2], test_top.WB.RF.ram[3], test_top.WB.RF.ram[4],test_top.WB.RF.ram[5], test_top.WB.RF.ram[6] , test_top.WB.RF.ram[7]);
		$display("TEST RAM: R0: %h, R1: %h, R2: %h, R3: %h, R4: %h, R5: %h, R6: %h, R7: %h", tram[0],  tram[1], tram[2], tram[3], tram[4], tram[5], tram[6], tram[7]);		
		$display("%0t Inputs: Reset = %b, ALUOUT = %h, PCOUT = %h, W_Control = %b, DR_test = %h, DR_dut: %h, dr = %h, sr1 = %h, sr2 = %h", $time, wb.reset, aluout_exe_chk,  pcout_exe_chk, W_Control_exe_chk, DR_in, wb.DR_in, dr_exe_chk, sr1_exe_chk, sr2_exe_chk);
		$display("EXPECTED write: %h , OBSERVED write: %h\n", tram[dr_exe_chk], test_top.WB.RF.ram[dr_exe_chk]);
	end
		

endtask
	

task Scoreboard::check_controller();
//FSM will generate the control signals and will call the required check task based on the state.
//For example: in the decode state, the enable_execute goes high and next state becomes execute, so check decode will be called
// to evaluate the required values(output of decode is input of execute) that will be sent to execute block.
//Similarlt for other states.
	@(posedge intf.clock);
	#0
	if(cntrl.reset) begin
		curr_state = fetch;
		enable_fetch_chk=1;
		enable_decode_chk=0;
		enable_execute_chk=0;
		enable_writeback_chk=0;
		enable_updatePC_chk=0;
		
	end
	else begin	
		case(curr_state)
			fetch: begin
				
				if(pkt_sent.complete_instr) begin
				enable_fetch_chk=0;
				enable_decode_chk=1;
				enable_execute_chk=0;
				enable_writeback_chk=0;
				enable_updatePC_chk=0;
				curr_state = decode;
				end
				check_fetch_updatePC();
			end
				
			decode: begin

				enable_fetch_chk=0;
				enable_decode_chk=0;
				enable_execute_chk=1;
				enable_writeback_chk=0;
				enable_updatePC_chk=0;
				curr_state = execute;				
				check_decode();			
			end

			execute: begin
			
				enable_fetch_chk=0;
				enable_decode_chk=0;
				enable_execute_chk=0;
				enable_writeback_chk=1;
				enable_updatePC_chk=0;
				curr_state = writeback;
				check_execute();		
			end

			writeback: begin

				enable_fetch_chk=0;
				enable_decode_chk=0;
				enable_execute_chk=0;
				enable_writeback_chk=0;
				enable_updatePC_chk=1;
				curr_state = updatePC;
				
				check_writeback();
			end

			updatePC: begin

				enable_fetch_chk=1;
				enable_decode_chk=0;
				enable_execute_chk=0;
				enable_writeback_chk=0;
				enable_updatePC_chk=0;
				curr_state = fetch;
				//check_fetch_instr();		
			end
		endcase
	end

send_control();


	assert(cntrl.enable_fetch == enable_fetch_chk) 
	else begin
		$display("------------------CONTROLLER: Error Fetch-----------------");
		$display("%0t INPUT: complete_instr = %b", $time, pkt_sent.complete_instr);
		$display("OUTPUT: Expected enable_fetch: %b, Observed enable_fetch = %b\n", enable_fetch_chk, cntrl.enable_fetch); 
	end
	assert(cntrl.enable_decode== enable_decode_chk)
	else begin
		$display("------------------CONTROLLER: Error Decode-----------------");
		$display("%0t INPUT: complete_instr = %b", $time, pkt_sent.complete_instr);
		$display("OUTPUT: Expected enable_decode: %b, Observed enable_decode = %b\n", enable_decode_chk, cntrl.enable_decode); 
	end
	assert(cntrl.enable_execute== enable_execute_chk) 
	else begin
		$display("------------------CONTROLLER: Error Execute-----------------");
		$display("%0t INPUT: complete_instr = %b", $time, pkt_sent.complete_instr);
		$display("OUTPUT: Expected enable_execute: %b, Observed enable_execute = %b\n", enable_execute_chk, cntrl.enable_execute); 
	end
	assert(cntrl.enable_writeback== enable_writeback_chk)
	else begin
		$display("------------------CONTROLLER: Error Writeback-----------------");
		$display("%0t INPUT: complete_instr = %b", $time, pkt_sent.complete_instr);
		$display("OUTPUT: Expected enable_writeback: %b, Observed enable_writeback = %b\n", enable_writeback_chk, cntrl.enable_writeback); 
	end
	assert(cntrl.enable_updatePC== enable_updatePC_chk)
	else begin
		$display("------------------CONTROLLER: Error Enable_updatePC-----------------");
		$display("%0t INPUT: complete_instr = %b", $time, pkt_sent.complete_instr);
		$display("OUTPUT: Expected enable_updatePC: %b, Observed enable_updatePC = %b\n", enable_updatePC_chk, cntrl.enable_updatePC); 
	end


		
		//calculating and checking the asynchronously evaluated values.

		if(enable_fetch_chk) begin
			instrmem_rd_fet_chk = 1;
			
		end
		else begin
			instrmem_rd_fet_chk = 1'bz;
			
		end

	
		sr1_exe_chk = pkt_sent.instr_dout[8:6];
		sr2_exe_chk = ((pkt_sent.instr_dout[15:12]==4'b1110)?3'b000:pkt_sent.instr_dout[2:0]); 
		
	
		VSR1_wb_chk = tram[sr1_exe_chk];
		VSR2_wb_chk = tram[sr2_exe_chk];
		
		assert (instrmem_rd_fet_chk == fet.instrmem_rd) 
		else begin
			$display("---------FETCH: ERROR INSTRMEM READ-------------");
			$display("%0t Inputs:  EnableUpdatePC= %b, EnableFetch= %b", $time, enable_updatePC_chk, enable_fetch_chk);
			$display("%0t BUG INSTR Expected Value %b, Observed value = %b\n" ,$time, instrmem_rd_fet_chk, fet.instrmem_rd);
		end

	
		assert(sr1_exe_chk==exe.sr1)
		else begin
			$display("---------EXECUTE: ERROR SR1------------");
			$display("%0t Inputs:  IR= %h, NPC= %h, E_Control = %b, W_Control = %b, VSR1 = %h, VSR2 = %h", $time, pkt_sent.instr_dout, npc_dec_chk, E_Control_dec_chk, W_Control_dec_chk, VSR1_wb_chk, VSR2_wb_chk);
			$display("EXPECTED SR1: %b , OBSERVED SR1: %b\n",  sr1_exe_chk, exe.sr1);	
		 end	
		
		assert(sr2_exe_chk==exe.sr2 )
		else begin
			$display("---------EXECUTE: ERROR SR2------------");
			$display("%0t Inputs:  IR= %h, NPC= %h, E_Control = %b, W_Control = %b, VSR1 = %h, VSR2 = %h", $time, pkt_sent.instr_dout, npc_dec_chk, E_Control_dec_chk, W_Control_dec_chk, VSR1_wb_chk, VSR2_wb_chk);
			$display("EXPECTED SR2: %b, OBSERVED SR2: %b\n", sr2_exe_chk, exe.sr2);
		end

		assert(wb.VSR1==VSR1_wb_chk) 
		else begin  
			$display("---------ERROR VSR1------------");
			$display("DUT RAM: R0: %h, R1: %h, R2: %h, R3: %h, R4: %h, R5: %h, R6: %h, R7: %h", 
				test_top.WB.RF.ram[0],  test_top.WB.RF.ram[1], test_top.WB.RF.ram[2], test_top.WB.RF.ram[3], test_top.WB.RF.ram[4],test_top.WB.RF.ram[5], test_top.WB.RF.ram[6] , test_top.WB.RF.ram[7]);
			$display("TEST RAM: R0: %h, R1: %h, R2: %h, R3: %h, R4: %h, R5: %h, R6: %h, R7: %h", tram[0],  tram[1], tram[2], tram[3], tram[4], tram[5], tram[6], tram[7]);
			$display("%0t Inputs: Reset = %b, sr1 = %d, sr2 = %d", $time, wb.reset, sr1_exe_chk, sr2_exe_chk);
			$display("EXPECTED VSR1: %h , OBSERVED VSR1: %h\n",  VSR1_wb_chk, wb.VSR1);

		end

		assert(wb.VSR2==VSR2_wb_chk) 
		else begin   
			$display("---------ERROR VSR2------------");
			$display("DUT RAM: R0: %h, R1: %h, R2: %h, R3: %h, R4: %h, R5: %h, R6: %h, R7: %h",
 					test_top.WB.RF.ram[0],  test_top.WB.RF.ram[1], test_top.WB.RF.ram[2], test_top.WB.RF.ram[3], test_top.WB.RF.ram[4],test_top.WB.RF.ram[5], test_top.WB.RF.ram[6] , test_top.WB.RF.ram[7]);
			$display("TEST RAM: R0: %h, R1: %h, R2: %h, R3: %h, R4: %h, R5: %h, R6: %h, R7: %h", tram[0],  tram[1], tram[2], tram[3], tram[4], tram[5], tram[6], tram[7]);
			$display("%0t Inputs: Reset = %b, sr1 = %d, sr2 = %d", $time, wb.reset, sr1_exe_chk, sr2_exe_chk);
			$display("EXPECTED VSR2: %h , OBSERVED VSR2: %h\n",  VSR2_wb_chk, wb.VSR2);
		end

	

endtask	






program tb(intfc intf, fetch_inf fet, decode_inf dec, execute_inf exe, writeback_inf wb, controller_inf cntrl);
	


	Generator  	generator;	// generator object
	Driver     	drvr;		// driver objects
	Scoreboard 	sb;		// scoreboard object
	

	
	packet 	pkt_sent = new();
	int 	number_packets;
	
	initial begin
		number_packets =50;
       		generator = new(number_packets);
		sb = new(intf, fet, dec, exe, wb , cntrl); 
		drvr = new( generator.in_box, sb.driver_mbox, intf, fet, dec, exe, wb , cntrl);
		
		generator.start();
		drvr.start(); 
		sb.start();
		
    	repeat(number_packets+1) 
		#100;
		//$display($time, " DONE --------------------------------------");
  	end


endprogram



module test_top();
//instantiating the interfaces and DUTs.
	logic clock;
	logic [2:0] dr_addr;

	intfc intf(clock);
	
	fetch_inf fet(clock);
	
	decode_inf dec(clock);

	execute_inf exe(clock);

	writeback_inf wb(clock);

	controller_inf cntrl(clock);
	
	
	
	tb test(intf, fet, dec, exe, wb, cntrl);


	Fetch	Fetch1 (
					.clock(clock), .reset(fet.reset), .enable_updatePC(fet.enable_updatePC), 
					.enable_fetch(fet.enable_fetch), .pc(fet.pc), .npc_out(fet.npc), 
					.instrmem_rd(fet.instrmem_rd), .taddr(fet.pcout), .br_taken(fet.br_taken)
				);

	Decode  Dec (
					.clock(clock), .reset(dec.reset), .enable_decode(dec.enable_decode), 
					.dout(dec.instr_dout), .E_Control(dec.E_Control), .npc_in(dec.npc_in), 
					.Mem_Control(dec.Mem_Control), .W_Control(dec.W_Control), 
					.IR(dec.IR), .npc_out(dec.npc_out)
	      		);							
	Execute	Ex	(		
					.clock(clock), .reset(exe.reset), .E_Control(exe.E_Control), .IR(exe.IR), 
					.npc(exe.npc_in), .W_Control_in(exe.W_Control_in), .Mem_Control_in(exe.Mem_Control_in), 
					.VSR1(exe.VSR1), .VSR2(exe.VSR2), .enable_execute(exe.enable_execute), 
					.W_Control_out(exe.W_Control_out), .Mem_Control_out(exe.Mem_Control_out), 
					.aluout(exe.aluout), .pcout(exe.pcout), .sr1(exe.sr1), .sr2(exe.sr2), .dr(exe.dr), 
					.M_Data(exe.M_Data), .NZP(exe.NZP)
				); 

	Writeback	WB 		(	
					.clock(clock), .reset(wb.reset), .enable_writeback(wb.enable_writeback), 
					.W_Control(wb.W_Control), .aluout(wb.aluout), .memout(wb.memout), .pcout(wb.pcout), 
					.npc(wb.npc), .sr1(wb.sr1), .sr2(wb.sr2), .dr(wb.dr), .d1(wb.VSR1), .d2(wb.VSR2), .psr(wb.psr)
				);
				
	Controller_Pipeline Ctrl (
					.clock(clock), .reset(cntrl.reset), .IR(cntrl.IR), .complete_instr(cntrl.complete_instr),
					.complete_data(cntrl.complete_data), .NZP(cntrl.NZP), .psr(cntrl.psr), .br_taken(cntrl.br_taken),
					
					.enable_fetch(cntrl.enable_fetch), .enable_decode(cntrl.enable_decode), 
					.enable_execute(cntrl.enable_execute), .enable_writeback(cntrl.enable_writeback), 
					.enable_updatePC(cntrl.enable_updatePC), .mem_state(cntrl.mem_state)

				);

//initialsing the DUT RAM
initial begin
	fet.br_taken<=0;
	for(int i=0; i<8;i++) begin
		WB.RF.ram[i]=i;
	end
end
	
//getting value from register file at the DR location
always @(posedge clock) wb.dr_val = WB.RF.ram[wb.dr_wb];
	


	initial begin 
		clock = 0;
		
	end
	always #10 clock = ~clock;

	


endmodule
