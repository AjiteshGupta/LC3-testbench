//******** Documentation Section ***************
//Layered testbench for LC3 Microprocessor
// Author : Ajitesh Gupta
// Date : 12th July 2019



//---- INTERFACES -----
//INTERFACE FOR OVERALL LC3
interface all_in_one_if(input bit clock);
	logic reset;
	logic complete_instr, complete_data;
	logic [15:0] pc, Data_addr;
	logic [15:0] Instr_dout, Data_dout;
	
	logic instrmem_rd, Data_rd; 
	logic [15:0] Data_din;
	logic D_macc, I_macc;
	
	clocking cb1@(posedge clock);
		input pc, Data_addr;
		input instrmem_rd, Data_rd; 
		input Data_din;
		input D_macc, I_macc;
		output reset;
		output complete_instr, complete_data;
		output Instr_dout, Data_dout;
	endclocking
endinterface

//INTERFACE FOR FETCH UNIT
interface fetch_if(input bit clock);
	logic [15:0]npc_out,pc;
	logic instrmem_rd;
	logic br_taken;
	logic [15:0]taddr;
	logic enable_updatePC,enable_fetch;

	clocking cb @(posedge clock);
		output pc,npc_out;
		output  br_taken;
		output taddr;
		output enable_updatePC;
	endclocking
	modport TB(clocking cb,input instrmem_rd,output enable_fetch);
endinterface


//INTERFECE FOR DEOCDE UNIT
interface decode_if(input bit clock);
	logic  enable_decode;
	logic  [15:0] dout;
	logic  [15:0] npc_in;
	logic [1:0] 	W_Control; 
	logic	 Mem_Control;
	logic [5:0] 	E_Control;
	logic [15:0] IR;
	logic [15:0] npc_out;
	
	clocking cb@(posedge clock);
		output 	W_Control; 
		output 	 Mem_Control;
		output  	E_Control;
		output  IR;
		output  npc_out;
		output   enable_decode;
		output   dout;
		output   npc_in;
	endclocking
	modport TB(clocking cb);
endinterface

//INTERFACE FOR EXECUTE UNIT
interface execute_if(input bit clock);
	logic   enable_execute;
	logic   [1:0] W_Control_in;													
	logic   Mem_Control_in;													
	logic   [5:0] 	E_Control;
	logic   [15:0] IR;
	logic   [15:0] npc;
	logic  [15:0] VSR1, VSR2;
 
	logic  [15:0] aluout, pcout;
	logic  [1:0] W_Control_out;
	logic 	Mem_Control_out;
	logic  [2:0] NZP;
	logic  [2:0] sr1, sr2, dr;
	logic  [15:0] M_Data;

	clocking cb@(posedge clock);
		output  aluout, pcout;
		output  W_Control_out;
		output	Mem_Control_out,clock;
		output  NZP;
		output   dr;
		output M_Data;
		output   enable_execute;
		output   W_Control_in;													
		output  Mem_Control_in;													
		output   	E_Control;
		output   IR;
		output   npc;
	endclocking
	modport TB(clocking cb,input sr1,sr2,output VSR1,VSR2);
endinterface

//INTERFACE FOR WRITEBACK UNIT
interface writeback_if(input bit clock);
	logic enable_writeback;
  	logic 	 [15:0] aluout, memout, pcout, npc;
  	logic  [1:0] W_Control;
  	logic  [2:0] sr1, sr2, dr;     	
  	logic	[2:0] psr;
  	logic 	[15:0] d1, d2;				
	
	clocking cb@(posedge clock);
		input	 psr;
  		//input 	 d1, d2;	
		output  enable_writeback;
  		output 	  aluout, memout, pcout, npc;
  		output   W_Control;
  		output    dr;    			
	endclocking
	modport TB(clocking cb,output sr1,sr2,input d1,d2);
endinterface


//INTERFACE FOR CONTROLLER UNIT
interface controller_if(input bit clock);
	logic  complete_instr, complete_data;	
	logic  [15:0]IR;
	logic  [2:0]psr, NZP;
	
	logic enable_fetch, enable_decode, enable_execute, enable_writeback, enable_updatePC;
	logic [1:0] mem_state;
	logic br_taken;

	clocking cb@(posedge clock);
		input enable_fetch, enable_decode, enable_execute, enable_writeback, enable_updatePC;
		input  mem_state;
		input br_taken;
		output   complete_data;	
		output  IR;
		output  psr, NZP;
	endclocking
	modport TB(clocking cb);
endinterface



//INPUT PACKET
class instr_packet;
	rand logic [15:0]ir;
	constraint lea_not_op{	ir[15:12] inside {14,9};
				(ir[15:12]==14) -> (ir[8] dist{0:=5,1:=5});
				}

	constraint add_reg_op{	ir[15:12]==1;
				ir[5]==0;}
	constraint add_imm_op{	ir[15:12]==1;
				ir[5]==1;
				ir[4] dist{0:=5,1:=5};}

	constraint and_reg_op{	ir[15:12]==5;
				ir[5]==0;}
	constraint and_imm_op{	ir[15:12]==5;
				ir[5]==1;
				ir[4] dist{0:=5,1:=5};}
endclass
	
//----OUTPUT PACKETS-----
//OUTPUT PACKET FOR FETCH
class output_packet_fetch;
	reg [15:0]npc_out,pc;
	reg instrmem_rd;
endclass


//OUTPUT PACKET FOR DECODE
class output_packet_decode;
	reg [1:0] 	W_Control; 
	reg	 Mem_Control;
	reg [5:0] 	E_Control;
	reg [15:0] IR;
	reg [15:0] npc_out;
endclass

//OUTPUT PACKET FOR EXECUTE
class output_packet_execute;
 	reg  [15:0] aluout, pcout;
	reg  [1:0] W_Control_out;
	reg 	Mem_Control_out;
	reg  [2:0] NZP;
	reg [2:0] sr1, sr2, dr;
	reg [15:0] M_Data;
endclass

//OUTPUT PACKET FOR WRITEBACK
class output_packet_wb;     	
  	reg	[2:0] psr;
  	reg	[15:0] d1, d2;	
endclass

//OUTPUT PACKET FOR CONTROLLER
class output_packet_control;
	reg enable_fetch, enable_decode, enable_execute, enable_writeback, enable_updatePC;
endclass
reg 	[15:0] mem [0:7]='{8{10}};


//------------- GENERATOR --------------------
class Generator;	
	instr_packet  pkt2send;	
	static int count;
    	typedef mailbox #(instr_packet) in_box_type;
	in_box_type inbox;

	
	int 	number_packets;
	extern function new(int number_packets);
	extern virtual task gen();
	extern virtual task start();
endclass

function Generator::new(int number_packets);
	this.pkt2send = new();
	this.inbox = new;
	this.number_packets = number_packets;
endfunction

task Generator::gen();
	if (!pkt2send.randomize()) 
	begin
		$display(" \n%m\n[ERROR]%0d gen(): Randomization Failed!", $time);
		$finish;	
	end
endtask

task Generator::start();
	    
	  $display ($time, "ns:  [GENERATOR] Generator Started");
	
	  fork
		  for (int i=0; i<number_packets; i++) 
		  begin
			if(count<=number_packets/4)
			begin
			  pkt2send.constraint_mode(0);
			  pkt2send.lea_not_op.constraint_mode(1);
			end
			else if(count>number_packets*0.25 && count<=number_packets*0.4)
			begin
			  pkt2send.constraint_mode(0);
			  pkt2send.add_reg_op.constraint_mode(1);
			end
			else if(count>number_packets*0.4 && count<=number_packets*0.6)
			begin
			  pkt2send.constraint_mode(0);
			  pkt2send.add_imm_op.constraint_mode(1);
			end
			else if(count>number_packets*0.6 && count<=number_packets*0.8)
			begin
			  pkt2send.constraint_mode(0);
			  pkt2send.and_reg_op.constraint_mode(1);
			end
			else
			begin
			  pkt2send.constraint_mode(0);
			  pkt2send.and_imm_op.constraint_mode(1);
			end
			count++;
			gen();
			  begin 
			     instr_packet pkt = new pkt2send; 
				  inbox.put(pkt);
				$displayh("Generated pkt value=%h",pkt.ir); 
			  end
		  end
		  $display($time, "ns:  [GENERATOR] Generation Finished Creating %d Packets  ", number_packets);
      join_none
endtask

 // ---------------- DRIVER ---------------
class DriverBase;
	virtual all_in_one_if intr_a;
	instr_packet   pkt2send;	
	reg [15:0]payload_ir;
	typedef mailbox #(instr_packet) in_box_type;
  	in_box_type inbox = new();

	extern virtual task start();
	extern virtual task send();
	extern virtual task send_payload();
	extern function new(in_box_type inbox, virtual all_in_one_if intr_a);
endclass

function DriverBase::new(in_box_type inbox, virtual all_in_one_if intr_a);
  this.inbox = inbox;
  this.intr_a = intr_a;
endfunction

task DriverBase::send();
	send_payload();
endtask

task DriverBase::send_payload();
	$display($time, "ns:  [DRIVER] Sending Payload Begin");
	intr_a.cb1.Instr_dout <= payload_ir;	
	$displayh("time=%0d INPUT INSTRUCTION IN HEXADECIMAL=%h\n",$time,intr_a.cb1.Instr_dout);
			
endtask


task DriverBase::start();
	int packets_sent = 0;
	while(intr_a.reset==1)
		#20;
	$display ($time, "ns:  [DRIVER] Driver Started");
    fork
	    forever
	    begin
	      		inbox.get(pkt2send); 
			packets_sent++; 
			$display ($time, "[DRIVER] Sending in new packet BEGIN");
		  	this.payload_ir = pkt2send.ir;	
 	     		send();
	 		
			$display ($time, "ns:  [DRIVER] Sending in new packet END");
			$display ($time, "ns:  [DRIVER] Number of packets sent = %d", packets_sent);
			$display ($time,  "ns:  [DRIVER] The number of Packets in the Generator Mailbox = %d", inbox.num());
			if(inbox.num() == 0)
			begin
				break;
			end
			repeat(5)
		  	@(intr_a.cb1);
	    end
	join_none	
endtask


// --------------- RECEIVER -------------------
class ReceiverBase;
	virtual fetch_if.TB intr_f;
	virtual decode_if.TB intr_d;
	virtual execute_if.TB intr_e;
	virtual writeback_if.TB intr_w;
	virtual controller_if.TB intr_c;
	virtual all_in_one_if intr_a;

	
	output_packet_fetch  	pkt_cmp_f;	
	output_packet_decode 	pkt_cmp_d;
	output_packet_execute 	pkt_cmp_e;
	output_packet_wb	pkt_cmp_w;
	output_packet_control	pkt_cmp_c;

	typedef mailbox #(output_packet_fetch) rx_box_type_f;
  	rx_box_type_f	rx_out_box_f;	

	typedef mailbox #(output_packet_decode) rx_box_type_d;
  	rx_box_type_d	rx_out_box_d;

	typedef mailbox #(output_packet_execute) rx_box_type_e;
  	rx_box_type_e	rx_out_box_e;	

	typedef mailbox #(output_packet_wb) rx_box_type_wb;
  	rx_box_type_wb	rx_out_box_w;

	typedef mailbox #(output_packet_control) rx_box_type_c;
  	rx_box_type_c	rx_out_box_c;	


	reg [15:0]npc_out_cmp,pc_cmp;
	reg instrmem_rd_cmp;

	reg [1:0] 	W_Control_cmp; 
	reg	 Mem_Control_cmp;
	reg [5:0] 	E_Control_cmp;
	reg [15:0] IR_cmp;
	reg [15:0] npc_out_d_cmp;

 	reg  [15:0] aluout_cmp, pcout_cmp;
	reg  [1:0] W_Control_out_cmp;
	reg 	Mem_Control_out_cmp;
	reg  [2:0] NZP_cmp;
	reg [2:0] sr1_cmp, sr2_cmp, dr_cmp;
	reg [15:0] M_Data_cmp;
   	
  	reg	[2:0] psr_cmp;
  	reg	[15:0] d1_cmp, d2_cmp;	

	reg enable_fetch_cmp, enable_decode_cmp, enable_execute_cmp, enable_writeback_cmp, enable_updatePC_cmp;
	int pkt_cnt = 0;
	

	extern function new(virtual fetch_if.TB intr_f,virtual decode_if.TB intr_d,virtual execute_if.TB intr_e,virtual writeback_if.TB intr_w,virtual controller_if.TB intr_c,virtual all_in_one_if intr_a,
				rx_box_type_f rx_out_box_f, rx_box_type_d rx_out_box_d, rx_box_type_e rx_out_box_e,
				rx_box_type_wb rx_out_box_w, rx_box_type_c rx_out_box_c );
	extern virtual task start();
	extern virtual task recv_f();
	extern virtual task recv_d();
	extern virtual task recv_e();
	extern virtual task recv_w();
	extern virtual task recv_c();
	extern virtual task get_payload();
endclass


function ReceiverBase::new(	virtual fetch_if.TB intr_f,
				virtual decode_if.TB intr_d,
				virtual execute_if.TB intr_e,
				virtual writeback_if.TB intr_w,
				virtual controller_if.TB intr_c,
				virtual all_in_one_if intr_a,
				rx_box_type_f rx_out_box_f, rx_box_type_d rx_out_box_d, rx_box_type_e rx_out_box_e,
				rx_box_type_wb rx_out_box_w, rx_box_type_c rx_out_box_c );
	
	this.intr_f = intr_f;
	this.intr_d = intr_d;
	this.intr_e = intr_e;
	this.intr_w = intr_w;
	this.intr_c = intr_c;
	this.intr_a = intr_a;
	this.rx_out_box_f = rx_out_box_f;
	this.rx_out_box_d = rx_out_box_d;
	this.rx_out_box_e = rx_out_box_e;
	this.rx_out_box_w = rx_out_box_w;
	this.rx_out_box_c = rx_out_box_c;
	pkt_cmp_f = new();
	pkt_cmp_d = new();
	pkt_cmp_e = new();
	pkt_cmp_w = new();
	pkt_cmp_c = new();
	
endfunction


task ReceiverBase::recv_f();
	get_payload();
	
	pkt_cmp_f.npc_out = npc_out_cmp;
	pkt_cmp_f.pc = pc_cmp ;
	pkt_cmp_f.instrmem_rd = instrmem_rd_cmp ;
endtask


task ReceiverBase::recv_d();
	get_payload();
	
	pkt_cmp_d.W_Control = W_Control_cmp; 
	pkt_cmp_d.Mem_Control = Mem_Control_cmp;
	pkt_cmp_d.E_Control = E_Control_cmp;
	pkt_cmp_d.IR = IR_cmp;
	pkt_cmp_d.npc_out = npc_out_d_cmp;
endtask


task ReceiverBase::recv_e();
	get_payload();
	
 	pkt_cmp_e.aluout = aluout_cmp;
	pkt_cmp_e.pcout = pcout_cmp;
	pkt_cmp_e.W_Control_out = W_Control_out_cmp;
	pkt_cmp_e.Mem_Control_out = Mem_Control_out_cmp;
	pkt_cmp_e.NZP = NZP_cmp;
	pkt_cmp_e.sr1 = sr1_cmp;
	pkt_cmp_e.sr2 = sr2_cmp;
	pkt_cmp_e.dr = dr_cmp;
	pkt_cmp_e.M_Data = M_Data_cmp;
endtask


task ReceiverBase::recv_w();
	get_payload();
	
  	pkt_cmp_w.psr = psr_cmp;
  	pkt_cmp_w.d1 = d1_cmp;
	pkt_cmp_w.d2 = d2_cmp;	
endtask


task ReceiverBase::recv_c();
	pkt_cmp_c.enable_fetch = enable_fetch_cmp;
	pkt_cmp_c.enable_decode = enable_decode_cmp;
	pkt_cmp_c.enable_execute = enable_execute_cmp;
	pkt_cmp_c.enable_writeback = enable_writeback_cmp;
	pkt_cmp_c.enable_updatePC = enable_updatePC_cmp;
endtask


task ReceiverBase::get_payload();
	
	npc_out_cmp = intr_f.cb.npc_out;
	pc_cmp = intr_f.cb.pc;
	instrmem_rd_cmp = intr_f.instrmem_rd;

	W_Control_cmp = intr_d.cb.W_Control; 
	Mem_Control_cmp = intr_d.cb.Mem_Control;
	E_Control_cmp = intr_d.cb.E_Control;
	IR_cmp = intr_d.cb.IR;
	npc_out_d_cmp = intr_d.cb.npc_out;

 	aluout_cmp = intr_e.cb.aluout;
	pcout_cmp = intr_e.cb.pcout;
	W_Control_out_cmp = intr_e.cb.W_Control_out;
	Mem_Control_out_cmp = intr_e.cb.Mem_Control_out;
	NZP_cmp = intr_e.cb.NZP;
	sr1_cmp = intr_e.sr1;
	sr2_cmp = intr_e.sr2;
	dr_cmp = intr_e.cb.dr;
	M_Data_cmp = intr_e.cb.M_Data;
   	
  	psr_cmp = intr_w.cb.psr;
  	d1_cmp = intr_w.d1;
	d2_cmp = intr_w.d2;	

	enable_fetch_cmp = intr_c.cb.enable_fetch;
	enable_decode_cmp = intr_c.cb.enable_decode;
	enable_execute_cmp = intr_c.cb.enable_execute;
	enable_writeback_cmp = intr_c.cb.enable_writeback;
	enable_updatePC_cmp = intr_c.cb.enable_updatePC;
endtask


task ReceiverBase::start();
	$display($time, "ns:  [RECEIVER]  RECEIVER STARTED");
	
	fork
		forever
		begin
			@(intr_a.cb1);
			#1 recv_f();
			recv_c();
			rx_out_box_c.put(pkt_cmp_c);
			rx_out_box_f.put(pkt_cmp_f);
			$display("time=%0d",$time," values in fetch packet i.e. output from DUT pc=%h npc=%h instrmem_rd=%h",pkt_cmp_f.pc,pkt_cmp_f.npc_out,pkt_cmp_f.instrmem_rd);
			
			@(intr_a.cb1);
			#1 recv_d();
			recv_c();
			rx_out_box_c.put(pkt_cmp_c);
			rx_out_box_d.put(pkt_cmp_d);
			$display("time=%0d",$time," values in decode packet i.e. output from DUT ir=%h E_Control=%b W_Control=%d npc=%h M_Control=%b",pkt_cmp_d.IR,pkt_cmp_d.E_Control,pkt_cmp_d.W_Control,pkt_cmp_d.npc_out,pkt_cmp_d.Mem_Control);
			
			@(intr_a.cb1);
			#1 recv_e();
			recv_c();
			rx_out_box_c.put(pkt_cmp_c);
			rx_out_box_e.put(pkt_cmp_e);
			$display("time=%0d",$time," values in execute packet i.e. output from DUT  aluout=%h, pcout=%h	W_Control_out=%b Mem_Control_out=%b NZP=%b sr1=%d, sr2=%d, dr=%d",pkt_cmp_e.aluout,pkt_cmp_e.pcout,
			pkt_cmp_e.W_Control_out,pkt_cmp_e.Mem_Control_out,pkt_cmp_e.NZP,pkt_cmp_e.sr1,pkt_cmp_e.sr2, pkt_cmp_e.dr);
			
			@(intr_a.cb1);
			#1 recv_w();
			recv_c();
			rx_out_box_c.put(pkt_cmp_c);
			rx_out_box_w.put(pkt_cmp_w);
			$display("time=%0d",$time," values in writeback packet i.e. output from DUT psr=%b d1=%h d2=%h",pkt_cmp_w.psr,pkt_cmp_w.d1,pkt_cmp_w.d2);
			
			@(intr_a.cb1);
			#1 recv_c();
			   recv_f();
			rx_out_box_c.put(pkt_cmp_c);
			rx_out_box_f.put(pkt_cmp_f);
			$display("time=%0d",$time," values in fetch packet i.e. output from DUT pc=%h npc=%h instrmem_rd=%h",pkt_cmp_f.pc,pkt_cmp_f.npc_out,pkt_cmp_f.instrmem_rd);
			$display($time, "ns:   [RECEIVER -> GETPAYLOAD]   Payload Obtained");
		end	
	join_none
endtask


// ----------------------- SCOREBOARD -------------------
class Scoreboard;
	virtual fetch_if.TB intr_f;
	virtual decode_if.TB intr_d;
	virtual execute_if.TB intr_e;
	virtual writeback_if.TB intr_w;
	virtual controller_if.TB intr_c;
	virtual all_in_one_if intr_a;

	output_packet_fetch  	pkt_cmp_f=new();	
	output_packet_decode 	pkt_cmp_d=new();
	output_packet_execute 	pkt_cmp_e=new();
	output_packet_wb	pkt_cmp_w=new();
	output_packet_control	pkt_cmp_c=new();	

  	typedef mailbox #(output_packet_fetch) rx_box_type_f;
  	rx_box_type_f	rx_out_mbox_f;	

	typedef mailbox #(output_packet_decode) rx_box_type_d;
  	rx_box_type_d	rx_out_mbox_d;

	typedef mailbox #(output_packet_execute) rx_box_type_e;
  	rx_box_type_e	rx_out_mbox_e;	

	typedef mailbox #(output_packet_wb) rx_box_type_wb;
  	rx_box_type_wb	rx_out_mbox_w;

	typedef mailbox #(output_packet_control) rx_box_type_c;
  	rx_box_type_c	rx_out_mbox_c;	
	
	typedef enum {idle,fetch,decode,execute,writeback,updatepc}st;
	st ps=idle;
	st ns;
	
		
	reg [15:0]npc_out_chk,pc_chk;
	reg instrmem_rd_chk;

	reg [1:0] 	W_Control_chk; 
	reg	 Mem_Control_chk;
	reg [5:0] 	E_Control_chk;
	reg [15:0] IR_chk;
	reg [15:0] npc_out_d_chk;

 	reg  [15:0] aluout_chk, pcout_chk;
	reg  [1:0] W_Control_out_chk;
	reg 	Mem_Control_out_chk;
	reg  [2:0] NZP_chk;
	reg [2:0] sr1_chk, sr2_chk, dr_chk;
	reg [15:0] M_Data_chk;
   	
  	reg	[2:0] psr_chk;
  	reg	[15:0] d1_chk, d2_chk;	

	reg [15:0]npc_out_cmp,pc_cmp;
	reg instrmem_rd_cmp;
	reg [15:0]d_in;
	reg enable_fetch_chk, enable_decode_chk, enable_execute_chk, enable_writeback_chk, enable_updatePC_chk;
	
	extern function new( 	virtual fetch_if.TB intr_f,virtual decode_if.TB intr_d,virtual execute_if.TB intr_e,virtual writeback_if.TB intr_w,virtual controller_if.TB intr_c,virtual all_in_one_if intr_a,
				rx_box_type_f rx_out_mbox_f = null, rx_box_type_d rx_out_mbox_d = null, rx_box_type_e rx_out_mbox_e = null, rx_box_type_wb rx_out_mbox_w = null,rx_box_type_c rx_out_mbox_c = null);
	extern virtual task start();
	extern virtual task check();
	extern virtual task send_correct_data();
	extern virtual task check_fetch();
	extern virtual task check_decode();
	extern virtual task check_execute();
	extern virtual task check_writeback();
	extern virtual task check_control();
	extern virtual task golden_fetch();
	extern virtual task golden_decode();
	extern virtual task golden_execute();
	extern virtual task golden_writeback();
	extern virtual task golden_control();
	extern virtual task reset_s();
endclass

task Scoreboard::reset_s();
	pc_chk=16'h3000;
	npc_out_chk=16'h3001;
	enable_fetch_chk=1;
	instrmem_rd_chk=1;
	ns=fetch;
	ps=fetch;
	intr_f.enable_fetch <= 1;	
	intr_f.cb.taddr <= 0;
	intr_f.cb.br_taken <= 0;
	enable_decode_chk <= 0;
	enable_execute_chk <= 0;
	enable_writeback_chk <= 0;
	enable_updatePC_chk <= 0;
	test_top_level_12.l11.WB.RF.ram ='{8{10}};
endtask

function Scoreboard::new( 	virtual fetch_if.TB intr_f,
				virtual decode_if.TB intr_d,
				virtual execute_if.TB intr_e,
				virtual writeback_if.TB intr_w,
				virtual controller_if.TB intr_c,
				virtual all_in_one_if intr_a,
				rx_box_type_f rx_out_mbox_f = null, rx_box_type_d rx_out_mbox_d = null, rx_box_type_e rx_out_mbox_e = null, rx_box_type_wb rx_out_mbox_w = null,rx_box_type_c rx_out_mbox_c = null);
	
	if (rx_out_mbox_f == null) 
		rx_out_mbox_f = new();

	if (rx_out_mbox_d == null) 
		rx_out_mbox_d = new();

	if (rx_out_mbox_e == null) 
		rx_out_mbox_e = new();

	if (rx_out_mbox_w == null) 
		rx_out_mbox_w = new();

	if (rx_out_mbox_c == null) 
		rx_out_mbox_c = new();
	
	this.rx_out_mbox_f = rx_out_mbox_f;
	this.rx_out_mbox_d = rx_out_mbox_d;
	this.rx_out_mbox_e = rx_out_mbox_e;
	this.rx_out_mbox_w = rx_out_mbox_w;
	this.rx_out_mbox_c = rx_out_mbox_c;
	this.intr_f = intr_f;
	this.intr_d = intr_d;
	this.intr_e = intr_e;
	this.intr_w = intr_w;
	this.intr_c = intr_c;
	this.intr_a = intr_a;
endfunction

		
task Scoreboard::start();
	$display ($time, "ns:  [SCOREBOARD] Scoreboard Started");
	E_Control_chk=0;
	W_Control_chk=0;
	Mem_Control_chk=0;
	aluout_chk=0;
	pcout_chk=0;
	W_Control_out_chk=0;
	intr_a.cb1.complete_instr <= 1;
	
	fork
		forever @(sr1_chk) d1_chk=mem[sr1_chk];
		forever @(sr2_chk) d2_chk=mem[sr2_chk];
		forever golden_control();
		forever check();
	join_none
	$display ($time, "[SCOREBOARD] Forking of Process Finished");
endtask

task Scoreboard::send_correct_data();
	intr_f.cb.enable_updatePC <= enable_updatePC_chk;
	intr_f.enable_fetch <= enable_fetch_chk;
	

	intr_d.cb.enable_decode <= enable_decode_chk;
	intr_d.cb.npc_in <= npc_out_chk;
	
	intr_e.cb.enable_execute <= enable_execute_chk;
	intr_e.cb.E_Control <= E_Control_chk;
	intr_e.cb.W_Control_in <= W_Control_chk;
	intr_e.cb.Mem_Control_in <= Mem_Control_chk;
	intr_e.cb.IR <= IR_chk;
	intr_e.cb.npc <= npc_out_d_chk;
	intr_e.VSR1 <= d1_chk;
	intr_e.VSR2 <= d2_chk;
	
	intr_w.cb.enable_writeback <= enable_writeback_chk;
	intr_w.sr1 <= sr1_chk;
	intr_w.sr2 <= sr2_chk;
	intr_w.cb.dr <= dr_chk;
	intr_w.cb.aluout <= aluout_chk;
	intr_w.cb.pcout <= pcout_chk;
	intr_w.cb.W_Control <= W_Control_out_chk;
	intr_w.cb.memout <= 0;
	
endtask

task Scoreboard::check();
		@(intr_a.cb1);
		 golden_fetch();
		#2 rx_out_mbox_f.try_get(pkt_cmp_f);
		rx_out_mbox_c.try_get(pkt_cmp_c);
		check_fetch();
		check_control();
		
		@(intr_a.cb1);
		golden_decode();
		#2 rx_out_mbox_d.try_get(pkt_cmp_d);
		rx_out_mbox_c.try_get(pkt_cmp_c);
		check_decode();
		check_control();

		@(intr_a.cb1);
		golden_execute();
		#2 rx_out_mbox_e.try_get(pkt_cmp_e);
		rx_out_mbox_c.try_get(pkt_cmp_c);
		check_execute();
		check_control();

		@(intr_a.cb1);
		golden_writeback();
		#2 rx_out_mbox_w.try_get(pkt_cmp_w);
		rx_out_mbox_c.try_get(pkt_cmp_c);
		check_writeback();
		check_control();

		@(intr_a.cb1);
		golden_fetch();
		#2 rx_out_mbox_c.try_get(pkt_cmp_c);
		rx_out_mbox_f.try_get(pkt_cmp_f);
		check_control();	
		check_fetch();
endtask


task Scoreboard::check_fetch();
	if(instrmem_rd_chk!==pkt_cmp_f.instrmem_rd)
		$display("time= %0d",$time," -------- BUG BUG [FETCH MODULE] IN[INSTRMEM_RD] -------- expected instrmem_rd:%h actual instrmem_rd:%h\n",instrmem_rd_chk,pkt_cmp_f.instrmem_rd);
	
	if(pc_chk!=pkt_cmp_f.pc)
		$display("time= %0d",$time," -------- BUG BUG [FETCH MODULE] IN [PC] -------- expected pc:%h actual pc:%h\n ",pc_chk,pkt_cmp_f.pc);
	
	if(npc_out_chk!= pkt_cmp_f.npc_out)
		$display("time= %0d",$time," -------- BUG BUG [FETCH MODULE] IN [NPC] -------- expected npc:%h actual npc:%h\n",npc_out_chk,pkt_cmp_f.npc_out);

endtask

task Scoreboard::check_decode();
	if(pkt_cmp_d.W_Control!=?W_Control_chk)
		$display("time= %0d",$time," -------- BUG BUG [DECODE MODULE] IN [W_CONTROL] -------- expected W_control:%h actual W_control:%h\n ",W_Control_chk,pkt_cmp_d.W_Control);
	
	if(pkt_cmp_d.E_Control!=?E_Control_chk)
		$display("time= %0d",$time," -------- BUG BUG [DECODE MODULE] IN [E_Control] -------- expected E_control:%b actual E_control:%b\n ",E_Control_chk,pkt_cmp_d.E_Control);
	
	if(Mem_Control_chk!=pkt_cmp_d.Mem_Control)
		$display("time= %0d",$time," -------- BUG BUG [DECODE MODULE] IN [M_Control] -------- expected M_control:%h actual M_control:%h\n ",Mem_Control_chk,pkt_cmp_d.Mem_Control);
	
	if(npc_out_d_chk!=pkt_cmp_d.npc_out)
		$display("time= %0d",$time," -------- BUG BUG [DECODE MODULE] IN [NPC_OUT] -------- expected npc_out:%h actual npc_out:%h\n ",npc_out_d_chk,pkt_cmp_d.npc_out);
	
	if(IR_chk!=pkt_cmp_d.IR)
		$display("time= %0d",$time," -------- BUG BUG [DECODE MODULE] IN [IR] -------- expected IR:%h actual IR:%h \n",IR_chk,pkt_cmp_d.IR);
endtask

task Scoreboard::check_execute();

	if(aluout_chk!==pkt_cmp_e.aluout && intr_e.cb.W_Control_in==2'b00)
		$display("Time=%0d, -------- BUG BUG [EXECUTE MODULE] IN [ALUOUT] ------- expected aluout:%h actual aluout:%h\n",$time,aluout_chk,pkt_cmp_e.aluout);
	
	if(pcout_chk!==pkt_cmp_e.pcout && intr_e.cb.W_Control_in==2'b10)
		$display("Time=%0d, -------- BUG BUG [EXECUTE MODULE] IN [PCOUT] -------- expected pcout:%h actual pcout:%h\n",$time,pcout_chk,pkt_cmp_e.pcout);
	
	if(dr_chk!==pkt_cmp_e.dr)
		$display("Time=%0d, -------- BUG BUG [EXECUTE MODULE] IN [DR_OUT] ---------   expected dr_out:%h actual dr_out:%h\n",$time,dr_chk,pkt_cmp_e.dr);
	
	if(W_Control_out_chk!==pkt_cmp_e.W_Control_out)
		$display("time=%0d, --------- BUG BUG [EXECUTE MODULE] IN [W_CONTROL_OUT] --------  expected W_Control_out:%0d actual W_Control_out:%0d\n",$time,W_Control_out_chk,pkt_cmp_e.W_Control_out);
	
	if(sr1_chk!==pkt_cmp_e.sr1)
		$display("Time=%0d, --------- BUG BUG [EXECUTE MODULE] IN [sr1] ---------  expected sr1:%0d  actual sr1:%0d\n",$time,sr1_chk,pkt_cmp_e.sr1);
	
	if(sr2_chk!==pkt_cmp_e.sr2)
		$display("Time=%0d, --------- BUG BUG [EXECUTE MODULE] IN [sr2] --------- expected sr2:%0d actual sr2:%0d\n",$time,sr2_chk,pkt_cmp_e.sr2);
	
endtask

task Scoreboard::check_writeback();
 	if(pkt_cmp_w.psr!==psr_chk)
		$display("time=%0d ---------- BUG BUG [WRITEBACK MODULE] IN [PSR] ---------- expected value=%b  actual value=%b\n",$time,psr_chk,pkt_cmp_w.psr);
	
	if(d1_chk!==pkt_cmp_w.d1) 
	begin
		$display("time=%0d ---------- BUG BUG [WRITEBACK MODULE] IN [VSR1] ---------- expected value=%h  actual value=%h",$time,d1_chk,pkt_cmp_w.d1);
		$displayh(" time=%0d --------- MEMORY DATA --------- expected value=%p  actual value=%p\n",$time,mem,test_top_level_12.l11.WB.RF.ram);
	end

	if(d2_chk!==pkt_cmp_w.d2) 
	begin
		$display("time=%0d ---------- BUG BUG [WRITEBACK MODULE] IN [VSR2] ---------- expected value=%h  actual value=%h",$time,d2_chk,pkt_cmp_w.d2);
		$displayh(" time=%0d --------- MEMORY DATA --------- expected value=%p  actual value=%p\n",$time,mem,test_top_level_12.l11.WB.RF.ram);
	end

	if(test_top_level_12.l11.WB.RF.ram[dr_chk]!==mem[dr_chk])
		$displayh(" time=%0d --------- BUG BUG [WRITEBACK MODULE] IN [WRITING DATA] --------- expected value=%p  actual value=%p\n",$time,mem,test_top_level_12.l11.WB.RF.ram);
	
endtask	


task Scoreboard::check_control();
 	if(enable_fetch_chk!==intr_c.cb.enable_fetch)
		$display("time=%0d ------------ BUG BUG [CONTROLLER MODULE] IN  %s state [enable_fetch] ---------- expected enable_fetch=%b  actual enable_fetch=%b  ps=%s complete_instr=%d\n",$time,ps,enable_fetch_chk,intr_c.cb.enable_fetch,ps,intr_a.complete_instr);
	
	if(enable_decode_chk!==intr_c.cb.enable_decode)
		$display("time=%0d -------------- BUG BUG [CONTROLLER MODULE] IN  %s state [enable_decode] ----------- expected enable_decode=%b  actual enable_decode=%b ps=%s\n",$time,ps,enable_decode_chk,intr_c.cb.enable_decode,ps);
	
	if(enable_execute_chk!==intr_c.cb.enable_execute)
		$display("time=%0d -------------- BUG BUG [CONTROLLER MODULE] IN   %s state [enable_execute] ----------- expected enable_execute=%b  actual enable_execute=%b ps=%s\n",$time,ps,enable_execute_chk,intr_c.cb.enable_execute,ps);
	
	if(enable_writeback_chk!==intr_c.cb.enable_writeback)
		$display("time=%0d -------------- BUG BUG [CONTROLLER MODULE] IN   %s state [enable_writeback] ---------- expected enable_writeback=%b  actual enable_writeback=%b ps=%s\n",$time,ps,enable_writeback_chk,intr_c.cb.enable_writeback,ps);
	
	if(enable_updatePC_chk!==intr_c.cb.enable_updatePC)
		$display("time=%0d -------------- BUG BUG [CONTROLLER MODULE] IN  %s state [enable_updatePC] --------- expected enable_updatePC=%b  actual enable_updatePC=%b ps=%s\n",$time,ps,enable_updatePC_chk,intr_c.cb.enable_updatePC,ps);
endtask

task Scoreboard::golden_fetch();

	if(intr_a.cb1.reset)
	begin	
		pc_chk=16'h3000;
		npc_out_chk=16'h3001;
	end
		
	else 
	begin
		if(enable_updatePC_chk)
		begin
			pc_chk=npc_out_chk;
			npc_out_chk=npc_out_chk+1;
		end
	end
	$display("INPUTS AT FETCH UNIT----- enable_fetch=%b enable_updatePC=%d",enable_fetch_chk,enable_updatePC_chk);
endtask

task Scoreboard::golden_decode();
	if(intr_a.cb1.reset)
	begin	
		Mem_Control_chk=16'h0000;
		E_Control_chk=16'h0000;
		IR_chk=0;
		W_Control_chk=0;
		npc_out_d_chk=0;
	end
	else 
	begin
		if(enable_decode_chk)
		begin	
			W_Control_chk=0;
			case(intr_a.cb1.Instr_dout[15:12])
			4'h1: 
				begin
					if(intr_a.cb1.Instr_dout[5])
						E_Control_chk=6'b00???0;
					else
						E_Control_chk=6'b00???1;
				end
			4'h5:
				begin
					if(intr_a.cb1.Instr_dout[5])
						E_Control_chk=6'b01???0;
					else
						E_Control_chk=6'b01???1;
				end
			4'h9:
				E_Control_chk=6'b10????;
			4'he:
				begin
					W_Control_chk=2;
					E_Control_chk=6'b??011?;
				end
			endcase
			IR_chk=intr_a.cb1.Instr_dout;
			npc_out_d_chk=intr_d.cb.npc_in;
			Mem_Control_chk=0;
		end
	end
	sr1_chk=intr_d.cb.IR[8:6];
	sr2_chk=W_Control_chk?'0:intr_d.cb.IR[2:0]; 
	$display("INPUTS AT DECODE UNIT----- reset=%b dout=%h npc_in=%h enable_decode=%d",intr_a.cb1.reset,intr_a.cb1.Instr_dout,npc_out_chk,enable_decode_chk);
endtask

task Scoreboard::golden_execute();
	reg op2select,pcselect2;
	reg [1:0]pcselect1,alu_control;
	reg [15:0]aluin1,aluin2;
	reg signed [15:0] imm5,offset6,offset9,offset11;
	reg signed [15:0]ext_out;
	reg [15:0] pcsel2_out;
	//sr1_chk=intr_e.cb.IR[8:6];
	//sr2_chk=intr_e.cb.W_Control_in?'0:intr_e.cb.IR[2:0]; 
	
	
	{alu_control,pcselect1,pcselect2,op2select}=intr_e.cb.E_Control;
	imm5={{11{intr_e.cb.IR[4]}},intr_e.cb.IR[4:0]};
	offset6={{10{intr_e.cb.IR[5]}},intr_e.cb.IR[5:0]};
	offset9={{7{intr_e.cb.IR[8]}},intr_e.cb.IR[8:0]};
	offset11={{5{intr_e.cb.IR[10]}},intr_e.cb.IR[10:0]};
	aluin1=intr_e.VSR1;
	case(pcselect1)
	0: ext_out=offset11;
	1: ext_out=offset9;
	2: ext_out=offset6;
	3: ext_out=0;
	endcase

	case(pcselect2)
	0: pcsel2_out=intr_e.VSR1;
	1: pcsel2_out=intr_e.cb.npc;
	endcase

	case(op2select)
	0: aluin2=imm5;
	1: aluin2=intr_e.VSR2;
	endcase
		
	if(intr_a.cb1.reset)
	begin
		aluout_chk=0;
		pcout_chk=0;
		W_Control_out_chk=0;
		dr_chk=0;
	end
	else 
	begin
		if(enable_execute_chk)
		begin			
			case(alu_control)
			0:aluout_chk=aluin2+aluin1;
			1:aluout_chk=aluin2&aluin1;
			2:aluout_chk=~intr_e.VSR1;
			endcase
				
			pcout_chk=ext_out+pcsel2_out;
			dr_chk=intr_e.cb.IR[11:9];
			W_Control_out_chk=intr_e.cb.W_Control_in;
		end
	end
	
	case(intr_e.cb.W_Control_in)
	0: d_in=aluout_chk;
	//1: d_in=memout_chk;
	2: d_in=pcout_chk;
	endcase
	
	$display("INPUTS AT EXECUTE UNIT----- reset=%b E_Control=%b W_Control=%d IR=%h npc_in=%h enable_execute=%d",intr_a.cb1.reset,E_Control_chk,W_Control_chk,IR_chk,npc_out_d_chk,enable_execute_chk);
endtask

task Scoreboard::golden_writeback();
	mem[dr_chk]=d_in;
	if(intr_a.cb1.reset)
		psr_chk=3'b000;
	else
	begin
		if(enable_writeback_chk)
		begin
			if(d_in===0)	
				psr_chk=3'b010;
			else if(d_in[15]==0)
				psr_chk=3'b001;	
			else if(d_in[15]==1)
				psr_chk=3'b100;
		end
	end
	if(dr_chk==sr1_chk || dr_chk ==sr2_chk)
	begin
		d1_chk = mem[sr1_chk];
		d2_chk = mem[sr2_chk];
	end
	$display("INPUTS AT WRITEBACK UNIT----- reset=%b W_Control=%d aluout=%h pcout=%h dr=%d sr1=%d sr2=%d enable_writeback=%d",intr_a.cb1.reset,W_Control_out_chk,aluout_chk,pcout_chk,dr_chk, sr1_chk,sr2_chk,enable_writeback_chk);

endtask

task Scoreboard::golden_control();
	@(intr_a.cb1);
	#0 case(ps)
	idle: begin
		if(intr_a.cb1.reset)
		ns=fetch;
		end
	fetch: begin
		if(intr_a.cb1.complete_instr)
		ns=decode;
		if(intr_a.cb1.reset)
		ns=fetch;
		end
	decode: begin
		ns=execute;
		if(intr_a.cb1.reset)
		ns=fetch;
		end
	execute: begin
		ns=writeback;
		if(intr_a.cb1.reset)
		ns=fetch;
		end
	writeback: begin
		ns=updatepc;
		if(intr_a.cb1.reset)
		ns=fetch;
		end
	updatepc: begin
		ns=fetch;
		end
	endcase
	
	ps=ns;
	case(ps)
	idle: begin
		enable_fetch_chk='x;
		end
	fetch: begin
		enable_fetch_chk=1;
		enable_updatePC_chk=0;
		enable_decode_chk=0;
		enable_execute_chk=0;
		enable_writeback_chk=0;
		instrmem_rd_chk=1;
		end
	decode: begin
		enable_fetch_chk=0;
		enable_decode_chk=1;
		instrmem_rd_chk=1'bz;
		end
	execute: begin
		enable_execute_chk=1;
		enable_decode_chk=0;
		end
	writeback: begin
		enable_writeback_chk=1;
		enable_execute_chk=0;		
		end
	updatepc: begin
		enable_writeback_chk=0;
		enable_updatePC_chk=1;
		end
	endcase
	send_correct_data();
endtask

//----------- TESTBENCH --------------------------
program test_top_overall( fetch_if.TB intr_f, decode_if.TB intr_d, execute_if.TB intr_e,
			  writeback_if.TB intr_w, controller_if.TB intr_c, all_in_one_if intr_a);
	

	Generator  	generator;	// generator object
	DriverBase     	drvr;		// driver objects
	Scoreboard 	sb;		// scoreboard object
	ReceiverBase 	rcvr;		// Receiver Object

	int 	number_packets;
	
	initial begin
		number_packets =50;
        	generator = new(number_packets);
		sb = new(intr_f,intr_d,intr_e,intr_w,intr_c,intr_a);  
		drvr = new(generator.inbox, intr_a);
		rcvr = new(intr_f,intr_d,intr_e,intr_w,intr_c,intr_a, sb.rx_out_mbox_f, sb.rx_out_mbox_d, sb.rx_out_mbox_e,
			   sb.rx_out_mbox_w,sb.rx_out_mbox_c);
		reset();
		generator.start(); 
		drvr.start(); 
		sb.start(); 
		rcvr.start();
    	repeat(number_packets*5) @(intr_a.cb1);
		
  	end

	task reset();
		$display ($time, "ns:  [RESET]  Design Reset Start");
		intr_a.cb1.reset <= 1'b1; 
		sb.reset_s();
		repeat(5) @(intr_a.cb1);
		intr_a.cb1.reset <= 1'b0;
		$display ($time, "ns:  [RESET]  Design Reset End");
	endtask
	
endprogram


//----------------- LC3 TOP ----------------------
module LC3_try (	 fetch_if.TB intr_f,
   			  decode_if.TB intr_d, execute_if.TB intr_e,
			  writeback_if.TB intr_w, controller_if.TB intr_c, all_in_one_if intr_a,
			input clock, reset,complete_instr, complete_data,output[15:0]   Data_rd,pc,Data_addr,Data_din, output  D_macc, I_macc,instrmem_rd,input [15:0] Instr_dout, Data_dout    
			
			);
	//input			clock, reset;
	//input			complete_instr, complete_data;
	//output	[15:0] 	pc, Data_addr;
	//input	[15:0]	Instr_dout, Data_dout;
	
	//output			instrmem_rd, Data_rd; 
	//output	[15:0]	Data_din;
	//output			D_macc, I_macc;
	
	
	wire			enable_updatePC, br_taken,  enable_decode, enable_execute, enable_writeback, enable_fetch;
	wire	[15:0]	npc_out_fetch, taddr, IR, npc_out_dec; 
	wire	[5:0] 	E_Control;
	wire 	[1:0] 	W_Control;													
	wire 			Mem_Control;													


	wire	[15:0]	VSR1, VSR2, aluout, pcout;
	wire	[2:0] 	psr;
   	wire	[1:0] 	W_Control_out;
   	wire			Mem_Control_out;
   	
   	wire	[2:0]	sr1, sr2, dr, NZP;
   	
   	wire	[1:0]	mem_state;
   	wire			M_Control;
   	wire	[15:0]	M_Data, memout;
   	
   	assign 	I_macc = enable_fetch;
	
	Fetch	Fetch (
					.clock(clock), .reset(intr_a.cb1.reset), .enable_updatePC(intr_f.cb.enable_updatePC), 
					.enable_fetch(intr_f.enable_fetch), .pc(intr_f.cb.pc), .npc_out(intr_f.cb.npc_out), 
					.instrmem_rd(intr_f.instrmem_rd), .taddr(intr_f.cb.taddr), .br_taken(intr_f.cb.br_taken)
				);

	Decode  Dec (
					.clock(clock), .reset(intr_a.cb1.reset), .enable_decode(intr_c.cb.enable_decode), 
					.dout(intr_a.cb1.Instr_dout), .E_Control(intr_d.cb.E_Control), .npc_in(intr_d.cb.npc_in), 
					.Mem_Control(intr_d.cb.Mem_Control), .W_Control(intr_d.cb.W_Control), 
					.IR(intr_d.cb.IR), .npc_out(intr_d.cb.npc_out)
	      		);							
	Execute	Ex	(		
					.clock(clock), .reset(intr_a.cb1.reset), .E_Control(intr_e.cb.E_Control), .IR(intr_e.cb.IR), 
					.npc(intr_e.cb.npc), .W_Control_in(intr_e.cb.W_Control_in), .Mem_Control_in(intr_e.cb.Mem_Control_in), 
					.VSR1(intr_e.VSR1), .VSR2(intr_e.VSR2), .enable_execute(intr_e.cb.enable_execute), 
					.W_Control_out(intr_e.cb.W_Control_out), .Mem_Control_out(intr_e.cb.Mem_Control_out), 
					.aluout(intr_e.cb.aluout), .pcout(intr_e.cb.pcout), .sr1(intr_e.sr1), .sr2(intr_e.sr2), .dr(intr_e.cb.dr), 
					.M_Data(intr_e.cb.M_Data), .NZP(intr_e.cb.NZP)
				); 

	MemAccess	MemAccess (	
					.mem_state(mem_state), .M_Control(Mem_Control_out), .M_Data(M_Data), 
					.M_Addr(pcout), .memout(memout), .Data_addr(Data_addr), .Data_din(Data_din), 
					.Data_dout(Data_dout), .Data_rd(Data_rd)
				);

	Writeback	WB 		(	
					.clock(clock), .reset(intr_a.cb1.reset), .enable_writeback(intr_w.cb.enable_writeback), 
					.W_Control(intr_w.cb.W_Control), .aluout(intr_w.cb.aluout), .memout(intr_w.cb.memout), .pcout(intr_w.cb.pcout), 
					.npc(intr_w.cb.npc), .sr1(intr_w.sr1), .sr2(intr_w.sr2), .dr(intr_w.cb.dr), .d1(intr_w.d1), .d2(intr_w.d2), .psr(intr_w.cb.psr)
				);
				
	Controller_Pipeline Ctrl (
					.clock(clock), .reset(intr_a.cb1.reset), .IR(intr_c.cb.IR), .complete_instr(intr_a.cb1.complete_instr),
					.complete_data(intr_c.cb.complete_data), .NZP(intr_c.cb.NZP), .psr(intr_c.cb.psr), .br_taken(intr_c.cb.br_taken),
					
					.enable_fetch(intr_c.cb.enable_fetch), .enable_decode(intr_c.cb.enable_decode), 
					.enable_execute(intr_c.cb.enable_execute), .enable_writeback(intr_c.cb.enable_writeback), 
					.enable_updatePC(intr_c.cb.enable_updatePC), .mem_state(intr_c.cb.mem_state)

				);
								
endmodule
//----------------- TOP MODULE ------------------
module test_top_level_12;
	reg  clock;

	//INTERFACE INSTANTIATION
	all_in_one_if a1(clock);
	fetch_if f1(clock);
	decode_if d1(clock);
	execute_if e1(clock);
	writeback_if w1(clock);
	controller_if c1(clock);

	//TEST BENCH INSTANTIATION
	test_top_overall t1(f1,d1,e1,w1,c1,a1);

	//LC3 INSTANTIATION
	LC3_try  l11(.intr_d(d1),.intr_e(e1),.intr_w(w1),.intr_c(c1),.intr_a(a1),.clock(clock), .reset(a1.reset), .pc(a1.pc), .instrmem_rd(a1.instrmem_rd), .Instr_dout(a1.Instr_dout), .Data_addr(a1.Data_addr), .complete_instr(a1.complete_instr), .complete_data(a1.complete_data),  
				.Data_din(a1.Data_din), .Data_dout(a1.Data_dout), .Data_rd(a1.Data_rd), .D_macc(a1.D_macc), .I_macc(a1.I_macc)
				,.intr_f(f1)
			);

	initial 
	begin
		clock = 0;
		forever 
		begin
			#10
			clock = ~clock;
		end
	end
endmodule

