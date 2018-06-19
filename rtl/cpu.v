module cpu(clk_i, rstn_i, icb_addr_o, icb_wdata_o, icb_wmask_o, icb_en_o, icb_rdata_i, icb_ready_i);
/* 时钟复位  */
input clk_i;
input rstn_i;

/* 外部总线 */
output [31:0]	icb_addr_o;
output [31:0]	icb_wdata_o;
output [3:0]	icb_wmask_o;
output			icb_en_o;

input [31:0]	icb_rdata_i;
input			icb_ready_i;

wire [11:0]		rf_int_addr_a;
wire [31:0]		rf_int_di_a;
wire [31:0]		rf_int_do_a;
wire [3:0]		rf_int_we_a;
wire			rf_int_ce_a;

/* 暂时B口仅写 */

wire [11:0]		rf_int_addr_b;
wire [31:0]		rf_int_di_b;
wire [31:0]		rf_int_do_b;
wire [3:0]		rf_int_we_b;
wire			rf_int_ce_b;

regfile_internal_ram rf_int_ram(
	.clka(clk_i),
	.addra(rf_int_addr_a),
	.dia(rf_int_di_a),
	.doa(rf_int_do_a),
	.wea(rf_int_we_a),
	.cea(rf_int_ce_a),
	.rsta(1'b0),

	.clkb(clk_i),
	.addrb(rf_int_addr_b),
	.dib(rf_int_di_b),
	.dob(rf_int_do_b),
	.web(rf_int_we_b),
	.ceb(rf_int_ce_b),
	.rstb(1'b0)
								);

/*
	地址分配
	0x0000_0000 ~ 0x0000_007F 寄存器
	0x0000_0080 ~ 0x0000_2000 程序/代码空间
	超出此范围为外部总线
*/

localparam FSM_FETCH =	6'b000001;
localparam FSM_DECODE =	6'b000010;
localparam FSM_REGR = 	6'b000100;
localparam FSM_EXEC = 	6'b001000;
localparam FSM_MEM	=	6'b010000;
localparam FSM_REGW =	6'b100000;

localparam FSM_FETCH_I	= 0;
localparam FSM_DECODE_I	= 1;
localparam FSM_REGR_I	= 2;
localparam FSM_EXEC_I	= 3;
localparam FSM_MEM_I	= 4;
localparam FSM_REGW_I	= 5;

`define CASE_FSM			1 //打开后时序变好,提高约10MHz,多占用6个LUT,为什么?
//`define CASE_FSM_NOSTART	1 //打开后不影响LUT占用数量,时钟频率低约2MHz,为什么?
//`define RFRD_IF				1 // 把寄存器地址往前提一拍,减少了一个LUT,降低了10MHz,为什么?

/* 处理器状态机 */
reg [5:0] fsm_reg = FSM_REGW;
reg [5:0] fsm_next;

wire [31:0] lsu_addr;
reg [3:0] lsu_mem_w;

reg [31:0] pc = 32'h0000_0080;

reg fetch_enable;

reg rs1_enable;
reg rs2_enable;

`ifndef CASE_FSM
reg fsm_wait; //FSM等待信号
`endif

always @(posedge clk_i)
begin
`ifndef CASE_FSM
	if(!fsm_wait)
		fsm_reg <= fsm_reg[FSM_REGW_I]? FSM_FETCH: {fsm_reg[4:0], 1'b0};
`else
	fsm_reg <= fsm_next;
`endif

	if(fsm_reg[FSM_FETCH_I])
	begin
	end

end

always @*
begin
	fetch_enable = 0;
	rs1_enable = 0;
	rs2_enable = 0;
`ifndef CASE_FSM
	fsm_wait = 0;

	if(fsm_reg[FSM_FETCH_I] || fsm_reg[FSM_REGW_I])
		fetch_enable = 1;

	if(fsm_reg[FSM_REGR_I])
	begin
		rs1_enable = 1;
		rs2_enable = 1;
	end
`endif
`ifdef CASE_FSM
	`ifdef CASE_FSM_NOSTART
		fsm_next = 32'bx;
	`endif
	case(fsm_reg)  //synopsys parallel_case
		FSM_FETCH: //取指令
		begin
			fetch_enable = 1;
			fsm_next = FSM_DECODE;
		end
		FSM_DECODE:
		begin
			fsm_next = FSM_REGR;
		end
		FSM_REGR:
		begin
			rs1_enable = 1;
			rs2_enable = 1;
			fsm_next = FSM_EXEC;
		end
		FSM_EXEC:
		begin
			fsm_next = FSM_MEM;
		end
		FSM_MEM: //多周期运算指令也卡在这里
		begin
			fsm_next = FSM_REGW;
		end
		FSM_REGW:
		begin
			fetch_enable = 1;
			fsm_next = FSM_FETCH;
		end
		`ifndef CASE_FSM_NOSTART
		default:
			fsm_next = FSM_REGW;
		`endif
	endcase
`endif
end


/* 指令解码 */
/* 常量 */
localparam OP_LUI		= 7'b0110111;
localparam OP_AUIPC		= 7'b0010111;
localparam OP_JAL		= 7'b1101111;
localparam OP_JALR		= 7'b1100111;
localparam OP_BRANCH	= 7'b1100011;
localparam OP_LOAD		= 7'b0000011;
localparam OP_STORE		= 7'b0100011;
localparam OP_IMM		= 7'b0010011;
localparam OP_R2R		= 7'b0110011;
//暂不支持CSR
/* ALU 功能 */
localparam ALU_ADD		= 3'b000;
localparam ALU_OR		= 3'b110;
localparam ALU_AND		= 3'b111;
localparam ALU_XOR		= 3'b100;
localparam ALU_SUB		= 3'b001;

wire [31:0] instr = rf_int_do_a;
reg [6:0] opcode = 0;
reg [2:0] alu_opcode = 0;

reg [4:0] reg_rd = 0;
reg [4:0] reg_rs1 = 0;
reg [4:0] reg_rs2 = 0;
reg [2:0] funct3 = 0;

reg [31:0] dec_imm = 0;

wire [6:0] opcode_comb = instr[6:0];

wire imm_i = (opcode_comb == OP_IMM || opcode_comb == OP_LOAD || opcode_comb == OP_JALR);
wire imm_s = (opcode_comb == OP_STORE);
wire imm_b = (opcode_comb == OP_BRANCH);
wire imm_u = (opcode_comb == OP_LUI || opcode_comb == OP_AUIPC);
wire imm_j = (opcode_comb == OP_JAL);

reg alu_op_0_is_reg = 0;
reg alu_op_1_is_imm = 0;
reg need_wb = 0;
reg need_store = 0;
reg need_load = 0;
reg jump = 0;
reg branch = 0;
reg rd_nonzero = 0;

always @(posedge clk_i)
begin
	if(fsm_reg[FSM_DECODE_I])// == FSM_DECODE)
	begin
		opcode <= opcode_comb;
		need_wb <= 1;
		need_store <= 0;
		need_load <= 0;
		jump <= 0;
		branch <= 0;
		funct3 <= instr[14:12];
		alu_op_1_is_imm <= 1;
		alu_op_0_is_reg <= 1;

		alu_opcode <= ALU_ADD;
		if(instr[11:7] == 0)
		begin
			rd_nonzero <= 0;
		end
		else
		begin
			rd_nonzero <= 1;
		end
		/* 指令译码 */
		if(opcode_comb == OP_IMM)
		begin
			alu_opcode <= instr[14:12];
		end
		if(opcode_comb == OP_AUIPC)
		begin
			alu_op_0_is_reg <= 0;
		end
		if(opcode_comb == OP_JAL)
		begin
			jump <= 1;
			alu_op_0_is_reg <= 0;
		end
		if(opcode_comb == OP_STORE)
		begin
            need_store <= 1;
            need_wb <= 0;
		end
		if(opcode_comb == OP_LOAD)
		begin
			need_load <= 1;
		end
		if(opcode_comb == OP_R2R)
		begin
			alu_op_1_is_imm <= 0;
		end

`ifndef RFRD_IF
		reg_rd <= instr[11:7];
		if(opcode_comb == OP_LUI)
		begin
			alu_op_0_is_reg <= 1;
			reg_rs1 <= 0;
		end
		else
			reg_rs1 <= instr[19:15];
		reg_rs2 <= instr[24:20];
`endif

		/* 立即数译码器 */
        if(imm_i)
			dec_imm <= {{21{instr[31]}}, instr[30:20]};
		else if(imm_s)
			dec_imm <= {{21{instr[31]}}, instr[30:25], instr[11:7]};
		else if(imm_b)
			dec_imm <= {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
		else if(imm_u)
			dec_imm <= {instr[31:12], 12'b0};
		else if(imm_j)
			dec_imm <= {{12{instr[31]}}, instr[19:12] ,instr[20] ,instr[30:21] ,1'b0};
	end
end
`ifdef RFRD_IF
always @(posedge clk_i)
begin
	if(fsm_reg[FSM_FETCH_I])
	begin
		reg_rd <= instr[11:7];
		reg_rs1 <= instr[19:15];
		reg_rs2 <= instr[24:20];
	end
end
`endif
assign icb_addr_o = (fsm_reg[FSM_MEM_I])? lsu_addr: pc;
assign rf_int_addr_a = (fsm_reg[FSM_REGR_I])? reg_rs1:
						icb_addr_o[13:2];

wire internal_ram_select = (icb_addr_o[31:14] == 0);//内部RAM选择

assign rf_int_we_a = icb_wmask_o;
assign rf_int_ce_a = rs1_enable | internal_ram_select;
assign rf_int_addr_b = (fsm_reg[FSM_REGR_I])? reg_rs2: reg_rd;


/* 指令执行 */
wire [31:0] rf_reg1 = rf_int_do_a;
wire [31:0] rf_reg2 = rf_int_do_b;
/* ALU */
wire [31:0] alu_op_0 = alu_op_0_is_reg? rf_reg1: pc;
wire [31:0] alu_op_1 = alu_op_1_is_imm? dec_imm: rf_reg2;
reg [31:0] alu_result_comb;
reg [31:0] ex_result = 0;
always @*
begin
	alu_result_comb = 32'bx;
	case(alu_opcode) //synopsys parallel_case
		ALU_ADD: alu_result_comb = alu_op_0 + alu_op_1;
		ALU_OR:  alu_result_comb = alu_op_0 | alu_op_1;
		ALU_AND: alu_result_comb = alu_op_0 & alu_op_1;
		ALU_XOR: alu_result_comb = alu_op_0 ^ alu_op_1;
	endcase
end

//比较器:分支比较EX阶段计算分支类型, MEM阶段复用ALU计算PC偏移?

always @(posedge clk_i)
begin
	if(fsm_reg[FSM_EXEC_I])
	begin
		ex_result <= alu_result_comb;

	end
end
/* 访存和第二阶段多周期指令如移位 */
always @(posedge clk_i)
begin
	if(fsm_reg[FSM_MEM_I])
	begin
		if(jump)
			pc <= {ex_result[31:2], 2'b00};
		else
			pc <= pc + 4;
	end
end
/* LSU */
reg [31:0] icb_wdata;

always @*
begin
	icb_wdata = rf_reg2;
	lsu_mem_w = 4'b0000;
	if(funct3 == 3'b000)
	begin
		lsu_mem_w = 4'b0001;
	end
	if(funct3 == 3'b001)
	begin
		lsu_mem_w = 4'b0011;
	end
	if(funct3 == 3'b010)
		lsu_mem_w = 4'b1111;
/*
	if(funct3 == 3'b000)
	begin // LSU这块地址译码时序太差,性能也不好...
		lsu_mem_w[0] = (lsu_addr[1:0] == 2'b00);
		lsu_mem_w[1] = (lsu_addr[1:0] == 2'b01);
		lsu_mem_w[2] = (lsu_addr[1:0] == 2'b10);
		lsu_mem_w[3] = (lsu_addr[1:0] == 2'b11);
		icb_wdata = {rf_reg2[7:0], rf_reg2[7:0], rf_reg2[7:0], rf_reg2[7:0]};
	end
	if(funct3 == 3'b001)
	begin
		lsu_mem_w[0] = (lsu_addr[1] == 1'b0);
		lsu_mem_w[1] = (lsu_addr[1] == 1'b0);
		lsu_mem_w[2] = (lsu_addr[1] == 1'b1);
		lsu_mem_w[3] = (lsu_addr[1] == 1'b1);
		icb_wdata = {rf_reg2[15:0], rf_reg2[15:0]};
	end
	if(funct3 == 3'b010)
		lsu_mem_w = 4'b1111;
*/
end

assign lsu_addr = ex_result;
assign icb_wdata_o = icb_wdata;
assign rf_int_di_a = icb_wdata_o;
assign icb_wmask_o = (fsm_reg[FSM_MEM_I] && need_store)? lsu_mem_w: 4'b0;

wire [31:0] icb_rdata = internal_ram_select? rf_int_do_a: icb_rdata_i;
reg [31:0] data_load;

always @* //符号扩展
begin
	data_load = icb_rdata;
	/*
	if(funct3[1:0] == 2'b00)
	begin
		if(lsu_addr[1:0] == 2'b00)
			data_load = {{24{funct3[2]? 1'b0:icb_rdata[7]}}, icb_rdata[7:0]};
		if(lsu_addr[1:0] == 2'b01)
			data_load = {{24{funct3[2]? 1'b0:icb_rdata[15]}}, icb_rdata[15:8]};
		if(lsu_addr[1:0] == 2'b10)
			data_load = {{24{funct3[2]? 1'b0:icb_rdata[23]}}, icb_rdata[23:16]};
		if(lsu_addr[1:0] == 2'b11)
			data_load = {{24{funct3[2]? 1'b0:icb_rdata[31]}}, icb_rdata[31:24]};
	end
	if(funct3[1:0] == 2'b01) //资源消耗太大,后面再想办法
	begin
		if(lsu_addr[1] == 1'b0)
			data_load = {{16{funct3[2]? 1'b0:icb_rdata[15]}}, icb_rdata[15:0]};
		if(lsu_addr[1] == 1'b1)
			data_load = {{16{funct3[2]? 1'b0:icb_rdata[31]}}, icb_rdata[31:16]};
	end
	*/
end

/* 回写 */
always @(posedge clk_i)
begin
	if(fsm_reg[FSM_REGW_I])
	begin

	end
end

assign rf_int_ce_b = rs2_enable | (fsm_reg[FSM_REGW_I]);
assign rf_int_we_b = (fsm_reg[FSM_REGW_I] && need_wb && rd_nonzero)? 4'b1111: 4'b0;
assign rf_int_di_b = (need_load)? data_load: ex_result;
endmodule

