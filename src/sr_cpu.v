/*
 * schoolRISCV - small RISC-V CPU 
 *
 * originally based on Sarah L. Harris MIPS CPU 
 *                   & schoolMIPS project
 * 
 * Copyright(c) 2017-2020 Stanislav Zhelnio 
 *                        Aleksandr Romanov 
 */ 

`include "sr_cpu.vh"

module sr_cpu
(
    input           clk,        // clock
    input           rst_n,      // reset
    input   [ 4:0]  regAddr,    // debug access reg address
    output  [31:0]  regData,    // debug access reg data
    output  [31:0]  imAddr,     // instruction memory address
    input   [31:0]  imData      // instruction memory data
);
    //control wires
    wire        aluZero;
    wire        pcSrc;
    wire        regWrite;
    wire        aluSrc;
    wire        wdSrc;
    wire  [2:0] aluControl;

    // func
    wire        funcBusy;
    wire        funcIRQ;
    wire [4:0]  funcRd;
    wire [2:0]  funcControl;
    wire [31:0] funcResult;
 
    wire [2:0]  unitControl;
    wire [31:0] unitResult;
    wire        unitSelect;
 
    //instruction decode wires
    wire [ 6:0] cmdOp;
    wire [ 4:0] rd;
    wire [ 2:0] cmdF3;
    wire [ 4:0] rs1;
    wire [ 4:0] rs2;
    wire [ 6:0] cmdF7;
    wire [31:0] immI;
    wire [31:0] immB;
    wire [31:0] immU;

    //program counter
    wire [31:0] pc;
    wire [31:0] pcBranch = pc + immB;
    wire [31:0] pcPlus4  = pc + 4;
    wire [31:0] pcNext = funcBusy | funcIRQ ? pc : (pcSrc ? pcBranch : pcPlus4);
 
    sm_register r_pc(
        clk, 
        rst_n,
        pcNext,
        pc
    );

    //program memory access
    assign imAddr = pc >> 2;
    wire [31:0] instr = funcBusy | funcIRQ ? `RVPE_NOP : imData;

    wire [4:0]  proxyRd;
    wire        proxyRegWrite;

    assign proxyRd = funcIRQ ? funcRd : rd; 
    assign proxyRegWrite = funcIRQ ? 1 : regWrite;

    //instruction decode
    sr_decode id (
        .instr      ( instr        ),
        .cmdOp      ( cmdOp        ),
        .rd         ( rd           ),
        .cmdF3      ( cmdF3        ),
        .rs1        ( rs1          ),
        .rs2        ( rs2          ),
        .cmdF7      ( cmdF7        ),
        .immI       ( immI         ),
        .immB       ( immB         ),
        .immU       ( immU         ) 
    );

    //register file
    wire [31:0] rd0;
    wire [31:0] rd1;
    wire [31:0] rd2;
    wire [31:0] wd3;

    sm_register_file rf (
        .clk        ( clk          ),
        .a0         ( regAddr      ),
        .a1         ( rs1          ),
        .a2         ( rs2          ),
        .a3         ( proxyRd      ),
        .rd0        ( rd0          ),
        .rd1        ( rd1          ),
        .rd2        ( rd2          ),
        .wd3        ( wd3          ),
        .we3        ( proxyRegWrite)
    );

    //debug register access
    assign regData = (regAddr != 0) ? rd0 : pc;

    //alu
    wire [31:0] srcB = aluSrc ? immI : rd2;
    wire [31:0] aluResult;

    sr_alu alu (
        .srcA       ( rd1          ),
        .srcB       ( srcB         ),
        .oper       ( aluControl   ),
        .zero       ( aluZero      ),
        .result     ( aluResult    ) 
    );

    assign wd3 = wdSrc ? immU : unitResult;

    //control
    sr_control sm_control (
        .cmdOp      ( cmdOp        ),
        .cmdF3      ( cmdF3        ),
        .cmdF7      ( cmdF7        ),
        .aluZero    ( aluZero      ),
        .pcSrc      ( pcSrc        ),
        .regWrite   ( regWrite     ),
        .aluSrc     ( aluSrc       ),
        .wdSrc      ( wdSrc        ),
        .aluControl ( aluControl   ), 
        .unitControl( unitControl  ),
        .unitSelect ( unitSelect   )
    );

    wire proxyUnitSelect;
    assign proxyUnitSelect = funcIRQ ? 1 : unitSelect;

    sr_unit_selector iselect (
        .unit         ( proxyUnitSelect  ),
        .aluControl   ( aluControl       ),
        .aluResult    ( aluResult        ),
        .funcControl  ( funcControl      ),
        .funcBusy     ( funcBusy         ),
        .funcResult   ( funcResult       ),
        .unitControl  ( unitControl      ),
        .unitResult   ( unitResult       )
    );

    func_unit fn_unit(
        .clk     ( clk         ),
        .rst     ( ~rst_n      ),
        .rd_i    ( proxyRd     ),
        .srcA    ( rd1         ),
        .srcB    ( srcB        ),
        .oper    ( funcControl ),
        .result  ( funcResult  ),
        .busy    ( funcBusy    ),
        .irq     ( funcIRQ     ),
        .rd_o    ( funcRd      )
    );

endmodule

module sr_decode
(
    input      [31:0] instr,
    output     [ 6:0] cmdOp,
    output     [ 4:0] rd,
    output     [ 2:0] cmdF3,
    output     [ 4:0] rs1,
    output     [ 4:0] rs2,
    output     [ 6:0] cmdF7,
    output reg [31:0] immI,
    output reg [31:0] immB,
    output reg [31:0] immU 
);
    assign cmdOp = instr[ 6: 0];
    assign rd    = instr[11: 7];
    assign cmdF3 = instr[14:12];
    assign rs1   = instr[19:15];
    assign rs2   = instr[24:20];
    assign cmdF7 = instr[31:25];

    // I-immediate
    always @ (*) begin
        immI[10: 0] = instr[30:20];
        immI[31:11] = { 21 {instr[31]} };
    end

    // B-immediate
    always @ (*) begin
        immB[    0] = 1'b0;
        immB[ 4: 1] = instr[11:8];
        immB[10: 5] = instr[30:25];
        immB[   11] = instr[7];
        immB[31:12] = { 20 {instr[31]} };
    end

    // U-immediate
    always @ (*) begin
        immU[11: 0] = 12'b0;
        immU[31:12] = instr[31:12];
    end

endmodule

module sr_control
(
    input      [6:0] cmdOp,
    input      [2:0] cmdF3,
    input      [6:0] cmdF7,
    input            aluZero,
    output           pcSrc, 
    output reg       regWrite, 
    output reg       aluSrc,
    output reg       wdSrc,
    output reg [2:0] aluControl,
    output reg [2:0] unitControl,
    output reg       unitSelect
);
    reg          branch   = 1'b0;
    reg          opIsBGE  = 1'b0;
    reg          condZero = 1'b0;

    assign pcSrc = branch & (opIsBGE ? aluZero : (aluZero == condZero));

    always @ (*) begin
        branch      = 1'b0;
        condZero    = 1'b0;
        regWrite    = 1'b0;
        aluSrc      = 1'b0;
        wdSrc       = 1'b0;
        opIsBGE     = 1'b0;
        aluControl  = `ALU_ADD;
        unitSelect  = 1'b0;
        unitControl = `ALU_ADD;

        casez( {cmdF7, cmdF3, cmdOp} )
            { `RVF7_ADD,  `RVF3_ADD,  `RVOP_ADD  } : begin regWrite = 1'b1; aluControl = `ALU_ADD;  end
            { `RVF7_OR,   `RVF3_OR,   `RVOP_OR   } : begin regWrite = 1'b1; aluControl = `ALU_OR;   end
            { `RVF7_SRL,  `RVF3_SRL,  `RVOP_SRL  } : begin regWrite = 1'b1; aluControl = `ALU_SRL;  end
            { `RVF7_SLTU, `RVF3_SLTU, `RVOP_SLTU } : begin regWrite = 1'b1; aluControl = `ALU_SLTU; end
            { `RVF7_SUB,  `RVF3_SUB,  `RVOP_SUB  } : begin regWrite = 1'b1; aluControl = `ALU_SUB;  end

            { `RVF7_ANY,  `RVF3_ADDI, `RVOP_ADDI } : begin regWrite = 1'b1; aluSrc = 1'b1; aluControl = `ALU_ADD; end
            { `RVF7_ANY,  `RVF3_ANY,  `RVOP_LUI  } : begin regWrite = 1'b1; wdSrc  = 1'b1; end

            { `RVF7_ANY,  `RVF3_BEQ,  `RVOP_BEQ  } : begin branch = 1'b1; condZero = 1'b1; aluControl = `ALU_SUB; end
            { `RVF7_ANY,  `RVF3_BNE,  `RVOP_BNE  } : begin branch = 1'b1; aluControl = `ALU_SUB; end

            { `RVF7_ANY,  `RVF3_BGE,  `RVOP_BGE  } : begin branch = 1'b1; opIsBGE = 1'b1; aluControl = `ALU_SLTU; unitControl = `ALU_SLTU; end

            { `RVF7_FUNC, `RVF3_FUNC, `RVOP_FUNC } : begin regWrite = 1'b1; unitControl = `FUNC_START; unitSelect = 1; end
        endcase
    end
endmodule

module sr_alu
(
    input      [31:0] srcA,
    input      [31:0] srcB,
    input      [ 2:0] oper,
    output            zero,
    output reg [31:0] result
);
    always @ (*) begin
        case (oper)
            default   : result = srcA + srcB;
            `ALU_ADD  : result = srcA + srcB;
            `ALU_OR   : result = srcA | srcB;
            `ALU_SRL  : result = srcA >> srcB [4:0];
            `ALU_SLTU : result = (srcA < srcB) ? 1 : 0;
            `ALU_SUB  : result = srcA - srcB;
        endcase
    end

    assign zero   = (result == 0);
endmodule

module sr_unit_selector 
(
    input             unit,

    output reg [ 2:0] aluControl,
    input      [31:0] aluResult,

    output reg [ 2:0] funcControl,
    input      [31:0] funcResult,
    input             funcBusy,

    input      [ 2:0] unitControl,
    output reg [31:0] unitResult 
);
    always @ (*)
            case (unit)
                1'b0: if (~funcBusy) begin
                    unitResult   = aluResult;
                    aluControl   = unitControl;
                    funcControl  = `FUNC_IDLE;
                end else begin 
                    unitResult   = funcResult;
                    funcControl  = 1'b1;
                    end
                1'b1: begin
                    unitResult   = funcResult;
                    funcControl  = unitControl;
                end
            endcase
endmodule

module sm_register_file
(
    input         clk,
    input  [ 4:0] a0,
    input  [ 4:0] a1,
    input  [ 4:0] a2,
    input  [ 4:0] a3,
    output [31:0] rd0,
    output [31:0] rd1,
    output [31:0] rd2,
    input  [31:0] wd3,
    input         we3
);
    reg [31:0] rf [31:0];

    assign rd0 = (a0 != 0) ? rf [a0] : 32'b0;
    assign rd1 = (a1 != 0) ? rf [a1] : 32'b0;
    assign rd2 = (a2 != 0) ? rf [a2] : 32'b0;

    always @ (posedge clk)
        if(we3) rf [a3] <= wd3;
endmodule

module func_unit
(
    input              clk,
    input              rst,
    input      [ 4:0]  rd_i,
    input      [31:0]  srcA,
    input      [31:0]  srcB,
    input      [ 2:0]  oper,
    output reg [31:0]  result,
    output             busy,
    output             irq,
    output reg [ 4:0]  rd_o
);
 
    localparam IDLE = 1'b0;
    localparam WORK = 1'b1;
    
    wire fn_busy;
    
    reg state = IDLE;
    reg res_unsaved = 0;
    reg rbusy = 0;
    reg start = 0;
    reg rirq = 0;
    assign irq = rirq;
    assign busy = start | fn_busy | res_unsaved;
    reg [31:0] a_bi;
    reg [31:0] b_bi;
    wire [31:0] y_bo;
    
    func fn (
        .clk_i      ( clk      ),
        .rst_i      ( rst      ),
        .start_i    ( start    ),
        .a_bi       ( a_bi     ),
        .b_bi       ( b_bi     ),
        .busy_o     ( fn_busy  ),
        .y_bo       ( y_bo     )
    );

    always @ ( posedge clk ) begin
        if (rst) begin
            state <= IDLE;
            res_unsaved <= 0;
            rbusy <= 0;
            start <= 0;
            rirq <= 0;
            start <= 0;
            rirq   <= 0;
        end else if ( oper == `FUNC_IDLE ) begin
            start <= 0;
            rirq   <= 0;
        end else case ( state )
            IDLE: if ( oper == `FUNC_START ) begin
                a_bi  <= srcA;
                b_bi  <= srcB;
                start <= 1;
                rd_o  <= rd_i;
                res_unsaved = 1;
                state <= WORK;
            end else begin
                start <= 0;
                result <= 0;
                rirq   <= 0;
            end
            WORK: if ( !(start | fn_busy) ) begin
                result <= y_bo;
                state <= IDLE;
                rirq <= 1;
                res_unsaved <= 0; 
            end else begin
                start <= 0;
            end
 
        endcase
    end
endmodule
