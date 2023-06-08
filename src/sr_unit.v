module func(
    input clk_i ,
    input rst_i ,
    
    input [7:0] a_bi ,
    input [7:0] b_bi ,
    
    input start_i ,

    // output ready,
    output busy_o ,
    output reg [31:0] y_bo
);

    localparam IDLE = 2'h0 ;
    localparam WORK_MULT = 2'h1 ;
    localparam WORK_SQRT = 2'h2 ;
    
    reg [1:0] state = IDLE;
    
    reg ready_in;
    reg start_mult;
    reg start_sqrt;
    reg start_translate;
    
    reg [15:0] a_res, b_res;
    wire [15:0] a_sq, b_sq;
    
    wire a_busy, b_busy, sqrt_busy, translate_busy;
    wire a_ready, b_ready, sqrt_ready;
    reg [16:0] sum;
    wire [7:0] res;
    wire [11:0] translate_result;

    reg a_rst;
    reg b_rst;
    reg sum_rst;
    
    // assign ready = ready_in;
    assign busy_o = state > 0 ;
    
    wire [16:0] summator_reg1;
    wire [16:0] summator_reg2;
    wire [16:0] summator_result;
    
    wire [16:0] sqrt_operand1;
    wire [16:0] sqrt_operand2;
    
    assign summator_reg1 = (state == WORK_MULT) ? a_sq : sqrt_operand1;
    assign summator_reg2 = (state == WORK_MULT) ? b_sq : sqrt_operand2;
       
    summator main_summator(
        .a1(summator_reg1), .b1(summator_reg2),
        .res1(summator_result)
    );
    
    reg in_rst;
    initial begin
        in_rst <= 1;
    end

    multer a_square(.clk_i(clk_i), .rst_i(a_rst), .ready(a_ready), 
                    .a_bi(a_bi), .b_bi(a_bi), .start_i(start_mult), 
                    .busy_o(a_busy), .y_bo(a_sq));

    multer b_square(.clk_i(clk_i), .rst_i(b_rst), .ready(b_ready), 
                    .a_bi(b_bi), .b_bi(b_bi), .start_i(start_mult), 
                    .busy_o(b_busy), .y_bo(b_sq));


    reg [16:0] sqrt_input;
    sqrt sqrt_res(.clk_i(clk_i), .rst_i(sum_rst), .ready(sqrt_ready),
                  .a_bi(sqrt_input), .start_i(start_sqrt), 
                  .busy_o(sqrt_busy), .y_bo(res), 
                  .summator_reg1_sqrt(sqrt_operand1), .summator_reg2_sqrt(sqrt_operand2), 
                  .summator_result_sqrt(summator_result)
                  );

    always @(posedge clk_i)
        if (rst_i == 1 || in_rst == 1) begin
            state <= IDLE ;
            sum_rst <= 1;
            
            start_mult <= 0;
            start_sqrt <= 0;
            start_translate <= 0;
            
            a_rst <= 1;
            b_rst <= 1;
                        
            sum <= 0;
            ready_in <= 1;
            
            in_rst <= 0;
        end else begin
            case ( state )
                IDLE :
                    if (start_i) begin
                        a_rst <= 0;
                        b_rst <= 0;
                        sum_rst <= 0;
                        state <= WORK_MULT ;
                        ready_in <= 0 ;
                        start_mult <= 1 ;
                    end
                WORK_MULT:
                    begin
                        if (start_mult) begin
                            start_mult <= 0; // and <= 1        
                        end else if (!a_busy && !b_busy) begin
                            a_res <= a_sq;
                            b_res <= b_sq;
                            sqrt_input <= summator_result; // a_sq + b_sq
                            start_sqrt <= 1;
                            state <= WORK_SQRT;
                        end
                    end
                WORK_SQRT:
                    begin
                        if (start_sqrt) begin
                            start_sqrt <= 0;
                        end else if (!sqrt_busy) begin
                            y_bo <= res;                        
                            state <= IDLE;
                            in_rst <= 1;
                        end
                    end

            endcase
        end
        
        
endmodule

//////////////////////////////////////////////////////////////////////////////////

module multer (
    input clk_i ,
    input rst_i ,
    
    input [7:0] a_bi ,
    input [7:0] b_bi ,
    
    input start_i ,
    
    output wire ready,
    output busy_o ,
    output reg [15:0] y_bo
);

    localparam IDLE = 1'b0 ;
    localparam WORK = 1'b1 ;
    
    reg [2:0] ctr ;
    wire [2:0] end_step ;
    wire [7:0] part_sum ;
    wire [15:0] shifted_part_sum ;
    reg [7:0] a , b ;
    reg [15:0] part_res ;
    reg state = IDLE;
    reg ready_in ; 
    
    assign part_sum = a & { 8{ b [ ctr ] } } ;
    assign shifted_part_sum = part_sum << ctr ;
    assign end_step = ( ctr == 3'h7 ) ;
    assign busy_o = state ;
    assign ready = ready_in ;
    
    always @(posedge clk_i)
        if (rst_i) begin
            ctr <= 0 ;
            part_res <= 0 ;
            y_bo <= 0 ;
            state <= IDLE ;
            ready_in <= 1 ;
        end else begin
            case ( state )
                IDLE :
                    if (start_i) begin
                        state <= WORK;
                        a <= a_bi ;
                        b <= b_bi ;
                        ctr <= 0 ;
                        part_res <= 0 ;
                        ready_in <= 0 ;
                    end
                WORK:
                begin
                    if ( end_step ) begin
                        state <= IDLE ;
                        y_bo <= part_res ;
                    end
                    part_res <= part_res + shifted_part_sum ;
                    ctr <= ctr + 1 ;
                end
            endcase
        end
endmodule

//////////////////////////////////////////////////////////////////////////////////

module sqrt (
    input clk_i ,
    input rst_i ,
    
    input [16:0] a_bi ,
    
    input start_i ,
    
    output wire ready,
    output busy_o ,
    output reg [7:0] y_bo,

    output wire [16:0] summator_reg1_sqrt,
    output wire [16:0] summator_reg2_sqrt,
    input wire [16:0] summator_result_sqrt
);

    localparam IDLE = 1'b0 ;
    localparam WORK = 1'b1 ;
    
    wire [2:0] end_step ;
    
    reg [15:0] a ;
    reg state  = IDLE;
    reg ready_in ; 
    
    reg [15:0] m ;
    reg [16:0] y_or_m, y ;
    
    reg[16:0] y_temp, yw, mw;
    reg [16:0] aw, bw;
    
    assign busy_o = state ;
    assign end_step = (m == 0) ;
    assign ready = ready_in ;
    
    assign summator_reg1_sqrt = a;
    assign summator_reg2_sqrt = -bw;
    
    always @(posedge clk_i)
        if (rst_i) begin
            m <= 1 << 14 ;
            y_bo <= 0 ;
            y <= 0 ;
            state <= IDLE ;
            ready_in <= 1 ;
        end else begin
            case ( state )
                IDLE :
                    if (ready && start_i) begin
                        m <= 1 << 14 ;
                        state <= WORK;
                        a <= a_bi ;
                        ready_in <= 0 ;
                    end
                WORK:
                begin
                    if (end_step) begin
                        state <= IDLE ;
                        y_bo <= y ;
                    end else begin
                        y_or_m <= bw;
                        a <= aw;
                        y <= yw;
                        m <= mw;
                    end
                end
            endcase
        end
        
        always @*
            begin
                bw = y | m ;
                yw = y >> 1 ;
                if (a >= bw) begin
                    aw = summator_result_sqrt ; // a - bw
                    yw = yw | m ;
                end else begin
                    aw = a;
                end
                mw = m >> 2 ;
            end
        
endmodule

//////////////////////////////////////////////////////////////////////////////////

module summator(
input [16:0] a1,
input [16:0] b1,
output [16:0] res1
);
    assign res1 = a1 + b1;
endmodule

