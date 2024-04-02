`timescale 1ns / 1ps

module PID(input wire clk, input wire rst, 
input wire [15:0] Kp_in,input wire [15:0] Ki_in,input wire [15:0] Kd_in,
input wire [15:0] SV_in,input wire [15:0] PV_in,output wire [31:0] MV,output wire [2:0] of);
    //SV: Set value     PV: Present value     MV:Manipulation value

    //Stage1
    reg [15:0] Kp,Ki0,Kd,SV,PV;
    always@(posedge clk) begin
        if(!rst) begin
            Kp<=0;Ki0<=0;Kd<=0;
            SV<=0;PV<=0;
        end
        else begin
            Kp<=Kp_in; Ki0<=Ki_in; Kd<=Kd_in;
            SV<=SV_in; PV<=PV_in;
        end
    end
    
    //Calculate Kd*e(n-1)
    wire [31:0] S1_pro; 
    reg [15:0] error;
    Booth_Multiplier S1_Mul(.clk(clk),.rst(rst),.M(Kd),.Q(error),.product(S1_pro));
    wire [31:0] Kd_err_pre0;
     

    //Calculate Kp + Kd
    wire [15:0] S1_sum1; 
    assign S1_sum1 = ($signed(Kp) + $signed(Kd)); 
    wire of_S1_sum1; //Overflow checking
    assign of_S1_sum1 = 
       ((Kp[15] == 1 && Kd[15] == 1 && S1_sum1[15] == 0) ||  // Negative + Negative = Positive
        (Kp[15] == 0 && Kd[15] == 0 && S1_sum1[15] == 1));   // Positive + Positive = Negative
    wire [15:0] Kpd0;
    

    //Calculate Current error
    wire [15:0] S1_sum2;
    assign S1_sum2 = $signed(SV) - $signed(PV);
    wire of_S1_sum2; //Overflow checking
    assign of_S1_sum2 = 
       ((SV[15] == 0 && PV[15] == 1 && S1_sum2[15] == 1) ||  // Positive - Negative = Negative
        (SV[15] == 1 && PV[15] == 0 && S1_sum2[15] == 0));   // Negative - Positive = Positive
    wire [15:0] error0; 
    

    //Overflow of Stage 1
    wire of_S1;
    assign of_S1 = of_S1_sum1 || of_S1_sum2; 
    //In one stage, if overflow happens in any ALU, the whole stage is stalled
    //Pass the data based on the overflow signal
    assign Kd_err_pre0 = of_S1?0:S1_pro;
    assign Kpd0 = of_S1?0:S1_sum1;
    assign error0 = of_S1?0:S1_sum2;

    //Stage2
    reg [15:0] Kpd, Ki;
    always@(posedge clk) begin
        if(!rst) begin
            Kpd<=0;
            Ki<=0;
            error<=0;
        end
        else begin
            Kpd<=Kpd0;
            Ki<=Ki0;
            error<=error0;
        end
    end

    //Calculate Kpd*e(n)
    wire [31:0] S2_pro1;
    Booth_Multiplier S2_Mul1(.clk(clk),.rst(rst),.M(Kpd),.Q(error),.product(S2_pro1));
    wire [31:0] Kpd_err0;
    

    //Calculate Ki*e(n)
    wire [31:0] S2_pro2;
    Booth_Multiplier S2_Mul2(.clk(clk),.rst(rst),.M(Ki),.Q(error),.product(S2_pro2));
    wire [31:0] Ki_err0;
    
    assign Kpd_err0 = S2_pro1; 
    assign Ki_err0 = S2_pro2;

    //Stage3
    reg [31:0] Kd_err_pre1;
    always@(posedge clk) begin
        if(!rst) begin
            Kd_err_pre1<=0;
        end
        else begin
            Kd_err_pre1<=Kd_err_pre0;
        end
    end

    //Stage4
    reg [31:0] Kd_err_pre,Kpd_err,Sigma,Ki_err;
    wire [31:0] New_Sigma; 
    always@(posedge clk) begin
        if(!rst) begin
            Kd_err_pre<=0;
            Kpd_err<=0;
            Sigma<=0;
            Ki_err<=0;
        end
        else begin
            Kd_err_pre<=Kd_err_pre1;
            Kpd_err<=Kpd_err0;
            Sigma<=New_Sigma;
            Ki_err<=Ki_err0;
        end
    end
    //Calculate PD =  Kpd*e(n) - Kd*e(n-1) 
    wire [31:0] S4_sum1;
    KSA_top_level S4_Add1(.a(Kpd_err),.b(~Kd_err_pre),.cin(1'b1),.sum(S4_sum1),.cout());
    wire of_S4_sum1; //Overflow checking
    assign of_S4_sum1 = 
       ((Kpd_err[31] == 0 && Kd_err_pre[31] == 1 && S4_sum1[31] == 1) ||  // Positive - Negative = Negative
        (Kpd_err[31] == 1 && Kd_err_pre[31] == 0 && S4_sum1[31] == 0));   // Negative - Positive = Positive
    wire [31:0] New_PD; 
    

    //Calculate I = Ki*e(n) + Integral
    wire [31:0] S4_sum2;
    KSA_top_level S4_Add2(.a(Ki_err),.b(Sigma),.cin(1'b0),.sum(S4_sum2),.cout());
    wire of_S4_sum2; //Overflow checking
    assign of_S4_sum2 = 
       ((Ki_err[31] == 1 && Sigma[31] == 1 && S4_sum2[31] == 0) ||  // Negative + Negative = Positive
        (Ki_err[31] == 0 && Sigma[31] == 0 && S4_sum2[31] == 1));   // Positive + Positive = Negative
    wire [31:0] New_I;
    assign New_Sigma = of_S4_sum2?Sigma:S4_sum2;

    //Overflow of Stage 4
    wire of_S4;
    assign of_S4 = of_S4_sum1 || of_S4_sum2;
    //Pass the data based on the overflow signal
    assign New_PD = of_S4_sum1?0:S4_sum1;
    assign New_I = of_S4_sum2?0:S4_sum2;

    //Stage5
    reg [31:0] PD,I;
    always@(posedge clk) begin
        if(!rst) begin
            PD<=0;
            I<=0;
        end
        else begin
            PD<=New_PD;
            I<=New_I;
        end
    end
    //Calculate Manipulation Value 
    wire [31:0] S5_sum;
    KSA_top_level S5_Add(.a(PD),.b(I),.cin(1'b0),.sum(S5_sum),.cout());
    wire of_S5; //Overflow checking
    assign of_S5 = 
       ((PD[31] == 1 && I[31] == 1 && S5_sum[31] == 0) ||  // Negative + Negative = Positive
        (PD[31] == 0 && I[31] == 0 && S5_sum[31] == 1));   // Positive + Positive = Negative 
    assign MV = of_S5?0:S5_sum;
    assign of = {of_S5,of_S4,of_S1};
endmodule
