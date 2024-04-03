`timescale 1ns / 1ps

module PID_tb;
    reg  clk = 0;
    reg  rst_n = 0;
    reg  [15:0] setpoint = 0;
    reg  [15:0] feedback = 0;
    reg  [15:0] Kp = 0;
    reg  [15:0] Ki = 0;
    reg  [15:0] Kd = 0;
    wire [31:0] control_signal;
    wire [3:0] of;

    PID PID_core(.clk(clk),.rst(rst_n),.Kp_in(Kp),.Ki_in(Ki),.Kd_in(Kd),.SV_in(setpoint),.PV_in(feedback),.MV(control_signal),.of(of));

    initial begin
        $dumpfile ("PID_core.dump");
        $dumpvars(0,PID_core);
    end

    initial begin
        rst_n <= 0; // Assert reset
        setpoint <= 20;
        Kp <= 1;
        Ki <= 1;
        Kd <= 1;
        #50 rst_n <= 1; // Deassert reset
    end

    always #10 clk = ~clk;

    always begin
    $monitor("Control signal is %d",$signed(control_signal));
        #200 feedback <= 1;
        #200 feedback <= 5;
        #200 feedback <= 8;
        #200 feedback <= 10; 
        #200 feedback <= 13;     
        #200 feedback <= 15;     
        #200 feedback <= 16;  
        #200 feedback <=19;
        #200 feedback <=21;     
        #250 $finish;
    end

endmodule
