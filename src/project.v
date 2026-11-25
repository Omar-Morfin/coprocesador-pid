/*
 * Copyright (c) 2026 Ever Omar Morfin Orozco
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_pid_coprocessor (
    input  wire [7:0] ui_in,    // Dedicated inputs: Bus de datos (0-255)
    output wire [7:0] uo_out,   // Dedicated outputs: Salida de control PID (0-255)
    input  wire [7:0] uio_in,   // IOs: [2:0]=sel_reg, [3]=cargar, [4]=ejecutar
    output wire [7:0] uio_out,  // IOs: [5]=saturado, [6]=error_cero, [7]=error_neg
    output wire [7:0] uio_oe,   // IOs: Enable path (0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

    wire [2:0] sel_reg  = uio_in[2:0];
    wire       cargar   = uio_in[3];
    wire       ejecutar = uio_in[4];

    // Ganancias compactas de 3 bits (potencias de 2)
    reg [2:0] reg_kp;
    reg [2:0] reg_ki;
    reg [2:0] reg_kd;
    reg [7:0] reg_setpoint;
    reg [7:0] reg_medicion;

    wire [7:0] salida_control;
    wire       saturado;
    wire       error_cero;
    wire       error_negativo;

    // Carga de parámetros por turnos
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_kp       <= 3'd4; // Kp = x1.0 por defecto
            reg_ki       <= 3'd0;
            reg_kd       <= 3'd0;
            reg_setpoint <= 8'd0;
            reg_medicion <= 8'd0;
        end else if (ena && cargar) begin
            case (sel_reg)
                3'd0: reg_kp       <= ui_in[2:0];
                3'd1: reg_ki       <= ui_in[2:0];
                3'd2: reg_kd       <= ui_in[2:0];
                3'd3: reg_setpoint <= ui_in;
                3'd4: reg_medicion <= ui_in;
                default: ;
            endcase
        end
    end

    // Instancia del núcleo PID compacto
    coprocesador_pid u_pid (
        .clk            (clk),
        .rst_n          (rst_n),
        .habilitar      (ena && ejecutar),
        .setpoint       (reg_setpoint),
        .medicion       (reg_medicion),
        .kp             (reg_kp),
        .ki             (reg_ki),
        .kd             (reg_kd),
        .salida_control (salida_control),
        .saturado       (saturado),
        .error_cero     (error_cero),
        .error_negativo (error_negativo)
    );

    assign uo_out  = salida_control;
    assign uio_out = {error_negativo, error_cero, saturado, 5'b00000};
    assign uio_oe  = 8'b1110_0000;

    wire _unused = &{uio_in[7:5], 1'b0};

endmodule

module coprocesador_pid (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       habilitar,
    input  wire [7:0] setpoint,
    input  wire [7:0] medicion,
    input  wire [2:0] kp,
    input  wire [2:0] ki,
    input  wire [2:0] kd,
    output reg  [7:0] salida_control,
    output reg        saturado,
    output reg        error_cero,
    output reg        error_negativo
);

    reg signed [11:0] acumulador;
    reg signed [9:0]  error_anterior;

    wire signed [9:0]  error_actual;
    wire signed [10:0] derivada;
    wire signed [12:0] acum_temp;
    wire signed [11:0] nuevo_acum;

    reg  signed [14:0] termino_p;
    reg  signed [14:0] termino_i;
    reg  signed [14:0] termino_d;
    wire signed [15:0] suma_total;

    assign error_actual = $signed({2'b00, setpoint}) - $signed({2'b00, medicion});
    assign derivada     = error_actual - error_anterior;

    // Anti-Windup (-1024 a +1023)
    assign acum_temp  = acumulador + error_actual;
    assign nuevo_acum = (acum_temp > 13'sd1023)  ?  12'sd1023 :
                        (acum_temp < -13'sd1024) ? -12'sd1024 :
                         acum_temp[11:0];

    // Escalado por desplazamiento de bits (sin multiplicadores)
    always @(*) begin
        case (kp)
            3'd1: termino_p = {{5{error_actual[9]}}, error_actual} >>> 3;
            3'd2: termino_p = {{5{error_actual[9]}}, error_actual} >>> 2;
            3'd3: termino_p = {{5{error_actual[9]}}, error_actual} >>> 1;
            3'd4: termino_p = {{5{error_actual[9]}}, error_actual};
            3'd5: termino_p = {{5{error_actual[9]}}, error_actual} <<< 1;
            3'd6: termino_p = {{5{error_actual[9]}}, error_actual} <<< 2;
            3'd7: termino_p = {{5{error_actual[9]}}, error_actual} <<< 3;
            default: termino_p = 15'sd0;
        endcase

        case (ki)
            3'd1: termino_i = {{3{nuevo_acum[11]}}, nuevo_acum} >>> 3;
            3'd2: termino_i = {{3{nuevo_acum[11]}}, nuevo_acum} >>> 2;
            3'd3: termino_i = {{3{nuevo_acum[11]}}, nuevo_acum} >>> 1;
            3'd4: termino_i = {{3{nuevo_acum[11]}}, nuevo_acum};
            3'd5: termino_i = {{3{nuevo_acum[11]}}, nuevo_acum} <<< 1;
            3'd6: termino_i = {{3{nuevo_acum[11]}}, nuevo_acum} <<< 2;
            3'd7: termino_i = {{3{nuevo_acum[11]}}, nuevo_acum} <<< 3;
            default: termino_i = 15'sd0;
        endcase

        case (kd)
            3'd1: termino_d = {{4{derivada[10]}}, derivada} >>> 3;
            3'd2: termino_d = {{4{derivada[10]}}, derivada} >>> 2;
            3'd3: termino_d = {{4{derivada[10]}}, derivada} >>> 1;
            3'd4: termino_d = {{4{derivada[10]}}, derivada};
            3'd5: termino_d = {{4{derivada[10]}}, derivada} <<< 1;
            3'd6: termino_d = {{4{derivada[10]}}, derivada} <<< 2;
            3'd7: termino_d = {{4{derivada[10]}}, derivada} <<< 3;
            default: termino_d = 15'sd0;
        endcase
    end

    assign suma_total = termino_p + termino_i + termino_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acumulador     <= 12'sd0;
            error_anterior <= 10'sd0;
            salida_control <= 8'd0;
            saturado       <= 1'b0;
            error_cero     <= 1'b0;
            error_negativo <= 1'b0;
        end else if (habilitar) begin
            acumulador     <= nuevo_acum;
            error_anterior <= error_actual;
            error_cero     <= (error_actual == 10'sd0);
            error_negativo <= (error_actual < 10'sd0);

            if (suma_total > 16'sd255) begin
                salida_control <= 8'd255;
                saturado       <= 1'b1;
            end else if (suma_total < 16'sd0) begin
                salida_control <= 8'd0;
                saturado       <= 1'b1;
            end else begin
                salida_control <= suma_total[7:0];
                saturado       <= 1'b0;
            end
        end
    end

endmodule
