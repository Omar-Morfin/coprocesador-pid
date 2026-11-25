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

    reg [7:0] reg_kp;
    reg [7:0] reg_ki;
    reg [7:0] reg_kd;
    reg [7:0] reg_setpoint;
    reg [7:0] reg_medicion;

    wire [7:0] salida_control;
    wire       saturado;
    wire       error_cero;
    wire       error_negativo;

    // Carga de parámetros por turnos
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_kp       <= 8'd16; // Kp = 1.0 (16/16) por defecto
            reg_ki       <= 8'd0;
            reg_kd       <= 8'd0;
            reg_setpoint <= 8'd0;
            reg_medicion <= 8'd0;
        end else if (ena && cargar) begin
            case (sel_reg)
                3'd0: reg_kp       <= ui_in;
                3'd1: reg_ki       <= ui_in;
                3'd2: reg_kd       <= ui_in;
                3'd3: reg_setpoint <= ui_in;
                3'd4: reg_medicion <= ui_in;
                default: ;
            endcase
        end
    end

    // Instancia del núcleo PID
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

    // Conexión a los pines físicos
    assign uo_out  = salida_control;
    assign uio_out = {error_negativo, error_cero, saturado, 5'b00000};
    assign uio_oe  = 8'b1110_0000; // [7:5] salidas, [4:0] entradas

    // Evita advertencias de bits sin usar en síntesis
    wire _unused = &{uio_in[7:5], 1'b0};

endmodule

module coprocesador_pid (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       habilitar,
    input  wire [7:0] setpoint,
    input  wire [7:0] medicion,
    input  wire [7:0] kp,
    input  wire [7:0] ki,
    input  wire [7:0] kd,
    output reg  [7:0] salida_control,
    output reg        saturado,
    output reg        error_cero,
    output reg        error_negativo
);

    reg signed [15:0] acumulador;
    reg signed [9:0]  error_anterior;

    wire signed [9:0]  error_actual;
    wire signed [10:0] derivada;
    wire signed [16:0] acum_temp;
    wire signed [15:0] nuevo_acum;
    wire signed [23:0] termino_p;
    wire signed [23:0] termino_i;
    wire signed [23:0] termino_d;
    wire signed [23:0] suma_escalada;

    assign error_actual = $signed({2'b00, setpoint}) - $signed({2'b00, medicion});
    assign derivada     = error_actual - error_anterior;

    assign acum_temp  = acumulador + error_actual;
    assign nuevo_acum = (acum_temp > 17'sd1023)  ?  16'sd1023 :
                        (acum_temp < -17'sd1024) ? -16'sd1024 :
                         acum_temp[15:0];

    assign termino_p = error_actual * $signed({1'b0, kp});
    assign termino_i = nuevo_acum   * $signed({1'b0, ki});
    assign termino_d = derivada     * $signed({1'b0, kd});

    assign suma_escalada = (termino_p + termino_i + termino_d) >>> 4;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acumulador     <= 16'sd0;
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

            if (suma_escalada > 24'sd255) begin
                salida_control <= 8'd255;
                saturado       <= 1'b1;
            end else if (suma_escalada < 24'sd0) begin
                salida_control <= 8'd0;
                saturado       <= 1'b1;
            end else begin
                salida_control <= suma_escalada[7:0];
                saturado       <= 1'b0;
            end
        end
    end

endmodule
