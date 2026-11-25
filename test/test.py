import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles


async def load_reg(dut, addr, value):
    dut.ui_in.value = value
    dut.uio_in.value = (1 << 3) | (addr & 0x7)  # cargar=1, ejecutar=0
    await ClockCycles(dut.clk, 1)
    dut.uio_in.value = 0


async def run_pid_step(dut):
    dut.uio_in.value = (1 << 4)  # ejecutar=1, cargar=0
    await ClockCycles(dut.clk, 1)
    dut.uio_in.value = 0


@cocotb.test()
async def test_pid_coprocessor_full(dut):
    dut._log.info("Iniciando prueba completa del Coprocesador PID")

    clock = Clock(dut.clk, 100, units="ns")
    cocotb.start_soon(clock.start())

    # 1. Reset inicial
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 1)
    assert int(dut.uo_out.value) == 0, "La salida no inicio en 0 tras reset"

    # 2. Prueba de SUBIDA: Setpoint = 100 (Kp=x1.0, Ki=x0.25, Kd=x0.25)
    await load_reg(dut, 0, 4)
    await load_reg(dut, 1, 2)
    await load_reg(dut, 2, 2)
    await load_reg(dut, 3, 100)

    planta = 0
    for _ in range(25):
        await load_reg(dut, 4, planta)
        await run_pid_step(dut)
        ctrl = int(dut.uo_out.value)
        planta = planta + ((ctrl - planta) // 4)

    dut._log.info(f"[Subida] Medicion final: {planta}")
    assert 95 <= planta <= 105, f"Error en subida: quedo en {planta} (esperado ~100)"

    # 3. Prueba de BAJADA: Setpoint = 40 (prueba errores y derivadas negativas)
    await load_reg(dut, 3, 40)
    err_neg_visto = False
    for _ in range(30):
        await load_reg(dut, 4, planta)
        await run_pid_step(dut)
        if (int(dut.uio_out.value) >> 7) & 1:
            err_neg_visto = True
        ctrl = int(dut.uo_out.value)
        planta = planta + ((ctrl - planta) // 4)

    dut._log.info(f"[Bajada] Medicion final: {planta}")
    assert err_neg_visto, "No se activo la bandera error_negativo al bajar el setpoint"
    assert 35 <= planta <= 45, f"Error en bajada: quedo en {planta} (esperado ~40)"

    # 4. Prueba de SATURACION y recuperacion a error_cero
    await load_reg(dut, 0, 6)    # Kp = x4.0
    await load_reg(dut, 1, 0)    # Ki = 0
    await load_reg(dut, 2, 0)    # Kd = 0
    await load_reg(dut, 3, 200)  # Setpoint = 200
    await load_reg(dut, 4, 0)    # Medicion = 0 -> P = 800 -> satura a 255
    await run_pid_step(dut)

    assert int(dut.uo_out.value) == 255, f"No saturo a 255 (dio {int(dut.uo_out.value)})"
    assert ((int(dut.uio_out.value) >> 5) & 1) == 1, "No se activo bandera saturado"

    # Recuperacion inmediata con error = 0
    await load_reg(dut, 4, 200)
    await run_pid_step(dut)
    assert int(dut.uo_out.value) == 0, "No salio de saturacion al llegar a error 0"
    assert ((int(dut.uio_out.value) >> 5) & 1) == 0, "Bandera saturado no se apago"
    assert ((int(dut.uio_out.value) >> 6) & 1) == 1, "Bandera error_cero no se activo"

    # 5. Prueba de RESET en caliente
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 3)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 1)
    assert int(dut.uo_out.value) == 0, "La salida no volvio a 0 tras reset en caliente"

    dut._log.info("Todas las pruebas del PID pasaron al 100%")
