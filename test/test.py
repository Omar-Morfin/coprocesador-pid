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
async def test_pid_coprocessor(dut):
    dut._log.info("Start PID Coprocessor test")

    clock = Clock(dut.clk, 100, units="ns")
    cocotb.start_soon(clock.start())

    # Reset
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 1)

    # Cargar parámetros: Kp=4 (x1.0), Ki=2 (x0.25), Kd=2 (x0.25), Setpoint=100
    await load_reg(dut, 0, 4)
    await load_reg(dut, 1, 2)
    await load_reg(dut, 2, 2)
    await load_reg(dut, 3, 100)

    # Simular 20 ciclos de control en lazo cerrado
    planta = 0
    for _ in range(20):
        await load_reg(dut, 4, planta)
        await run_pid_step(dut)
        ctrl = int(dut.uo_out.value)
        planta = planta + ((ctrl - planta) // 4)

    dut._log.info(f"Medicion final alcanzada: {planta}")
    assert planta >= 95, f"El PID no convergio cerca de 100 (quedo en {planta})"
