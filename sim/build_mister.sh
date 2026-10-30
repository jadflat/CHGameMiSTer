#!/bin/sh
# Builds sim/out/mister/mister.exe (sim/tb_mister.cpp) and sim/out/datcard/datcard.exe (sim/tb_datcard.cpp).
cd "$(dirname "$0")/.."
R=core/rtl; P=core/rtl/periph; C=core/rtl/cpu
sh sim/vbuild.sh mister mister_top sim/tb_mister.cpp sim/hps_io.vlt sim/mister_top.sv core/sys/hps_io.sv \
    $R/chg_throttle.sv $R/chg_speed.sv $R/chg_loader.sv $R/chg_save.sv $P/chg_card.sv $P/chg_datcard.sv \
    $R/chg_soc.sv $R/chg_dpram.sv $C/chg_cpu.sv $C/chg_cpu_decode.sv $C/chg_cpu_div.sv $C/chg_pfic.sv \
    $P/chg_periph_sim.sv $P/chg_systick.sv $P/chg_gpio.sv $P/chg_rcc.sv $P/chg_tim.sv $P/chg_flashctl.sv \
    $P/chg_adc.sv $P/chg_misc.sv $P/chg_spi_tb_top.sv $P/chg_spi.sv $P/chg_dma.sv $P/chg_sdspi.sv $R/st7735.sv || exit 1
sh sim/vbuild.sh datcard chg_datcard sim/tb_datcard.cpp $P/chg_datcard.sv
