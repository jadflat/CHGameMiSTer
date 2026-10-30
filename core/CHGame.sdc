derive_pll_clocks
derive_clock_uncertainty

# core specific constraints

# MiSTer's HQ2x blender (sys/hq2x.sv, Blend): its registers all load on
# ce_in, which is ce_x4 of the scandoubler; the glass's pixel is 16 clocks of
# clk_sys (chg_video), so ce_x4 comes every 4 clocks. Paths between them have
# two clocks at least.
set_multicycle_path -setup 2 -from [get_registers {*|Hq2x:Hq2x|Blend:blender|*}] -to [get_registers {*|Hq2x:Hq2x|Blend:blender|*}]
set_multicycle_path -hold 1 -from [get_registers {*|Hq2x:Hq2x|Blend:blender|*}] -to [get_registers {*|Hq2x:Hq2x|Blend:blender|*}]
