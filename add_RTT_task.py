Import("env")

# openocd target and RTT search range from the board definition, so the
# search stays inside RAM (GD32F130C6 has only 4 KB) on every env.
board = env.BoardConfig()
openocd_target = board.get("debug.openocd_target", "stm32f1x")
ram_size = board.get("upload.maximum_ram_size", 4096)

env.AddCustomTarget(
    "RTT Start",
    None,
    "$PROJECT_PACKAGES_DIR/tool-openocd/bin/openocd" +
    " -f interface/stlink.cfg" +
    " -c \"transport select hla_swd\"" +
    " -f target/%s.cfg" % openocd_target +
    " -c init" +
    " -c \"rtt setup 0x20000000 %d \\\"SEGGER RTT\\\"\"" % ram_size +
    " -c \"rtt start\"" +
    " -c \"rtt server start 9090 0\""
)
