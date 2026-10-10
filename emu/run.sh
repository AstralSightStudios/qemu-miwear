#!/bin/sh
# Run the best1503 machine against the AP firmware for a bounded time.
# usage: emu/run.sh <outdir> <seconds> [extra qemu args...]
#   seconds = 0 runs until QEMU exits.
#   GUI=1 opens the panel in a window (the mouse is the finger; the host
#   cursor stays visible); otherwise the run is headless.
#   NSH=<port> serves the console UART (uart2) on that TCP port with an
#   interactive NuttX shell (emu/nsh.py, through the gdbstub on port
#   NSH+1); attach with `emu/nsh.py console <port>`. uart2.log still
#   records everything.
#   BT_PORT=<port> listens there for the peer Bluetooth host (H4 HCI).
#   NOR=<file> backs the NOR flash with that image (factory data, see
#   emu/mknor.py); written back on exit.
#   FW=<dir> picks the firmware (its vela_ap.bin); the machine follows its
#   chip, and the board profile its product. PROFILE=<name> forces a board
#   (PROFILE=help lists them).
#   EMMC=<file> attaches that eMMC image (best1700 devices; build it with
#   emu/mkemmc.py <firmware dir>).
# Must be run from the repository root (machine paths are relative).
OUT=$1; SECS=$2; shift 2
FW=${FW:-miwear.watch.p67tc_v3.101.043_full_a4ce8564}
if [ -n "$GUI" ]; then
    DISPLAY_ARGS="-display cocoa,show-cursor=on"
else
    DISPLAY_ARGS="-nographic"
fi
if [ -n "$NSH" ]; then
    NSH_GDB=$((NSH + 1))
    UART2_ARGS="-chardev socket,id=uart2,host=127.0.0.1,port=$NSH,server=on,wait=off,logfile=$OUT/uart2.log"
    UART2_ARGS="$UART2_ARGS -serial chardev:uart2 -S -gdb tcp:127.0.0.1:$NSH_GDB"
else
    UART2_ARGS="-serial file:$OUT/uart2.log"
fi
if [ -n "$BT_PORT" ]; then
    BT_ARGS="-chardev socket,id=bt,host=127.0.0.1,port=$BT_PORT,server=on,wait=off"
    BT_ARGS="$BT_ARGS -global driver=bes.btss,property=chardev,value=bt"
fi
if [ -n "$NOR" ]; then
    NOR_ARGS="-M flash-image=$NOR"
fi
if [ -n "$PROFILE" ]; then
    PROFILE_ARGS="-M profile=$PROFILE"
fi
if [ -n "$EMMC" ]; then
    EMMC_ARGS="-drive if=sd,index=0,format=raw,file=$EMMC"
fi
mkdir -p "$OUT"
build/qemu-system-arm -M miwear -kernel "$FW/vela_ap.bin" \
    $DISPLAY_ARGS $BT_ARGS $NOR_ARGS $PROFILE_ARGS $EMMC_ARGS -monitor none \
    -serial file:"$OUT/uart0.log" -serial file:"$OUT/uart1.log" \
    $UART2_ARGS -D "$OUT/qemu.log" "$@" > "$OUT/stdout.log" 2>&1 &
PID=$!
if [ -n "$NSH" ]; then
    python3 emu/nsh.py inject "$NSH_GDB" "$FW/vela_ap.bin" &
fi
i=0
while { [ "$SECS" -eq 0 ] || [ $i -lt "$SECS" ]; } && kill -0 $PID 2>/dev/null; do
    sleep 1; i=$((i + 1))
done
kill -TERM $PID 2>/dev/null
wait $PID 2>/dev/null
echo "qemu exited after ${i}s"
