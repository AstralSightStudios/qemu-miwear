#!/bin/sh
# Run the best1503 / best1700 machine against the AP firmware for a bounded time.
# usage: emu/run.sh <outdir> <seconds> [extra qemu args...]
#   seconds = 0 runs until QEMU exits.
#   GUI=1 opens the panel in a window (the mouse is the finger; the host
#   cursor stays visible); otherwise the run is headless.
#   NSH=<port> serves the console UART on that TCP port with an interactive
#   NuttX shell (emu/nsh.py, through the gdbstub on port NSH+1); attach
#   with `emu/nsh.py console <port>`. Console logs still record everything.
#   BT_PORT=<port> listens there for the peer Bluetooth host (H4 HCI).
#   NOR=<file> backs the NOR flash with that image (factory data, see
#   emu/mknor.py); written back on exit.
#   SENSOR_NOR=<file> backs the sensor hub's NOR flash (q63 best1700, see
#   emu/mksensornor.py); written back on exit.
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

# Pick the console UART: best1700 (q63) uses UART0; best1503 (p67) uses
# UART2; n67 and best1502p (p65, q65) use UART1.
CONSOLE_UART=uart2
case "$PROFILE-$FW" in
    *q63*|*best1700*)
        CONSOLE_UART=uart0
        ;;
    *n67*|*p65*|*q65*)
        CONSOLE_UART=uart1
        ;;
esac

if [ -n "$NSH" ]; then
    NSH_GDB=$((NSH + 1))
    SERIAL_ARGS=""
    for u in uart0 uart1 uart2; do
        if [ "$u" = "$CONSOLE_UART" ]; then
            SERIAL_ARGS="$SERIAL_ARGS -chardev socket,id=$u,host=127.0.0.1,port=$NSH,server=on,wait=off,logfile=$OUT/$u.log -serial chardev:$u"
        else
            SERIAL_ARGS="$SERIAL_ARGS -serial file:$OUT/$u.log"
        fi
    done
    SERIAL_ARGS="$SERIAL_ARGS -S -gdb tcp:127.0.0.1:$NSH_GDB"
else
    SERIAL_ARGS="-serial file:$OUT/uart0.log -serial file:$OUT/uart1.log -serial file:$OUT/uart2.log"
fi

if [ -n "$BT_PORT" ]; then
    BT_ARGS="-chardev socket,id=bt,host=127.0.0.1,port=$BT_PORT,server=on,wait=off"
    BT_ARGS="$BT_ARGS -global driver=bes.btss,property=chardev,value=bt"
fi
if [ -n "$NOR" ]; then
    NOR_ARGS="-M flash-image=$NOR"
fi
if [ -n "$SENSOR_NOR" ]; then
    SENSOR_NOR_ARGS="-M sensor-nor=$SENSOR_NOR"
fi
if [ -n "$PROFILE" ]; then
    PROFILE_ARGS="-M profile=$PROFILE"
fi
if [ -n "$EMMC" ]; then
    EMMC_ARGS="-drive if=sd,index=0,format=raw,file=$EMMC"
fi
mkdir -p "$OUT"
build/qemu-system-arm -M miwear -kernel "$FW/vela_ap.bin" \
    $DISPLAY_ARGS $BT_ARGS $NOR_ARGS $SENSOR_NOR_ARGS $PROFILE_ARGS $EMMC_ARGS -monitor none \
    $SERIAL_ARGS -D "$OUT/qemu.log" "$@" > "$OUT/stdout.log" 2>&1 &
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
