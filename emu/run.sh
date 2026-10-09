#!/bin/sh
# Run the best1503 machine against the AP firmware for a bounded time.
# usage: emu/run.sh <outdir> <seconds> [extra qemu args...]
#   seconds = 0 runs until QEMU exits.
#   GUI=1 opens the panel in a window (the mouse is the finger);
#   otherwise the run is headless.
#   BT_PORT=<port> listens there for the peer Bluetooth host (H4 HCI).
#   NOR=<file> backs the NOR flash with that image (factory data, see
#   emu/mknor.py); written back on exit.
# Must be run from the repository root (machine paths are relative).
OUT=$1; SECS=$2; shift 2
FW=${FW:-miwear.watch.p67tc_v3.101.043_full_a4ce8564}
if [ -n "$GUI" ]; then
    DISPLAY_ARGS="-display cocoa"
else
    DISPLAY_ARGS="-nographic"
fi
if [ -n "$BT_PORT" ]; then
    BT_ARGS="-chardev socket,id=bt,host=127.0.0.1,port=$BT_PORT,server=on,wait=off"
    BT_ARGS="$BT_ARGS -global driver=bes.btss,property=chardev,value=bt"
fi
if [ -n "$NOR" ]; then
    NOR_ARGS="-M flash-image=$NOR"
fi
mkdir -p "$OUT"
build/qemu-system-arm -M best1503 -kernel "$FW/vela_ap.bin" \
    $DISPLAY_ARGS $BT_ARGS $NOR_ARGS -monitor none \
    -serial file:"$OUT/uart0.log" -serial file:"$OUT/uart1.log" \
    -serial file:"$OUT/uart2.log" \
    -D "$OUT/qemu.log" "$@" > "$OUT/stdout.log" 2>&1 &
PID=$!
i=0
while { [ "$SECS" -eq 0 ] || [ $i -lt "$SECS" ]; } && kill -0 $PID 2>/dev/null; do
    sleep 1; i=$((i + 1))
done
kill -TERM $PID 2>/dev/null
wait $PID 2>/dev/null
echo "qemu exited after ${i}s"
