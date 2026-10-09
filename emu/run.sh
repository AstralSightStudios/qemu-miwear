#!/bin/sh
# Run the best1503 machine against the AP firmware for a bounded time.
# usage: emu/run.sh <outdir> <seconds> [extra qemu args...]
# Must be run from the repository root (machine paths are relative).
OUT=$1; SECS=$2; shift 2
FW=${FW:-miwear.watch.p67tc_v3.101.043_full_a4ce8564}
mkdir -p "$OUT"
build/qemu-system-arm -M best1503 -kernel "$FW/vela_ap.bin" \
    -nographic -monitor none \
    -serial file:"$OUT/uart0.log" -serial file:"$OUT/uart1.log" \
    -serial file:"$OUT/uart2.log" \
    -D "$OUT/qemu.log" "$@" > "$OUT/stdout.log" 2>&1 &
PID=$!
i=0
while [ $i -lt "$SECS" ] && kill -0 $PID 2>/dev/null; do
    sleep 1; i=$((i + 1))
done
kill -TERM $PID 2>/dev/null
wait $PID 2>/dev/null
echo "qemu exited after ${i}s"
