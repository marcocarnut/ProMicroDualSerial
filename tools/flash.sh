#!/bin/sh
# Flash a Pro Micro running the Caterina bootloader.
#
# Opening the firmware's first port at 1200 baud and dropping DTR makes it jump
# to the bootloader (see BOOTLOADER_1200BAUD_TOUCH in src/config.h). The
# firmware's ports then disappear and the bootloader enumerates its own CDC
# port for ~8 seconds, which is where avrdude is pointed.
#
# usage: tools/flash.sh [/dev/ttyACM0] [firmware.hex]
set -e
PORT=${1:-/dev/ttyACM0}
HEX=${2:-ProMicroDualUART.hex}

if [ -e "$PORT" ]; then
	echo "Resetting $PORT into the bootloader (1200 baud touch)..."
	stty -F "$PORT" 1200 hupcl
	# open + close with HUPCL drops DTR (if HUPCL was already set, stty's own
	# close did that and the port may be gone: in a subshell, as a failed
	# redirection would end the script)
	( : < "$PORT" ) 2>/dev/null || true

	# wait for the firmware's ports (both of them) to go away, so the
	# second one isn't mistaken for the bootloader's
	i=0
	while { [ -e "$PORT" ] || ls /dev/serial/by-id/*ProMicroDualUART* >/dev/null 2>&1; } && [ $i -lt 30 ]; do
		sleep 0.1; i=$((i + 1))
	done
fi

echo "Waiting for the bootloader port..."
i=0
BLPORT=
while [ $i -lt 100 ]; do
	BLPORT=$(ls /dev/ttyACM* 2>/dev/null | head -n1 || true)
	[ -n "$BLPORT" ] && break
	sleep 0.1
	i=$((i + 1))
done
[ -n "$BLPORT" ] || { echo "Bootloader port not found (double-tap RST to GND to enter it manually)" >&2; exit 1; }
sleep 0.3
echo "Bootloader on $BLPORT"
avrdude -p atmega32u4 -c avr109 -P "$BLPORT" -U flash:w:"$HEX":i
