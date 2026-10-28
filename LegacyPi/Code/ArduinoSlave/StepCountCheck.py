#!/usr/bin/env python3
"""StepCountCheck.py - is the Arduino's step count real or noise?

Motor power ON, cart centred, arms hanging. The cart creeps at about 2 steps/s, then moves
300 steps (about 3 cm) slowly. A healthy count changes by 0-3 per 10 ms. Jumps of
hundreds or thousands mean INT3 (pin 18) is counting noise: check the D5 -> D18 jumper.
    python3 StepCountCheck.py
"""
import struct
import sys
import time

import serial

PORT, BAUD = "/dev/ttyACM0", 115200
M_PER_STEP = 0.638175 / 6400


def main():
    ard = serial.Serial(PORT, BAUD, timeout=0.3, write_timeout=0.2)
    print("Resetting the Arduino...")
    ard.dtr = False
    time.sleep(0.1)
    ard.dtr = True
    time.sleep(3.0)
    ard.reset_input_buffer()
    input("Turn motor power ON (cart centred, arms hanging), then press Enter: ")

    last, glitches, n = None, 0, 0

    def cmd(timer):
        nonlocal last, glitches, n
        ard.write(struct.pack("<i", timer))
        buf = ard.read(2)
        if len(buf) < 2:
            ard.close()
            sys.exit("No reply from the Arduino. It is stuck in its stop loop (a limit switch or the stop "
                     "button was triggered) or the USB reset did not work. Motor power OFF, release the "
                     "E-stop, move the cart off the limit switches, press the Mega's RESET button "
                     "(or unplug and replug its USB), then run this again.")
        steps = struct.unpack(">h", buf)[0]
        if last is not None:
            d = ((steps - last + 32768) & 0xFFFF) - 32768
            if abs(d) > 5:
                glitches += 1
                print(f"  GLITCH: count jumped {d:+d} steps in one 10 ms cycle (now {steps})")
        last = steps
        n += 1
        return steps

    try:
        s0 = cmd(-65535)
        print(f"Start count {s0}. Creeping for 5 s (expect about +10 steps)...")
        t_end = time.monotonic() + 5
        while time.monotonic() < t_end:
            s = cmd(-65535)
            time.sleep(0.01)
        print(f"  after creep: {s - s0:+d} steps")
        s1 = s
        print("Moving about 300 steps at 300 steps/s (expect about +300 steps, 3 cm)...")
        t_end = time.monotonic() + 1.0
        while time.monotonic() < t_end:
            s = cmd(-415)
            if abs(s - s0) > 1500:
                print("  count left +/-1500 steps: stopping")
                break
            time.sleep(0.01)
        moved = ((s - s1 + 32768) & 0xFFFF) - 32768
        print(f"  after move: {moved:+d} steps = {moved * M_PER_STEP * 1000:+.1f} mm by the count")
        print("Measure how far the cart actually moved with a ruler and compare.")
    finally:
        ard.write(struct.pack("<i", -1))   # 0xFFFFFFFF: stop and disable the motor
        ard.close()
    print(f"\n{n} replies, {glitches} glitches.",
          "Count looks healthy." if glitches == 0 else
          "Count is NOT trustworthy: reseat/shorten the D5 -> D18 wire (see notes), then rerun.")
    print("Reset the Arduino (power-cycle or rerun a script that resets it) before driving again.")


if __name__ == "__main__":
    main()
