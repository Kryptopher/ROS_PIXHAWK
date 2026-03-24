#!/usr/bin/env python3
"""
Dual quadrature encoder angle tracker - DEBUG MODE
"""

import pigpio
import math
import time

# ── Pin assignments ────────────────────────────────────────────────────────────
ENC1_A = 27   # green  = A
ENC1_B = 17   # white  = B
ENC2_A = 13
ENC2_B = 19

# ── Encoder config ─────────────────────────────────────────────────────────────
PPR            = 1000
COUNT_MODE     = 4
GEAR_RATIO     = 1.0
COUNTS_PER_REV = PPR * COUNT_MODE * GEAR_RATIO
DEG_PER_COUNT  = 360.0 / COUNTS_PER_REV

# ── State ──────────────────────────────────────────────────────────────────────
pitch_count = 0
roll_count  = 0

QUAD_TABLE = {
    (0b00, 0b01): +1,
    (0b01, 0b11): +1,
    (0b11, 0b10): +1,
    (0b10, 0b00): +1,
    (0b00, 0b10): -1,
    (0b10, 0b11): -1,
    (0b11, 0b01): -1,
    (0b01, 0b00): -1,
}

last_pitch_state = 0b00
last_roll_state  = 0b00

# ── Debug state ────────────────────────────────────────────────────────────────
debug_log        = []
error_count      = {'pitch': 0, 'roll': 0}
DEBUG_MAX_ROWS   = 500
LOG_EVERY_N      = 10
callback_counter = {'pitch': 0, 'roll': 0}


def make_callback(axis: str):
    if axis == 'pitch':
        pin_A, pin_B = ENC1_A, ENC1_B
    else:
        pin_A, pin_B = ENC2_A, ENC2_B

    def callback(gpio, level, tick):
        global pitch_count, roll_count, last_pitch_state, last_roll_state

        A = pi.read(pin_A)
        B = pi.read(pin_B)
        new_state = (A << 1) | B

        if axis == 'pitch':
            old_state = last_pitch_state
            delta = QUAD_TABLE.get((old_state, new_state), 0)
            if delta == 0 and old_state != new_state:
                error_count['pitch'] += 1
            pitch_count += delta
            last_pitch_state = new_state
            callback_counter['pitch'] += 1
            n = callback_counter['pitch']
        else:
            old_state = last_roll_state
            delta = QUAD_TABLE.get((old_state, new_state), 0)
            if delta == 0 and old_state != new_state:
                error_count['roll'] += 1
            roll_count += delta
            last_roll_state = new_state
            callback_counter['roll'] += 1
            n = callback_counter['roll']

        if n % LOG_EVERY_N == 0 and len(debug_log) < DEBUG_MAX_ROWS:
            debug_log.append((tick, pitch_count, roll_count))

    return callback


def print_debug_report():
    print("\n" + "=" * 60)
    print("  DEBUG REPORT")
    print("=" * 60)

    print(f"\n  Config:")
    print(f"    PPR={PPR}  COUNT_MODE={COUNT_MODE}x  GEAR_RATIO={GEAR_RATIO}")
    print(f"    Counts/rev = {COUNTS_PER_REV:.0f}")
    print(f"    Deg/count  = {DEG_PER_COUNT:.4f}°")

    print(f"\n  Final counts:")
    print(f"    Pitch: {pitch_count:+6d} counts  →  {pitch_count * DEG_PER_COUNT:+7.2f}°")
    print(f"    Roll:  {roll_count:+6d} counts  →  {roll_count  * DEG_PER_COUNT:+7.2f}°")

    print(f"\n  Callback totals:")
    print(f"    Pitch callbacks: {callback_counter['pitch']}")
    print(f"    Roll  callbacks: {callback_counter['roll']}")

    print(f"\n  Invalid transitions (missed edges / noise):")
    print(f"    Pitch errors: {error_count['pitch']}")
    print(f"    Roll  errors: {error_count['roll']}")

    if debug_log:
        print(f"\n  Count log (every {LOG_EVERY_N} callbacks, showing last 20):")
        print(f"    {'tick':>12}  {'pitch_cnt':>10}  {'roll_cnt':>10}  {'pitch_°':>9}  {'roll_°':>9}")
        print(f"    {'-'*58}")
        for tick, pc, rc in debug_log[-20:]:
            print(f"    {tick:>12}  {pc:>10}  {rc:>10}  {pc*DEG_PER_COUNT:>+9.2f}  {rc*DEG_PER_COUNT:>+9.2f}")

        reversals = 0
        for i in range(2, len(debug_log)):
            dp = debug_log[i][1] - debug_log[i-1][1]
            pp = debug_log[i-1][1] - debug_log[i-2][1]
            if dp != 0 and pp != 0 and (dp > 0) != (pp > 0):
                reversals += 1
        print(f"\n  Direction reversals mid-move: {reversals}")
        if reversals > 5:
            print("    ⚠  High reversal count — possible wiring noise or loose encoder shaft")

    print(f"\n  Expected for 90° move: ~{int(COUNTS_PER_REV / 4)} counts on active axis")
    print("=" * 60 + "\n")


def main():
    global pi, last_pitch_state, last_roll_state

    pi = pigpio.pi()
    if not pi.connected:
        print("ERROR: Could not connect to pigpio daemon. Run: sudo pigpiod")
        return

    for pin in [ENC1_A, ENC1_B, ENC2_A, ENC2_B]:
        
    A1 = pi.read(ENC1_A); B1 = pi.read(ENC1_B)
    A2 = pi.read(ENC2_A); B2 = pi.read(ENC2_B)
    last_pitch_state = (A1 << 1) | B1
    last_roll_state  = (A2 << 1) | B2

    cb1a = pi.callback(ENC1_A, pigpio.EITHER_EDGE, make_callback('pitch'))
    cb1b = pi.callback(ENC1_B, pigpio.EITHER_EDGE, make_callback('pitch'))
    cb2a = pi.callback(ENC2_A, pigpio.EITHER_EDGE, make_callback('roll'))
    cb2b = pi.callback(ENC2_B, pigpio.EITHER_EDGE, make_callback('roll'))

    print(f"DEBUG MODE  |  {COUNTS_PER_REV:.0f} counts/rev  |  {DEG_PER_COUNT:.4f}°/count")
    print("Move the gimbal ~90° on ONE axis, then press Ctrl+C.\n")
    print(f"  {'Pitch':>10}  {'Roll':>10}  {'PitchCnt':>10}  {'RollCnt':>10}  {'Errs P/R':>10}")
    print(f"  {'-'*56}")

    ALPHA = 0.3
    pitch_disp = roll_disp = 0.0

    try:
        while True:
            pitch_raw = pitch_count * DEG_PER_COUNT
            roll_raw  = roll_count  * DEG_PER_COUNT
            pitch_disp = ALPHA * pitch_raw + (1 - ALPHA) * pitch_disp
            roll_disp  = ALPHA * roll_raw  + (1 - ALPHA) * roll_disp

            print(
                f"\r  {pitch_disp:>+9.2f}°  {roll_disp:>+9.2f}°  "
                f"{pitch_count:>+10d}  {roll_count:>+10d}  "
                f"{error_count['pitch']}/{error_count['roll']}     ",
                end="", flush=True
            )
            time.sleep(0.05)

    except KeyboardInterrupt:
        print_debug_report()
    finally:
        cb1a.cancel(); cb1b.cancel()
        cb2a.cancel(); cb2b.cancel()
        pi.stop()


if __name__ == "__main__":
    main()
