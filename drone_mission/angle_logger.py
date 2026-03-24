#!/usr/bin/env python3
"""
Dual quadrature encoder angle logger - 200Hz CSV output
Tick-accurate, chatter filtered
"""

import pigpio
import time
import csv
import sys
import os
from datetime import datetime

# ── Pin assignments ────────────────────────────────────────────────────────────
ENC1_A = 6    # white
ENC1_B = 13   # green
ENC2_A = 19
ENC2_B = 26

# ── Encoder config ─────────────────────────────────────────────────────────────
PPR            = 1000
COUNT_MODE     = 4
GEAR_RATIO     = 1.0
COUNTS_PER_REV = PPR * COUNT_MODE * GEAR_RATIO
DEG_PER_COUNT  = 360.0 / COUNTS_PER_REV
MIN_PULSE_US   = 300

# ── State ──────────────────────────────────────────────────────────────────────
pin_level = {ENC1_A: 0, ENC1_B: 0, ENC2_A: 0, ENC2_B: 0}
pitch_count    = 0
roll_count     = 0
last_tick      = {'pitch': 0, 'roll': 0}
error_count    = {'pitch': 0, 'roll': 0}
filtered_count = {'pitch': 0, 'roll': 0}
last_pitch_state = 0b00
last_roll_state  = 0b00

QUAD_TABLE = {
    (0b00, 0b01): +1, (0b01, 0b11): +1,
    (0b11, 0b10): +1, (0b10, 0b00): +1,
    (0b00, 0b10): -1, (0b10, 0b11): -1,
    (0b11, 0b01): -1, (0b01, 0b00): -1,
}

def make_callback(axis):
    if axis == 'pitch':
        pin_A, pin_B = ENC1_A, ENC1_B
    else:
        pin_A, pin_B = ENC2_A, ENC2_B

    def callback(gpio, level, tick):
        global pitch_count, roll_count, last_pitch_state, last_roll_state

        dt = tick - last_tick[axis]
        if dt < 0:
            dt += (1 << 32)
        if dt < MIN_PULSE_US:
            filtered_count[axis] += 1
            return
        last_tick[axis] = tick

        pin_level[gpio] = level
        A = pin_level[pin_A]
        B = pin_level[pin_B]
        new_state = (A << 1) | B

        if axis == 'pitch':
            old_state = last_pitch_state
            delta = QUAD_TABLE.get((old_state, new_state), 0)
            if delta == 0 and old_state != new_state:
                error_count['pitch'] += 1
            pitch_count += delta
            last_pitch_state = new_state
        else:
            old_state = last_roll_state
            delta = QUAD_TABLE.get((old_state, new_state), 0)
            if delta == 0 and old_state != new_state:
                error_count['roll'] += 1
            roll_count += delta
            last_roll_state = new_state

    return callback


def run_logger(log_file, duration=None):
    """
    Run the angle logger.
    duration: seconds to log, None = log until Ctrl+C
    """
    global pi, last_pitch_state, last_roll_state

    os.makedirs(os.path.dirname(log_file), exist_ok=True)

    pi = pigpio.pi()
    if not pi.connected:
        print('ERROR: pigpio not running. Run: sudo pigpiod')
        return

    for pin in [ENC1_A, ENC1_B, ENC2_A, ENC2_B]:
        pi.set_mode(pin, pigpio.INPUT)
        pin_level[pin] = pi.read(pin)

    last_pitch_state = (pin_level[ENC1_A] << 1) | pin_level[ENC1_B]
    last_roll_state  = (pin_level[ENC2_A] << 1) | pin_level[ENC2_B]

    cb1a = pi.callback(ENC1_A, pigpio.EITHER_EDGE, make_callback('pitch'))
    cb1b = pi.callback(ENC1_B, pigpio.EITHER_EDGE, make_callback('pitch'))
    cb2a = pi.callback(ENC2_A, pigpio.EITHER_EDGE, make_callback('roll'))
    cb2b = pi.callback(ENC2_B, pigpio.EITHER_EDGE, make_callback('roll'))

    print(f'Logging angles to {log_file} at 200Hz')
    if duration:
        print(f'Duration: {duration}s')
    print('Press Ctrl+C to stop\n')

    start_time = time.time()
    dt = 1.0 / 200.0
    row_count = 0

    try:
        with open(log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t', 'pitch_deg', 'roll_deg',
                             'pitch_count', 'roll_count'])

            next_t = time.time()
            while True:
                now = time.time()
                elapsed = now - start_time

                if duration and elapsed >= duration:
                    break

                if now >= next_t:
                    pitch_deg = pitch_count * DEG_PER_COUNT
                    roll_deg  = roll_count  * DEG_PER_COUNT
                    writer.writerow([
                        round(elapsed, 5),
                        round(pitch_deg, 4),
                        round(roll_deg, 4),
                        pitch_count,
                        roll_count
                    ])
                    row_count += 1
                    next_t += dt

                    # Print at 2Hz
                    if row_count % 100 == 0:
                        print(f'\rt={elapsed:.1f}s  '
                              f'pitch={pitch_deg:+.2f}°  '
                              f'roll={roll_deg:+.2f}°  '
                              f'rows={row_count}    ',
                              end='', flush=True)

    except KeyboardInterrupt:
        pass
    finally:
        cb1a.cancel(); cb1b.cancel()
        cb2a.cancel(); cb2b.cancel()
        pi.stop()
        print(f'\nSaved {row_count} rows to {log_file}')
        print(f'Errors — pitch: {error_count["pitch"]}  roll: {error_count["roll"]}')
        print(f'Filtered — pitch: {filtered_count["pitch"]}  roll: {filtered_count["roll"]}')


def main():
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_file  = sys.argv[1] if len(sys.argv) > 1 \
                else f'/home/pi/logs/angles_{timestamp}.csv'
    duration  = float(sys.argv[2]) if len(sys.argv) > 2 else None
    run_logger(log_file, duration)


if __name__ == '__main__':
    main()
