import csv
import sys
import os

def load_log(path):
    rows = []
    with open(path) as f:
        for row in csv.DictReader(f):
            rows.append({k: float(v) if k not in ('mode','armed') else v
                         for k, v in row.items()})
    return rows

def plot(rows, out_dir):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("Install matplotlib: pip3 install matplotlib --break-system-packages")
        return

    os.makedirs(out_dir, exist_ok=True)
    t = [r['t'] for r in rows]

    # ── Plot 1: XY trajectory ─────────────────────────────────
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot([r['act_x'] for r in rows], [r['act_y'] for r in rows],
            label='actual', color='steelblue', linewidth=1.5)
    ax.plot([r['cmd_px'] for r in rows], [r['cmd_py'] for r in rows],
            label='commanded pos', color='orange', linewidth=1, linestyle='--')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('XY trajectory')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    fig.savefig(f'{out_dir}/xy_trajectory.png', dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved xy_trajectory.png')

    # ── Plot 2: Velocity X — commanded vs actual ──────────────
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    for i, (axis, label) in enumerate([('x','vx'), ('y','vy'), ('z','vz')]):
        axes[i].plot(t, [r[f'act_v{axis}'] for r in rows],
                     label=f'actual {label}', color='steelblue', linewidth=1)
        axes[i].plot(t, [r[f'cmd_v{axis}'] for r in rows],
                     label=f'commanded {label}', color='coral',
                     linewidth=1, linestyle='--')
        axes[i].set_ylabel(f'{label} (m/s)')
        axes[i].legend(loc='upper right', fontsize=8)
        axes[i].grid(True, alpha=0.3)
    axes[-1].set_xlabel('time (s)')
    axes[0].set_title('Commanded vs actual velocity')
    fig.tight_layout()
    fig.savefig(f'{out_dir}/velocity.png', dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved velocity.png')

    # ── Plot 3: Position X Y Z over time ─────────────────────
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    for i, axis in enumerate(['x', 'y', 'z']):
        axes[i].plot(t, [r[f'act_{axis}'] for r in rows],
                     label=f'actual {axis}', color='steelblue', linewidth=1)
        axes[i].plot(t, [r[f'cmd_p{axis}'] for r in rows],
                     label=f'cmd pos {axis}', color='orange',
                     linewidth=1, linestyle='--')
        axes[i].set_ylabel(f'{axis} (m)')
        axes[i].legend(loc='upper right', fontsize=8)
        axes[i].grid(True, alpha=0.3)
    axes[-1].set_xlabel('time (s)')
    axes[0].set_title('Position over time')
    fig.tight_layout()
    fig.savefig(f'{out_dir}/position.png', dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved position.png')

    # ── Plot 4: Position error magnitude ─────────────────────
    import math
    error = [math.sqrt(
        (r['act_x']-r['cmd_px'])**2 +
        (r['act_y']-r['cmd_py'])**2 +
        (r['act_z']-r['cmd_pz'])**2) for r in rows]

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t, error, color='crimson', linewidth=1)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('position error (m)')
    ax.set_title('Position error magnitude over time')
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(f'{out_dir}/position_error.png', dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved position_error.png')

    print(f'\nAll plots saved to {out_dir}/')

def main():
    if len(sys.argv) < 2:
        print('Usage: python3 plot_flight.py <log.csv> [output_dir]')
        return
    log_path = sys.argv[1]
    out_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(log_path)
    rows = load_log(log_path)
    print(f'Loaded {len(rows)} rows from {log_path}')
    plot(rows, out_dir)

if __name__ == '__main__':
    main()
