import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State, PositionTarget
from nav_msgs.msg import Odometry
import csv, time, math, sys
import threading
import subprocess
from datetime import datetime
MASK_POS_ONLY    = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
                    PositionTarget.IGNORE_VZ  | PositionTarget.IGNORE_AFX |
                    PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                    PositionTarget.IGNORE_YAW_RATE)
MASK_POS_VEL_ACC = PositionTarget.IGNORE_YAW_RATE

class MissionExecutor(Node):
    def __init__(self, mission_file, shaper_type='none',
                 rope_length=1.0, damping=0.02):
        super().__init__('mission_executor')
        self.state           = State()
        self.current_pos     = None
        self.current_vel     = None
        self.mission         = []
        self.takeoff_alt     = 5.0
        self.mission_file    = mission_file
        self.shaper_type     = shaper_type
        self.rope_length     = rope_length
        self.damping         = damping
        self._current_cmd    = None
        self._mission_start  = None
        self._seg_index      = 0
        self._mission_active = False
        self._last_debug_t   = 0.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)

        self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.create_subscription(Odometry, '/mavros/local_position/odom',
                                 self.odom_cb, sensor_qos)

        self.raw_pub = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', 10)

        self.create_timer(0.02, self._mission_tick)

        self.arm_client     = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client    = self.create_client(SetMode,     '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL,  '/mavros/cmd/takeoff')

        self.get_logger().info('Waiting for MAVROS services...')
        self.arm_client.wait_for_service(timeout_sec=10.0)
        self.mode_client.wait_for_service(timeout_sec=10.0)
        self.takeoff_client.wait_for_service(timeout_sec=10.0)

    def state_cb(self, msg): self.state = msg
    def odom_cb(self, msg):
        self.current_pos = msg.pose.pose.position
        self.current_vel = msg.twist.twist.linear

    def pub_raw(self, mask, px=0.0, py=0.0, pz=0.0,
                vx=0.0, vy=0.0, vz=0.0, yaw=0.0):
        msg = PositionTarget()
        msg.header.stamp     = self.get_clock().now().to_msg()
        msg.header.frame_id  = 'map'
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask        = mask
        msg.position.x = px; msg.position.y = py; msg.position.z = pz
        msg.velocity.x = vx; msg.velocity.y = vy; msg.velocity.z = vz
        msg.yaw = yaw
        self.raw_pub.publish(msg)

    def pub_raw_acc(self, px=0.0, py=0.0, pz=0.0,
                    vx=0.0, vy=0.0, vz=0.0,
                    ax=0.0, ay=0.0, az=0.0, yaw=0.0):
        msg = PositionTarget()
        msg.header.stamp     = self.get_clock().now().to_msg()
        msg.header.frame_id  = 'map'
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask        = MASK_POS_VEL_ACC
        msg.position.x = px; msg.position.y = py; msg.position.z = pz
        msg.velocity.x = vx; msg.velocity.y = vy; msg.velocity.z = vz
        msg.acceleration_or_force.x = ax
        msg.acceleration_or_force.y = ay
        msg.acceleration_or_force.z = az
        msg.yaw = yaw
        self.raw_pub.publish(msg)

    def compute_shaper_impulses(self):
        wn   = math.sqrt(9.81 / self.rope_length)
        zeta = self.damping
        wd   = wn * math.sqrt(1.0 - zeta**2)
        T_d  = math.pi / wd
        K    = math.exp(-zeta * math.pi / math.sqrt(1.0 - zeta**2))
        if self.shaper_type == 'ZV':
            A1 = 1.0 / (1.0 + K)
            A2 = K   / (1.0 + K)
            return [(A1, 0.0), (A2, T_d)]
        elif self.shaper_type == 'ZVD':
            K2    = K * K
            denom = 1.0 + 2*K + K2
            return [(1.0/denom, 0.0), (2*K/denom, T_d), (K2/denom, 2*T_d)]
        else:
            return [(1.0, 0.0)]

    def build_profile(self, vx, vy, vz, ax, duration,
                      profile='trap', dt=0.02):
        samples = []
        t = 0.0
        if profile == 'step':
            while t <= duration + 1e-9:
                samples.append((t, vx, vy, vz))
                t += dt
        elif profile == 'trap':
            speed  = math.sqrt(vx**2 + vy**2 + vz**2)
            t_ramp = (speed / ax) if ax > 1e-6 else 0.0
            while t <= duration + 1e-9:
                if t < t_ramp:
                    s = t / t_ramp
                elif t > duration - t_ramp:
                    s = (duration - t) / t_ramp
                else:
                    s = 1.0
                s = max(0.0, min(1.0, s))
                samples.append((t, vx*s, vy*s, vz*s))
                t += dt
        elif profile == 'sine':
            while t <= duration + 1e-9:
                s = math.sin(math.pi * t / duration)
                samples.append((t, vx*s, vy*s, vz*s))
                t += dt
        else:
            while t <= duration + 1e-9:
                samples.append((t, vx, vy, vz))
                t += dt
        return samples

    def convolve_shaper(self, samples, impulses, dt=0.02):
        t_end  = samples[-1][0] + impulses[-1][1]
        shaped = []
        t = 0.0
        while t <= t_end + 1e-9:
            svx = svy = svz = 0.0
            for (amp, t_imp) in impulses:
                v    = self._interp(samples, t - t_imp)
                svx += amp * v[0]
                svy += amp * v[1]
                svz += amp * v[2]
            shaped.append((t, svx, svy, svz))
            t += dt
        return shaped

    def _interp(self, samples, t):
        if t <= samples[0][0]:  return samples[0][1:]
        if t >= samples[-1][0]: return samples[-1][1:]
        for i in range(len(samples)-1):
            t0, t1 = samples[i][0], samples[i+1][0]
            if t0 <= t <= t1:
                a = (t - t0) / (t1 - t0)
                return tuple(samples[i][j+1] +
                             a*(samples[i+1][j+1] - samples[i][j+1])
                             for j in range(3))
        return (0.0, 0.0, 0.0)

    def _mission_tick(self):
        if not self._mission_active:
            return

        now = time.time() - self._mission_start

        while self._seg_index < len(self.mission):
            seg = self.mission[self._seg_index]
            if now < seg['t_start']:
                break

            self.get_logger().info(
                f"t={now:.1f}s  cmd {self._seg_index+1}/"
                f"{len(self.mission)}  mode={seg['mode']}  "
                f"vel=({seg['vx']:.2f},{seg['vy']:.2f},{seg['vz']:.2f})")

            if seg['mode'] == 'pos':
                self._current_cmd = ('pos', seg)

            elif seg['mode'] == 'vel':
                duration = seg['t_end'] - seg['t_start']
                samples  = self.build_profile(
                    seg['vx'], seg['vy'], seg['vz'],
                    seg['ax'], duration, seg['profile'])
                impulses = self.compute_shaper_impulses()
                if self.shaper_type != 'none':
                    profile = self.convolve_shaper(samples, impulses)
                    self.get_logger().info(
                        f"  Shaper: {self.shaper_type} "
                        f"T={impulses[-1][1]:.2f}s "
                        f"L={self.rope_length}m")
                else:
                    profile = samples

                ex = self.current_pos.x if self.current_pos else 0.0
                ey = self.current_pos.y if self.current_pos else 0.0
                ez = self.current_pos.z if self.current_pos else 0.0
                self._current_cmd = ('vel', seg, profile,
                                      time.time(), ex, ey, ez)
            self._seg_index += 1

        self._publish_current_cmd()

        if self._seg_index >= len(self.mission):
            self._mission_active = False
            self.get_logger().info('Mission complete — all commands fired')

    def _publish_current_cmd(self):
        if self._current_cmd is None:
            return

        cmd = self._current_cmd
        now = time.time()
        debug = (now - self._last_debug_t) >= 0.5

        if cmd[0] == 'pos':
            seg = cmd[1]
            if debug and self.current_pos:
                act = self.current_pos
                self.get_logger().info(
                    f"[pos] cmd=({seg['x']:.2f},{seg['y']:.2f},{seg['z']:.2f}) "
                    f"act=({act.x:.2f},{act.y:.2f},{act.z:.2f}) "
                    f"err=({seg['x']-act.x:.2f},"
                    f"{seg['y']-act.y:.2f},"
                    f"{seg['z']-act.z:.2f})")
                self._last_debug_t = now
            self.pub_raw(MASK_POS_ONLY,
                         px=seg['x'], py=seg['y'], pz=seg['z'])

        elif cmd[0] == 'vel':
            _, seg, profile, cmd_start, ex, ey, ez = cmd
            elapsed = time.time() - cmd_start
            dt      = 0.02

            v_now  = self._interp(profile, elapsed)
            v_prev = self._interp(profile, max(0.0, elapsed - dt))

            svx = v_now[0]; svy = v_now[1]; svz = v_now[2]
            sax = (v_now[0] - v_prev[0]) / dt
            say = (v_now[1] - v_prev[1]) / dt
            saz = (v_now[2] - v_prev[2]) / dt

            if debug and self.current_pos and self.current_vel:
                ap = self.current_pos
                av = self.current_vel
                self.get_logger().info(
                    f"[vel] t={elapsed:.2f}s "
                    f"cmd_v=({svx:.3f},{svy:.3f},{svz:.3f}) "
                    f"act_v=({av.x:.3f},{av.y:.3f},{av.z:.3f}) "
                    f"cmd_a=({sax:.2f},{say:.2f},{saz:.2f}) "
                    f"exp_p=({ex:.2f},{ey:.2f},{ez:.2f}) "
                    f"act_p=({ap.x:.2f},{ap.y:.2f},{ap.z:.2f})")
                self._last_debug_t = now

            if abs(svx) < 0.001 and abs(svy) < 0.001 and abs(svz) < 0.001:
                if self.current_pos:
                    self.pub_raw(MASK_POS_ONLY,
                                 px=self.current_pos.x,
                                 py=self.current_pos.y,
                                 pz=self.current_pos.z)
                return

            ex += svx * dt
            ey += svy * dt
            ez += svz * dt
            self._current_cmd = ('vel', seg, profile,
                                  cmd_start, ex, ey, ez)

            self.pub_raw_acc(px=ex,  py=ey,  pz=ez,
                             vx=svx, vy=svy, vz=svz,
                             ax=sax, ay=say, az=saz)

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        f = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        return f.result()

    def arm(self, value):
        req = CommandBool.Request()
        req.value = value
        f = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        return f.result()

    def takeoff(self, altitude):
        req = CommandTOL.Request()
        req.altitude  = altitude
        req.min_pitch = 0.0
        req.yaw       = 0.0
        req.latitude  = 0.0
        req.longitude = 0.0
        f = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        return f.result()

    def wait_for_state(self, armed=None, mode=None, timeout=10.0):
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if armed is not None and self.state.armed != armed: continue
            if mode  is not None and self.state.mode  != mode:  continue
            return True
        return False

    def load_mission(self):
        with open(self.mission_file) as f:
            rows = list(csv.DictReader(f))

        self.takeoff_alt = 5.0
        self.manual_takeoff  = False
        self.mission     = []

        for i, row in enumerate(rows):
            seg_type = row['type'].strip()
            t_start  = float(row['t'])
            t_end    = float(rows[i+1]['t']) if i+1 < len(rows) else t_start

            if seg_type == 'takeoff':
                self.takeoff_alt = float(row['z'])
                self.get_logger().info(
                    f'Takeoff altitude: {self.takeoff_alt}m')
                continue

            if seg_type == 'manual_takeoff':
                self.manual_takeoff = True
                self.get_logger().info('Manual takeoff mode')
                continue

            if seg_type == 'end':
                break

            self.mission.append({
                't_start': t_start,
                't_end':   t_end,
                'type':    seg_type,
                'mode':    row.get('mode', 'vel').strip(),
                'profile': row.get('profile', 'trap').strip(),
                'x':  float(row['x']),
                'y':  float(row['y']),
                'z':  float(row['z']),
                'vx': float(row['vx']),
                'vy': float(row['vy']),
                'vz': float(row['vz']),
                'ax': float(row['ax']),
            })

        self.get_logger().info(f'Loaded {len(self.mission)} commands')
        for seg in self.mission:
            self.get_logger().info(
                f"  t={seg['t_start']:.0f}s  {seg['mode']:3s}  "
                f"vel=({seg['vx']:.2f},{seg['vy']:.2f},{seg['vz']:.2f})")

    def run(self):
        self.load_mission()

        self.get_logger().info('Waiting for position data...')
        start = time.time()
        while self.current_pos is None and time.time() - start < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.current_pos is None:
            self.get_logger().error('No position data')
            return

        if self.manual_takeoff:
            # Wait for armed state
            self.get_logger().info('Manual takeoff mode — waiting for you to arm and takeoff...')
            self.get_logger().info('Take off manually, switch to LOITER, then press ENTER to start mission')
            input('\nPress ENTER when ready to start mission...')
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_pos:
                self.get_logger().info(
                    f'Starting at ({self.current_pos.x:.1f},{self.current_pos.y:.1f},{self.current_pos.z:.1f})')
        else:
            # GUIDED mode
            self.set_mode('GUIDED')
            if not self.wait_for_state(mode='GUIDED'):
                self.get_logger().error('GUIDED failed')
                return
            self.get_logger().info('GUIDED confirmed')

            # Arm
            self.arm(True)
            if not self.wait_for_state(armed=True, timeout=5.0):
                self.get_logger().error('Arm failed')
                return
            self.get_logger().info('Armed!')

            # Takeoff
            self.get_logger().info(f'Taking off to {self.takeoff_alt}m...')
            if not self.takeoff(self.takeoff_alt).success:
                self.get_logger().error('Takeoff failed')
                return

            # Wait for altitude
            start = time.time()
            while time.time() - start < 20.0:
                rclpy.spin_once(self, timeout_sec=0.0)
                if self.current_pos and \
                   self.current_pos.z >= self.takeoff_alt * 0.85:
                    self.get_logger().info(
                        f'Reached {self.current_pos.z:.1f}m — starting mission!')
                    break
                time.sleep(0.1)

        self.get_logger().info('--- MISSION START ---')
        self._mission_start  = time.time()
        self._seg_index      = 0
        self._mission_active = True
        
        # Start angle logger in background
        angle_log = f'/home/pi/logs/angles_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
        angle_proc = subprocess.Popen(
            ['python3', '/home/pi/ROS_PIXHAWK/drone_mission/angle_logger.py',
             angle_log],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.get_logger().info(f'Angle logger started → {angle_log}')
        
        while self._mission_active:
            rclpy.spin_once(self, timeout_sec=0.02)
        
        # Stop angle logger
        angle_proc.terminate()
        angle_proc.wait()
        self.get_logger().info('Angle logger stopped')

        self.get_logger().info('Landing...')
        self.set_mode('LAND')
        start = time.time()
        while time.time() - start < 20.0:
            rclpy.spin_once(self, timeout_sec=0.0)
            if not self.state.armed:
                break
            time.sleep(0.05)

        self.get_logger().info('Done.')

def main():
    mission_file = sys.argv[1] if len(sys.argv) > 1 else '/home/pi/mission.csv'
    shaper_type  = sys.argv[2] if len(sys.argv) > 2 else 'none'
    rope_length  = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0

    rclpy.init()
    node = MissionExecutor(mission_file,
                           shaper_type=shaper_type,
                           rope_length=rope_length)
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
