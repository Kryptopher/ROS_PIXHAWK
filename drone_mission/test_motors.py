import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode, CommandLong
from mavros_msgs.msg import State
import time

class MotorTest(Node):
    def __init__(self):
        super().__init__('motor_test')
        self.state = None

        self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.arm_client  = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode,     '/mavros/set_mode')
        self.cmd_client  = self.create_client(CommandLong, '/mavros/cmd/command')

        self.get_logger().info('Waiting for MAVROS services...')
        self.arm_client.wait_for_service(timeout_sec=10.0)
        self.mode_client.wait_for_service(timeout_sec=10.0)
        self.cmd_client.wait_for_service(timeout_sec=10.0)

    def state_cb(self, msg): self.state = msg

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        f = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        self.get_logger().info(f'Mode {mode}: OK')

    def arm(self, value):
        req = CommandBool.Request()
        req.value = value
        f = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        result = f.result().success if f.result() else False
        self.get_logger().info(f'{"Arm" if value else "Disarm"}: {"OK" if result else "FAILED"}')
        return result

    def set_throttle(self, percent):
        """Send throttle override to all motors 0-100%"""
        from mavros_msgs.msg import OverrideRCIn
        # RC channel 3 is throttle, range 1000-2000
        pwm = int(1000 + (percent / 100.0) * 1000)
        self.get_logger().info(f'Throttle: {percent:.0f}% (PWM={pwm})')

        msg = OverrideRCIn()
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
        msg.channels[2] = pwm  # channel 3 = index 2
        self.rc_pub.publish(msg)

    def ramp(self, start_pct, end_pct, duration_s, step_s=0.1):
        """Ramp throttle from start to end over duration"""
        steps = int(duration_s / step_s)
        for i in range(steps + 1):
            pct = start_pct + (end_pct - start_pct) * (i / steps)
            self.set_throttle(pct)
            time.sleep(step_s)
            rclpy.spin_once(self, timeout_sec=0.0)

    def run(self):
        from mavros_msgs.msg import OverrideRCIn
        self.rc_pub = self.create_publisher(OverrideRCIn,
                                            '/mavros/rc/override', 10)

        # Wait for connection
        self.get_logger().info('Waiting for FCU...')
        start = time.time()
        while self.state is None and time.time() - start < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f'Connected! Mode={self.state.mode}')

        # Arm in STABILIZE
        self.set_mode('STABILIZE')
        time.sleep(0.5)
        input('\nProps OFF. Press ENTER to ARM...')
        if not self.arm(True):
            self.get_logger().error('Arm failed')
            return

        time.sleep(1.0)
        self.get_logger().info('Armed! Starting motor test...')

        # ── Test sequence ─────────────────────────────────────
        input('\nPress ENTER to ramp 0→100% (3s)...')
        self.ramp(0, 100, 3.0)

        input('\nPress ENTER to ramp 100→0% (3s)...')
        self.ramp(100, 0, 3.0)
        time.sleep(1.0)

        input('\nPress ENTER to ramp 0→30% (2s)...')
        self.ramp(0, 30, 2.0)
        time.sleep(1.0)

        input('\nPress ENTER to ramp 30→60% (2s)...')
        self.ramp(30, 60, 2.0)

        input('\nPress ENTER to ramp back to 0%...')
        self.ramp(60, 0, 2.0)

        # ── Disarm ────────────────────────────────────────────
        # Make sure throttle is at 0 before disarming
        self.set_throttle(0)
        time.sleep(2.0)
        rclpy.spin_once(self, timeout_sec=0.0)

        # Retry disarm
        for i in range(3):
            if self.arm(False):
                break
            self.get_logger().warn(f'Disarm attempt {i+1} failed, retrying...')
            time.sleep(1.0)

def main():
    rclpy.init()
    node = MotorTest()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
