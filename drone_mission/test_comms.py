import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
import time

class CommsTest(Node):
    def __init__(self):
        super().__init__('comms_test')
        self.state = None

        self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.arm_client  = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.get_logger().info('Waiting for MAVROS...')
        self.arm_client.wait_for_service(timeout_sec=10.0)
        self.mode_client.wait_for_service(timeout_sec=10.0)

    def state_cb(self, msg):
        if self.state is None:
            self.get_logger().info(
                f'Connected! FCU: armed={msg.armed} mode={msg.mode}')
        self.state = msg

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        f = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        success = f.result().mode_sent if f.result() else False
        self.get_logger().info(f'Set mode {mode}: {"OK" if success else "FAILED"}')
        return success

    def arm(self, value):
        req = CommandBool.Request()
        req.value = value
        f = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        success = f.result().success if f.result() else False
        self.get_logger().info(f'{"Arm" if value else "Disarm"}: {"OK" if success else "FAILED"}')
        return success

    def run(self):
        # Wait for connection
        self.get_logger().info('Waiting for FCU connection...')
        start = time.time()
        while self.state is None and time.time() - start < 15.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.state is None:
            self.get_logger().error('No connection after 15s')
            return

        # Print state
        self.get_logger().info('--- COMMS TEST ---')
        self.get_logger().info(f'Mode:  {self.state.mode}')
        self.get_logger().info(f'Armed: {self.state.armed}')
        self.get_logger().info(f'Connected: {self.state.connected}')

        # Test mode change
        input('\nPress ENTER to test STABILIZE mode...')
        self.set_mode('STABILIZE')
        time.sleep(1)
        rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'Mode now: {self.state.mode}')

        # Test arm
        input('\nPress ENTER to ARM (props off!)...')
        self.arm(True)
        time.sleep(1)
        rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'Armed: {self.state.armed}')

        # Disarm
        input('\nPress ENTER to DISARM...')
        self.arm(False)
        time.sleep(1)
        rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'Armed: {self.state.armed}')

        self.get_logger().info('--- TEST COMPLETE ---')

def main():
    rclpy.init()
    node = CommsTest()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
