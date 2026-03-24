import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from mavros_msgs.msg import State
from sensor_msgs.msg import Imu, NavSatFix, FluidPressure
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import time

class SensorTest(Node):
    def __init__(self):
        super().__init__('sensor_test')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)

        self.state    = None
        self.imu      = None
        self.gps      = None
        self.odom     = None
        self.baro     = None

        self.create_subscription(State,        '/mavros/state',              self.state_cb,  10)
        self.create_subscription(Imu,          '/mavros/imu/data',           self.imu_cb,    sensor_qos)
        self.create_subscription(NavSatFix,    '/mavros/global_position/global', self.gps_cb, sensor_qos)
        self.create_subscription(Odometry,     '/mavros/local_position/odom',self.odom_cb,   sensor_qos)
        self.create_subscription(FluidPressure,'/mavros/imu/static_pressure',self.baro_cb,   sensor_qos)

    def state_cb(self, msg): self.state = msg
    def imu_cb(self,   msg): self.imu   = msg
    def gps_cb(self,   msg): self.gps   = msg
    def odom_cb(self,  msg): self.odom  = msg
    def baro_cb(self,  msg): self.baro  = msg

    def run(self):
        self.get_logger().info('Waiting for FCU...')
        start = time.time()
        while self.state is None and time.time() - start < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.state is None:
            self.get_logger().error('No connection')
            return

        self.get_logger().info(f'Connected! Mode={self.state.mode} Armed={self.state.armed}')
        self.get_logger().info('Collecting sensor data for 3 seconds...')

        start = time.time()
        while time.time() - start < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('\n--- SENSOR REPORT ---')

        # IMU
        if self.imu:
            a = self.imu.linear_acceleration
            g = self.imu.angular_velocity
            self.get_logger().info(
                f'IMU ✅  accel=({a.x:.2f},{a.y:.2f},{a.z:.2f}) '
                f'gyro=({g.x:.2f},{g.y:.2f},{g.z:.2f})')
        else:
            self.get_logger().warn('IMU ❌  no data')

        # GPS
        if self.gps:
            self.get_logger().info(
                f'GPS ✅  lat={self.gps.latitude:.6f} '
                f'lon={self.gps.longitude:.6f} '
                f'alt={self.gps.altitude:.1f}m '
                f'status={self.gps.status.status}')
        else:
            self.get_logger().warn('GPS ❌  no data')

        # Barometer
        if self.baro:
            self.get_logger().info(
                f'BARO ✅  pressure={self.baro.fluid_pressure:.1f} Pa')
        else:
            self.get_logger().warn('BARO ❌  no data')

        # Local position
        if self.odom:
            p = self.odom.pose.pose.position
            v = self.odom.twist.twist.linear
            self.get_logger().info(
                f'ODOM ✅  pos=({p.x:.2f},{p.y:.2f},{p.z:.2f}) '
                f'vel=({v.x:.2f},{v.y:.2f},{v.z:.2f})')
        else:
            self.get_logger().warn('ODOM ❌  no data')

        self.get_logger().info('--- END REPORT ---')

        # Live stream for 10 seconds
        self.get_logger().info('\nLive sensor stream (10s)...')
        start = time.time()
        last_print = 0
        while time.time() - start < 10.0:
            rclpy.spin_once(self, timeout_sec=0.0)
            now = time.time()
            if now - last_print >= 1.0:
                if self.odom and self.imu:
                    p = self.odom.pose.pose.position
                    a = self.imu.linear_acceleration
                    self.get_logger().info(
                        f't={now-start:.0f}s  '
                        f'pos=({p.x:.2f},{p.y:.2f},{p.z:.2f})  '
                        f'accel=({a.x:.2f},{a.y:.2f},{a.z:.2f})')
                last_print = now

        self.get_logger().info('Done.')

def main():
    rclpy.init()
    node = SensorTest()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
