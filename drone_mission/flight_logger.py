import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, PositionTarget
import csv, time, os, sys
from datetime import datetime
from rclpy.executors import ExternalShutdownException

class FlightLogger(Node):
    def __init__(self, log_file):
        super().__init__('flight_logger')
        self.log_file   = log_file
        self.start_time = time.time()
        self.rows       = []

        self.actual_pos = None
        self.actual_vel = None
        self.cmd_raw    = None
        self.state      = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)

        self.create_subscription(Odometry, '/mavros/local_position/odom',
                                 self.odom_cb, sensor_qos)
        self.create_subscription(State, '/mavros/state',
                                 self.state_cb, 10)
        self.create_subscription(PositionTarget, '/mavros/setpoint_raw/local',
                                 self.cmd_raw_cb, 10)

        self.create_timer(0.05, self.log_cb)
        self.get_logger().info(f'Logging to {log_file}')

    def odom_cb(self, msg):
        self.actual_pos = msg.pose.pose.position
        self.actual_vel = msg.twist.twist.linear

    def state_cb(self, msg):
        self.state = msg

    def cmd_raw_cb(self, msg):
        self.cmd_raw = msg

    def log_cb(self):
        t = time.time() - self.start_time
        row = {
            't': round(t, 3),
            # actual position
            'act_x':  round(self.actual_pos.x, 4) if self.actual_pos else 0.0,
            'act_y':  round(self.actual_pos.y, 4) if self.actual_pos else 0.0,
            'act_z':  round(self.actual_pos.z, 4) if self.actual_pos else 0.0,
            # actual velocity
            'act_vx': round(self.actual_vel.x, 4) if self.actual_vel else 0.0,
            'act_vy': round(self.actual_vel.y, 4) if self.actual_vel else 0.0,
            'act_vz': round(self.actual_vel.z, 4) if self.actual_vel else 0.0,
            # commanded position (from setpoint_raw)
            'cmd_px': round(self.cmd_raw.position.x, 4) if self.cmd_raw else 0.0,
            'cmd_py': round(self.cmd_raw.position.y, 4) if self.cmd_raw else 0.0,
            'cmd_pz': round(self.cmd_raw.position.z, 4) if self.cmd_raw else 0.0,
            # commanded velocity (from setpoint_raw)
            'cmd_vx': round(self.cmd_raw.velocity.x, 4) if self.cmd_raw else 0.0,
            'cmd_vy': round(self.cmd_raw.velocity.y, 4) if self.cmd_raw else 0.0,
            'cmd_vz': round(self.cmd_raw.velocity.z, 4) if self.cmd_raw else 0.0,
            # mode
            'mode':  self.state.mode  if self.state else '',
            'armed': self.state.armed if self.state else False,
        }
        self.rows.append(row)

    def save(self):
        if not self.rows:
            self.get_logger().warn('No data to save')
            return
        with open(self.log_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.rows[0].keys())
            writer.writeheader()
            writer.writerows(self.rows)
        self.get_logger().info(f'Saved {len(self.rows)} rows to {self.log_file}')

def main():
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_file  = sys.argv[1] if len(sys.argv) > 1 \
                else f'/home/pi/logs/flight_{timestamp}.csv'
    os.makedirs(os.path.dirname(log_file), exist_ok=True)

    rclpy.init()
    node = FlightLogger(log_file)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        print(f'Logger error: {e}')
    finally:
        try:
            node.save()
        except Exception as e:
            print(f'Save error: {e}')
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
