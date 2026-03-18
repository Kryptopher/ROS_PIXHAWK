#!/usr/bin/env python3

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandLong
from sensor_msgs.msg import NavSatFix
import math, time, yaml, os

# ── Load config ───────────────────────────────────
CONFIG_FILE = os.path.join(os.path.dirname(__file__), 'mission_config.yaml')

def load_config():
    with open(CONFIG_FILE, 'r') as f:
        cfg = yaml.safe_load(f)
    flight = cfg['flight']

    waypoints = []
    for wp in cfg['waypoints']:
        waypoints.append({
            'lat':       float(wp['lat']),
            'lon':       float(wp['lon']),
            'alt':       float(wp['alt']),
            'yaw':       float(wp.get('yaw', 0.0)),
            'speed':     float(wp.get('speed',     flight['speed'])),
            'hold_time': float(wp.get('hold_time', flight['waypoint_hold_time'])),
        })

    return {
        'altitude':    float(flight['takeoff_altitude']),
        'speed':       float(flight['speed']),
        'hold_time':   float(flight['waypoint_hold_time']),
        'threshold':   float(flight['waypoint_threshold']),
        'loiter':      float(flight['loiter_time']),
        'return_home': bool(flight['return_home']),
        'waypoints':   waypoints,
    }

# ── Globals ───────────────────────────────────────
current_state  = None
current_pose   = None
home_position  = None

def state_cb(msg):
    global current_state
    current_state = msg

def gps_cb(msg):
    global current_pose, home_position
    current_pose = msg
    # Capture home position on first GPS fix
    if home_position is None and msg.latitude != 0.0:
        home_position = (msg.latitude, msg.longitude, msg.altitude)
        print(f"Home position set: {home_position[0]:.7f}, {home_position[1]:.7f}")

# ── Helpers ───────────────────────────────────────
def gps_distance(lat1, lon1, lat2, lon2):
    """Approximate distance in metres between two GPS points."""
    R  = 6371000
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    a = (math.sin(d_lat / 2) ** 2 +
         math.cos(math.radians(lat1)) *
         math.cos(math.radians(lat2)) *
         math.sin(d_lon / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def make_geo_setpoint(node, lat, lon, alt, yaw=0.0):
    sp = GeoPoseStamped()
    sp.header.stamp          = node.get_clock().now().to_msg()
    sp.header.frame_id       = "map"
    sp.pose.position.latitude  = lat
    sp.pose.position.longitude = lon
    sp.pose.position.altitude  = alt
    yaw_rad = math.radians(yaw)
    sp.pose.orientation.z    = math.sin(yaw_rad / 2.0)
    sp.pose.orientation.w    = math.cos(yaw_rad / 2.0)
    return sp

def set_speed(node, speed_client, speed):
    """Set speed via MAV_CMD_DO_CHANGE_SPEED."""
    req = CommandLong.Request()
    req.command  = 178   # MAV_CMD_DO_CHANGE_SPEED
    req.param1   = 0.0   # 0 = airspeed, 1 = groundspeed
    req.param2   = float(speed)
    req.param3   = -1.0  # throttle (-1 = no change)
    req.param4   = 0.0
    speed_client.call_async(req)

def fly_to(node, pub, speed_client, wp, label, threshold):
    lat, lon, alt = wp['lat'], wp['lon'], wp['alt']
    yaw       = wp['yaw']
    speed     = wp['speed']
    hold_time = wp['hold_time']

    print(f"\n-> Flying to {label}")
    print(f"   GPS: {lat:.7f}, {lon:.7f}  Alt: {alt} m  Yaw: {yaw}°")
    print(f"   Speed: {speed} m/s  |  Hold: {hold_time} s  |  Threshold: {threshold} m")

    set_speed(node, speed_client, speed)

    arrived     = False
    arrive_time = None

    while rclpy.ok():
        pub.publish(make_geo_setpoint(node, lat, lon, alt, yaw))
        rclpy.spin_once(node, timeout_sec=0.05)

        if current_pose is None:
            continue

        dist = gps_distance(
            current_pose.latitude, current_pose.longitude,
            lat, lon
        )

        if dist < threshold and not arrived:
            arrived     = True
            arrive_time = time.time()
            print(f"✔  {label} reached!  (distance: {dist:.2f} m)")
            print(f"   Holding for {hold_time} seconds...")

        if arrived and (time.time() - arrive_time) >= hold_time:
            print(f"   {label} complete.")
            return

# ── Main ─────────────────────────────────────────
def main():
    cfg = load_config()

    print("=" * 50)
    print("  DRONE MISSION CONFIGURATION")
    print("=" * 50)
    print(f"  Takeoff Altitude   : {cfg['altitude']} m")
    print(f"  Default Speed      : {cfg['speed']} m/s")
    print(f"  Default Hold Time  : {cfg['hold_time']} s")
    print(f"  Arrival Threshold  : {cfg['threshold']} m")
    print(f"  Post-Takeoff Loiter: {cfg['loiter']} s")
    print(f"  Return Home        : {cfg['return_home']}")
    print(f"  Waypoints          : {len(cfg['waypoints'])}")
    for i, wp in enumerate(cfg['waypoints'], 1):
        print(f"    WP{i}: lat={wp['lat']}  lon={wp['lon']}  "
              f"alt={wp['alt']}m  yaw={wp['yaw']}°  "
              f"speed={wp['speed']} m/s  hold={wp['hold_time']} s")
    print("=" * 50)
    input("\nPress ENTER to start mission...")

    rclpy.init()
    node = rclpy.create_node('drone_mission')

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE
    )

    # Subscribers
    node.create_subscription(State,     '/mavros/state',                  state_cb, 10)
    node.create_subscription(NavSatFix, '/mavros/global_position/global', gps_cb,   qos)

    # Publishers
    pub = node.create_publisher(GeoPoseStamped, '/mavros/setpoint_position/global', 10)

    # Services
    arming       = node.create_client(CommandBool, '/mavros/cmd/arming')
    set_mode     = node.create_client(SetMode,     '/mavros/set_mode')
    takeoff      = node.create_client(CommandTOL,  '/mavros/cmd/takeoff')
    land         = node.create_client(CommandTOL,  '/mavros/cmd/land')
    speed_client = node.create_client(CommandLong, '/mavros/cmd/command')

    # Wait for FCU
    print("\nWaiting for FCU connection...")
    while current_state is None or not current_state.connected:
        rclpy.spin_once(node, timeout_sec=0.1)
    print("Connected!")

    # Wait for GPS fix
    print("Waiting for GPS fix...")
    while current_pose is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    print("GPS fix acquired!")

    # GUIDED mode
    req = SetMode.Request()
    req.custom_mode = "GUIDED"
    rclpy.spin_until_future_complete(node, set_mode.call_async(req))
    print("GUIDED mode set")

    # Arm
    req = CommandBool.Request()
    req.value = True
    rclpy.spin_until_future_complete(node, arming.call_async(req))
    print("Armed!")

    # Takeoff
    print(f"\nTaking off to {cfg['altitude']} m...")
    req = CommandTOL.Request()
    req.altitude = cfg['altitude']
    rclpy.spin_until_future_complete(node, takeoff.call_async(req))
    print("Takeoff command sent!")

    while current_pose is None or current_pose.altitude < (home_position[2] + cfg['altitude'] - 1.0):
        rclpy.spin_once(node, timeout_sec=0.1)
    print("Altitude reached!")

    # Post-takeoff loiter
    if cfg['loiter'] > 0:
        print(f"Loitering for {cfg['loiter']} seconds to stabilize...")
        time.sleep(cfg['loiter'])
        print("Loiter complete.")

    # Waypoints
    for i, wp in enumerate(cfg['waypoints'], 1):
        fly_to(node, pub, speed_client, wp,
               f"Waypoint {i}/{len(cfg['waypoints'])}", cfg['threshold'])

    print("\nAll waypoints complete!")

    # Return home if enabled
    if cfg['return_home'] and home_position is not None:
        print("Returning home...")
        home_wp = {
            'lat':       home_position[0],
            'lon':       home_position[1],
            'alt':       cfg['altitude'],
            'yaw':       0.0,
            'speed':     cfg['speed'],
            'hold_time': 2.0
        }
        fly_to(node, pub, speed_client, home_wp, "HOME", cfg['threshold'])

    # Land
    print("Landing...")
    req = CommandTOL.Request()
    rclpy.spin_until_future_complete(node, land.call_async(req))

    while current_pose is None or current_pose.altitude > (home_position[2] + 0.5):
        rclpy.spin_once(node, timeout_sec=0.1)

    # Disarm
    req = CommandBool.Request()
    req.value = False
    rclpy.spin_until_future_complete(node, arming.call_async(req))

    print("\n" + "=" * 50)
    print("  MISSION COMPLETE ✅")
    print("=" * 50)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
