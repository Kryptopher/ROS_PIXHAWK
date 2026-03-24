#!/bin/bash

MISSION_FILE="${1:-/home/pi/mission.csv}"
SHAPER="${2:-none}"
ROPE_LENGTH="${3:-1.0}"
FCU_URL="udp://0.0.0.0:14551@192.168.0.100:14550"
LOG_DIR="/home/pi/logs"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_FILE="$LOG_DIR/flight_${SHAPER}_${TIMESTAMP}.csv"

source /opt/ros/humble/setup.bash
mkdir -p $LOG_DIR

echo "=========================================="
echo " Drone Mission Launcher"
echo " Shaper:   $SHAPER"
echo " Rope:     ${ROPE_LENGTH}m"
echo "=========================================="

# ── Step 1: choose mode ───────────────────────────────────────
echo ""
echo "Control mode:"
echo "  [1] Position  — give waypoints (x, y, z)"
echo "  [2] Velocity  — give velocity commands (vx, vy, vz)"
echo "  [3] Mixed     — mix position and velocity"
echo "  [E] Edit CSV manually"
echo ""
read -p "Choose [1/2/3/E]: " -n1 mode_choice
echo ""

if [[ "$mode_choice" == "e" || "$mode_choice" == "E" ]]; then
    nano $MISSION_FILE

elif [[ "$mode_choice" == "1" ]]; then
    # ── Position mode wizard ──────────────────────────────────
    echo ""
    read -p "Takeoff altitude (m) [default 5]: " alt
    alt=${alt:-5}

    echo ""
    echo "Enter waypoints as: t x y z"
    echo "Example:  5 3 0 5  (at t=5s go to x=3 y=0 z=5)"
    echo "Type 'done' when finished."
    echo ""

    tmpfile=$(mktemp)
    echo "t,type,mode,profile,x,y,z,vx,vy,vz,ax" > $tmpfile
    echo "0,takeoff,pos,hold,0,0,$alt,0,0,0,0"    >> $tmpfile

    while true; do
        read -p "waypoint> " input
        [[ "$input" == "done" ]] && break
        read -r t x y z <<< "$input"
        echo "$t,cmd,pos,hold,$x,$y,$z,0,0,0,0" >> $tmpfile
    done

    # add end row using last t + 5
    last_t=$(tail -1 $tmpfile | cut -d',' -f1)
    end_t=$((last_t + 5))
    echo "$end_t,end,end,end,0,0,0,0,0,0,0" >> $tmpfile
    cp $tmpfile $MISSION_FILE

elif [[ "$mode_choice" == "2" ]]; then
    # ── Velocity mode wizard ──────────────────────────────────
    echo ""
    read -p "Takeoff altitude (m) [default 5]: " alt
    alt=${alt:-5}

    read -p "Velocity profile [step/trap/sine, default step]: " prof
    prof=${prof:-step}

    if [[ "$prof" == "trap" ]]; then
        read -p "Acceleration (m/s²) [default 0.3]: " acc
        acc=${acc:-0.3}
    else
        acc=0.3
    fi

    echo ""
    echo "Enter commands as: t vx vy vz"
    echo "Example:  5 0.5 0 0  (at t=5s move at vx=0.5 m/s)"
    echo "Use 0 0 0 to stop.  Type 'done' when finished."
    echo ""

    tmpfile=$(mktemp)
    echo "t,type,mode,profile,x,y,z,vx,vy,vz,ax" > $tmpfile
    echo "0,takeoff,pos,hold,0,0,$alt,0,0,0,0"    >> $tmpfile

    while true; do
        read -p "velocity> " input
        [[ "$input" == "done" ]] && break
        read -r t vx vy vz <<< "$input"
        echo "$t,cmd,vel,$prof,0,0,0,$vx,$vy,$vz,$acc" >> $tmpfile
    done

    last_t=$(tail -1 $tmpfile | cut -d',' -f1)
    end_t=$((last_t + 5))
    echo "$end_t,end,end,end,0,0,0,0,0,0,0" >> $tmpfile
    cp $tmpfile $MISSION_FILE

elif [[ "$mode_choice" == "3" ]]; then
    # ── Mixed mode wizard ─────────────────────────────────────
    echo ""
    read -p "Takeoff altitude (m) [default 5]: " alt
    alt=${alt:-5}

    read -p "Velocity profile [step/trap/sine, default step]: " prof
    prof=${prof:-step}

    if [[ "$prof" == "trap" ]]; then
        read -p "Acceleration (m/s²) [default 0.3]: " acc
        acc=${acc:-0.3}
    else
        acc=0.3
    fi

    echo ""
    echo "Enter commands as: t mode values"
    echo "  Position:  t p x y z          → e.g.  5 p 3 0 5"
    echo "  Velocity:  t v vx vy vz        → e.g.  10 v 0.5 0 0"
    echo "Type 'done' when finished."
    echo ""

    tmpfile=$(mktemp)
    echo "t,type,mode,profile,x,y,z,vx,vy,vz,ax" > $tmpfile
    echo "0,takeoff,pos,hold,0,0,$alt,0,0,0,0"    >> $tmpfile

    while true; do
        read -p "cmd> " input
        [[ "$input" == "done" ]] && break
        read -r t m rest <<< "$input"
        if [[ "$m" == "p" ]]; then
            read -r x y z <<< "$rest"
            echo "$t,cmd,pos,hold,$x,$y,$z,0,0,0,0" >> $tmpfile
        elif [[ "$m" == "v" ]]; then
            read -r vx vy vz <<< "$rest"
            echo "$t,cmd,vel,$prof,0,0,0,$vx,$vy,$vz,$acc" >> $tmpfile
        else
            echo "Unknown mode '$m' — use p or v"
        fi
    done

    last_t=$(tail -1 $tmpfile | cut -d',' -f1)
    end_t=$((last_t + 5))
    echo "$end_t,end,end,end,0,0,0,0,0,0,0" >> $tmpfile
    cp $tmpfile $MISSION_FILE
fi

# ── Show final CSV ────────────────────────────────────────────
echo ""
echo "--- Mission preview ---"
cat $MISSION_FILE
echo ""
echo "Press [ENTER] to launch, or Ctrl+C to abort..."
read

# ── Kill old processes ────────────────────────────────────────
echo "[1/4] Cleaning up..."
pkill -f mavros_node   2>/dev/null
pkill -f flight_logger 2>/dev/null
sleep 1

# ── Start MAVROS ──────────────────────────────────────────────
echo "[2/4] Starting MAVROS..."
ros2 run mavros mavros_node --ros-args \
    -p fcu_url:=$FCU_URL \
    -p target_system_id:=1 \
    -p target_component_id:=1 \
    -p system_id:=255 \
    --log-level mavros:=WARN &
MAVROS_PID=$!

echo "      Waiting for FCU connection..."
TIMEOUT=30
ELAPSED=0
while [ $ELAPSED -lt $TIMEOUT ]; do
    CONNECTED=$(python3 -c "
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
import sys, time
rclpy.init()
n = Node('check')
result = []
def cb(m): result.append(m.connected)
n.create_subscription(State, '/mavros/state', cb, 10)
start = time.time()
while time.time()-start < 3.0:
    rclpy.spin_once(n, timeout_sec=0.1)
    if result: break
print('yes' if result and result[0] else 'no')
n.destroy_node()
rclpy.shutdown()
" 2>/dev/null)
    if [ "$CONNECTED" == "yes" ]; then
        echo "      Connected! (${ELAPSED}s)"
        break
    fi
    sleep 3
    ELAPSED=$((ELAPSED + 3))
done

if [ "$CONNECTED" != "yes" ]; then
    echo "[ERROR] MAVROS not connected after ${TIMEOUT}s"
    kill $MAVROS_PID 2>/dev/null
    exit 1
fi

# ── Start logger ──────────────────────────────────────────────
echo "[3/4] Starting flight logger..."
python3 /home/pi/flight_logger.py $LOG_FILE &
LOGGER_PID=$!
sleep 1

# ── Run mission ───────────────────────────────────────────────
echo "[4/4] Running mission..."
echo ""
python3 /home/pi/mission_executor.py $MISSION_FILE $SHAPER $ROPE_LENGTH
MISSION_EXIT=$?

# ── Cleanup ───────────────────────────────────────────────────
echo ""
echo "=========================================="
[ $MISSION_EXIT -eq 0 ] && echo " Mission complete!" \
                         || echo " Mission exited with code $MISSION_EXIT"

kill $LOGGER_PID 2>/dev/null
wait $LOGGER_PID 2>/dev/null
echo " Log saved: $LOG_FILE"

kill $MAVROS_PID 2>/dev/null
wait $MAVROS_PID 2>/dev/null
echo " MAVROS stopped."

# ── Plot ──────────────────────────────────────────────────────
echo ""
echo "Press [P] to generate plots, or [ENTER] to skip..."
read -n1 -r key
echo ""
if [[ "$key" == "p" || "$key" == "P" ]]; then
    PLOT_DIR="$LOG_DIR/plots_${SHAPER}_${TIMESTAMP}"
    python3 /home/pi/plot_flight.py $LOG_FILE $PLOT_DIR
    echo " Plots saved: $PLOT_DIR"
    echo " scp pi@192.168.0.152:$PLOT_DIR/*.png ."
fi

echo "=========================================="
