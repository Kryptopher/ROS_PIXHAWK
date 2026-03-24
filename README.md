# ROS2 Drone Mission System

Velocity control + input shaping for payload swing suppression.

## Files
- `mission_executor.py` — mission runner, velocity/position control, ZV/ZVD shaping
- `flight_logger.py` — logs actual vs commanded position and velocity
- `plot_flight.py` — generates plots from flight logs
- `start_mission.sh` — launches MAVROS + logger + mission
- `mission.csv` — mission file
- `angle.py` — encoder payload swing angle reader

## Usage
```bash
~/start_mission.sh ~/mission.csv none 1.0    # no shaping
~/start_mission.sh ~/mission.csv ZV 1.0     # ZV shaping
~/start_mission.sh ~/mission.csv ZVD 1.0    # ZVD shaping
```
