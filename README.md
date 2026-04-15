* `mission_executor.py` — mission runner, velocity/position control, ZV/ZVD shaping
* `flight_logger.py` — logs actual vs commanded position and velocity
* `angle_logger.py` — 200Hz encoder payload swing angle logger (synced to mission start)
* `angle.py` — encoder payload swing angle reader
* `plot_flight.py` — generates plots from flight logs
* `start_mission.sh` — launches MAVROS + logger + mission
* `mission.csv` — mission file
* `mission_sysid.csv` — system identification mission file
* `test_comms.py` — communication tests
* `test_motors.py` — motor tests
* `test_sensors.py` — sensor tests
