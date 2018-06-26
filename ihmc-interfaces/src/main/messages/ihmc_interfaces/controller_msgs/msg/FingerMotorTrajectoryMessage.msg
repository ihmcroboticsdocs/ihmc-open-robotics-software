# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Delay time for each finger motor trajectory
float64 delay_time 0.0

# Desired motor position
TrajectoryPoint1DMessage desired_position