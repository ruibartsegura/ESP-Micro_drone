# Runtime parameters of the drone (topic drone/params).
# A field that is not filled keeps its default value -1, and the drone does
# not change it (any negative value = do not change). If one value is out
# of range, the whole message is rejected. They are applied in CHECKING.

# Flight
float32 v_max            -1.0   # Max velocity (m/s)

# Controller (attitude_controller.c)
float32 kp_h             -1.0   # m error -> motor speed (rad/s)
float32 kd_h             -1.0   # m/s vertical velocity -> motor speed (rad/s)
float32 kp_vel           -1.0   # m/s error -> target roll / pitch (rad)
float32 kp_angle         -1.0   # deg error -> deg/s target
float32 kp_rate          -1.0   # deg/s error -> motor speed (roll / pitch)
float32 ki_rate          -1.0   # integral of the deg/s error -> motor speed
float32 kp_yaw_rate      -1.0   # deg/s error -> motor speed (yaw)
float32 max_rate         -1.0   # Max target roll/pitch rate (deg/s)
float32 max_pow_rp       -1.0   # Max speed correction of roll / pitch (rad/s)
float32 base             -1.0   # Hover motor speed (rad/s)
float32 alpha            -1.0   # Complementary filter (0-1)
float32 max_angle        -1.0   # Max target tilt (rad)
float32 max_pow_yaw      -1.0   # Max speed correction of yaw (rad/s)
float32 pitch_deviation  -1.0   # Start value of the pitch integral (rad/s)
float32 roll_deviation   -1.0   # Start value of the roll integral (rad/s)

# Odometry (odometry_estimator.c)
float32 k_drag           -1.0   # Rotor drag (1/s)
