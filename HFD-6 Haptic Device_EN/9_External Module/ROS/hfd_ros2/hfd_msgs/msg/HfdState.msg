std_msgs/Header header

geometry_msgs/Pose pose  # Unit: m & rad
geometry_msgs/Twist velocity  # Unit: m/s & rad/s
geometry_msgs/Wrench wrench  # Unit: N & N*m

float64 gripper_angle  # Unit: rad
float64 gripper_gap  # Unit: m
geometry_msgs/Point gripper_thumb_pos  # Unit: m
geometry_msgs/Point gripper_finger_pos  # Unit: m
float64 gripper_linear_velocity  # Unit: m/s
float64 gripper_angular_velocity  # Unit: rad/s
float64 gripper_force  # Unit: N

bool button_status  # True: pressed, False: released
