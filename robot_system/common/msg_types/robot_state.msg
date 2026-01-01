# Base robot state representation
# Message definition for robot state including position, orientation, joint angles, and operational status

# Standard header
std_msgs/Header header

# Position and orientation in world coordinates
geometry_msgs/Pose pose

# Joint states (positions, velocities, efforts)
sensor_msgs/JointState joint_states

# Battery level as percentage (0.0 to 1.0)
float64 battery_level

# Operational status (IDLE, MOVING, PROCESSING, ERROR)
string operational_status

# Safety status (SAFE, WARNING, DANGER)
string safety_status

# Timestamp of the state measurement
builtin_interfaces/Time timestamp