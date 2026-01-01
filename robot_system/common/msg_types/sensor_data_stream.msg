# Sensor data stream message definition
# Represents real-time data from various sensors (LiDAR, cameras, IMU, etc.)

# Standard header
std_msgs/Header header

# Timestamp of the sensor reading
builtin_interfaces/Time timestamp

# Enum for sensor type (LIDAR, CAMERA_RGB, CAMERA_DEPTH, IMU, JOINT_ENCODER)
string sensor_type

# Unique identifier for the specific sensor
string sensor_id

# Sensor-specific data payload (using generic message type)
sensor_msgs/PointCloud2 point_cloud  # For LiDAR data
sensor_msgs/Image image              # For camera data
sensor_msgs/Imu imu_data            # For IMU data
sensor_msgs/JointState joint_data   # For joint encoder data

# Coordinate frame in which the data is expressed
string frame_id

# Float percentage (0.0-1.0) representing data quality
float64 quality

# Additional metadata
string metadata