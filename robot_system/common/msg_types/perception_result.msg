# Perception result message definition
# Represents the output of perception processing including detected objects and classifications

# Standard header
std_msgs/Header header

# Timestamp of perception result
builtin_interfaces/Time timestamp

# List of detected objects with bounding boxes
DetectedObject[] detected_objects

# List of class labels for detected objects
string[] object_classes

# List of confidence scores for detections
float64[] confidence_scores

# Semantic segmentation data (optional)
sensor_msgs/Image semantic_segmentation

# Confidence map (optional)
sensor_msgs/Image confidence_map

# Define the DetectedObject message
================================================================================
MSG: robot_system/DetectedObject
string class_name
float64 confidence
geometry_msgs/Pose pose
# Bounding box in image coordinates
uint32 x
uint32 y
uint32 width
uint32 height