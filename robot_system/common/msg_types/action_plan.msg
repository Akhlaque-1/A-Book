# Action plan message definition
# Represents a sequence of low-level motor commands to achieve a high-level goal

# Standard header
std_msgs/Header header

# Unique identifier for the action plan
string plan_id

# Natural language description of the goal
string goal_description

# List of action steps with parameters
ActionStep[] action_sequence

# Estimated time to complete the plan (in seconds)
float64 estimated_duration

# List of safety constraints for plan execution
string[] safety_constraints

# Status enum (PENDING, EXECUTING, COMPLETED, FAILED, CANCELLED)
string status

# Timestamp when plan was created
builtin_interfaces/Time timestamp

# Define the ActionStep message for action sequence
================================================================================
MSG: robot_system/ActionStep
string action_type
float64[] parameters
geometry_msgs/Pose target_pose
int32 execution_priority