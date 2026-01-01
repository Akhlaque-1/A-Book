# User command message definition
# Represents a high-level instruction from human operator in natural language or text

# Standard header
std_msgs/Header header

# Unique identifier for the command
string command_id

# Original natural language command
string command_text

# Processed and normalized command text
string processed_text

# Classified intent (NAVIGATE, PERFORM_ACTION, REPORT_STATUS, etc.)
string intent

# Map of extracted parameters for the command
KeyValue[] parameters

# Float percentage (0.0-1.0) representing NLP confidence
float64 confidence

# Status enum (RECEIVED, PROCESSED, EXECUTING, COMPLETED, REJECTED)
string status

# Timestamp when command was received
builtin_interfaces/Time timestamp

# Define the KeyValue message for parameters
================================================================================
MSG: std_msgs/KeyValue
string key
string value