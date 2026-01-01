"""
Logging and Error Handling Infrastructure

This module provides comprehensive logging and error handling for the humanoid robot system.
"""

import logging
import sys
import os
from datetime import datetime
import traceback
import json
from enum import Enum


class LogLevel(Enum):
    """Enumeration for log levels"""
    DEBUG = 1
    INFO = 2
    WARNING = 3
    ERROR = 4
    CRITICAL = 5


class RobotSystemLogger:
    """
    Comprehensive logging system for the humanoid robot system.
    Provides structured logging with different levels and error handling.
    """
    
    def __init__(self, name="RobotSystem", log_file=None, console_output=True):
        self.name = name
        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging.DEBUG)
        
        # Clear any existing handlers
        self.logger.handlers.clear()
        
        # Create formatter
        self.formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        
        # Add console handler if requested
        if console_output:
            console_handler = logging.StreamHandler(sys.stdout)
            console_handler.setLevel(logging.DEBUG)
            console_handler.setFormatter(self.formatter)
            self.logger.addHandler(console_handler)
        
        # Add file handler if specified
        if log_file:
            file_handler = logging.FileHandler(log_file)
            file_handler.setLevel(logging.DEBUG)
            file_handler.setFormatter(self.formatter)
            self.logger.addHandler(file_handler)
        
        # Store log file path
        self.log_file = log_file
        
        # Error tracking
        self.error_count = 0
        self.warning_count = 0
        self.error_log = []
    
    def debug(self, message, extra_data=None):
        """Log a debug message"""
        self._log_message(LogLevel.DEBUG, message, extra_data)
    
    def info(self, message, extra_data=None):
        """Log an info message"""
        self._log_message(LogLevel.INFO, message, extra_data)
    
    def warning(self, message, extra_data=None):
        """Log a warning message"""
        self.warning_count += 1
        self._log_message(LogLevel.WARNING, message, extra_data)
    
    def error(self, message, extra_data=None):
        """Log an error message"""
        self.error_count += 1
        self._log_message(LogLevel.ERROR, message, extra_data)
    
    def critical(self, message, extra_data=None):
        """Log a critical message"""
        self.error_count += 1
        self._log_message(LogLevel.CRITICAL, message, extra_data)
    
    def _log_message(self, level, message, extra_data=None):
        """Internal method to log a message with the specified level"""
        # Format the message with extra data if provided
        if extra_data:
            message = f"{message} | Extra data: {extra_data}"
        
        # Log the message using the underlying logger
        if level == LogLevel.DEBUG:
            self.logger.debug(message)
        elif level == LogLevel.INFO:
            self.logger.info(message)
        elif level == LogLevel.WARNING:
            self.logger.warning(message)
        elif level == LogLevel.ERROR:
            self.logger.error(message)
        elif level == LogLevel.CRITICAL:
            self.logger.critical(message)
    
    def log_exception(self, e, context=""):
        """Log an exception with traceback"""
        tb_str = traceback.format_exc()
        error_msg = f"Exception occurred{f' in {context}' if context else ''}: {str(e)}"
        self.error(f"{error_msg}\nTraceback:\n{tb_str}")
        
        # Store error for tracking
        self.error_log.append({
            'timestamp': datetime.now().isoformat(),
            'error': str(e),
            'context': context,
            'traceback': tb_str
        })
    
    def get_stats(self):
        """Get logging statistics"""
        return {
            'error_count': self.error_count,
            'warning_count': self.warning_count,
            'log_file': self.log_file
        }
    
    def save_error_log(self, filepath):
        """Save the error log to a file"""
        with open(filepath, 'w') as f:
            json.dump(self.error_log, f, indent=2)


class RobotSystemError(Exception):
    """Base exception class for the robot system"""
    pass


class SafetyError(RobotSystemError):
    """Exception raised for safety-related errors"""
    pass


class CommunicationError(RobotSystemError):
    """Exception raised for communication-related errors"""
    pass


class HardwareError(RobotSystemError):
    """Exception raised for hardware-related errors"""
    pass


class NavigationError(RobotSystemError):
    """Exception raised for navigation-related errors"""
    pass


class PerceptionError(RobotSystemError):
    """Exception raised for perception-related errors"""
    pass


class ValidationError(RobotSystemError):
    """Exception raised for validation-related errors"""
    pass


class ErrorHandler:
    """
    Error handling system for the humanoid robot system.
    Provides structured error handling and recovery mechanisms.
    """
    
    def __init__(self, logger):
        self.logger = logger
        self.error_handlers = {}
        self.recovery_strategies = {}
    
    def register_error_handler(self, error_type, handler_func):
        """Register a custom error handler for a specific error type"""
        self.error_handlers[error_type] = handler_func
    
    def register_recovery_strategy(self, error_type, strategy_func):
        """Register a recovery strategy for a specific error type"""
        self.recovery_strategies[error_type] = strategy_func
    
    def handle_error(self, error, context=""):
        """Handle an error with appropriate logging and response"""
        error_type = type(error)
        
        # Log the error
        self.logger.log_exception(error, context)
        
        # Check if there's a custom handler for this error type
        if error_type in self.error_handlers:
            try:
                return self.error_handlers[error_type](error, context)
            except Exception as handler_error:
                self.logger.error(f"Error in custom error handler: {handler_error}")
        
        # Default error handling based on error type
        if isinstance(error, SafetyError):
            return self._handle_safety_error(error, context)
        elif isinstance(error, CommunicationError):
            return self._handle_communication_error(error, context)
        elif isinstance(error, HardwareError):
            return self._handle_hardware_error(error, context)
        elif isinstance(error, NavigationError):
            return self._handle_navigation_error(error, context)
        elif isinstance(error, PerceptionError):
            return self._handle_perception_error(error, context)
        elif isinstance(error, ValidationError):
            return self._handle_validation_error(error, context)
        else:
            return self._handle_general_error(error, context)
    
    def attempt_recovery(self, error, context=""):
        """Attempt to recover from an error using registered strategies"""
        error_type = type(error)
        
        if error_type in self.recovery_strategies:
            try:
                self.logger.info(f"Attempting recovery for {error_type.__name__}")
                return self.recovery_strategies[error_type](error, context)
            except Exception as recovery_error:
                self.logger.error(f"Recovery failed: {recovery_error}")
                return False
        
        self.logger.warning(f"No recovery strategy registered for {error_type.__name__}")
        return False
    
    def _handle_safety_error(self, error, context):
        """Handle safety-related errors"""
        self.logger.critical(f"Safety error in {context}: {error}")
        # In a real system, this might trigger emergency stop procedures
        return {"status": "safety_error", "action": "emergency_stop"}
    
    def _handle_communication_error(self, error, context):
        """Handle communication-related errors"""
        self.logger.error(f"Communication error in {context}: {error}")
        # In a real system, this might trigger reconnection attempts
        return {"status": "communication_error", "action": "reconnect"}
    
    def _handle_hardware_error(self, error, context):
        """Handle hardware-related errors"""
        self.logger.error(f"Hardware error in {context}: {error}")
        # In a real system, this might trigger hardware diagnostics
        return {"status": "hardware_error", "action": "diagnostics"}
    
    def _handle_navigation_error(self, error, context):
        """Handle navigation-related errors"""
        self.logger.error(f"Navigation error in {context}: {error}")
        # In a real system, this might trigger alternative navigation
        return {"status": "navigation_error", "action": "alternative_route"}
    
    def _handle_perception_error(self, error, context):
        """Handle perception-related errors"""
        self.logger.error(f"Perception error in {context}: {error}")
        # In a real system, this might trigger sensor recalibration
        return {"status": "perception_error", "action": "recalibrate"}
    
    def _handle_validation_error(self, error, context):
        """Handle validation-related errors"""
        self.logger.warning(f"Validation error in {context}: {error}")
        return {"status": "validation_error", "action": "reject_input"}
    
    def _handle_general_error(self, error, context):
        """Handle general errors"""
        self.logger.error(f"General error in {context}: {error}")
        return {"status": "general_error", "action": "log_and_continue"}
    
    def safe_execute(self, func, *args, context="", default_return=None, **kwargs):
        """Safely execute a function with error handling"""
        try:
            return func(*args, **kwargs)
        except Exception as e:
            self.handle_error(e, context)
            return default_return


# Global logger instance
_robot_logger = None
_error_handler = None


def get_logger():
    """Get the global robot system logger"""
    global _robot_logger
    if _robot_logger is None:
        # Create a default logger
        log_dir = "logs"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_file = os.path.join(log_dir, f"robot_system_{timestamp}.log")
        
        _robot_logger = RobotSystemLogger(log_file=log_file)
    return _robot_logger


def get_error_handler():
    """Get the global error handler"""
    global _error_handler
    if _error_handler is None:
        _error_handler = ErrorHandler(get_logger())
    return _error_handler


def setup_logging_and_error_handling():
    """Setup the logging and error handling infrastructure"""
    logger = get_logger()
    error_handler = get_error_handler()
    
    # Register default recovery strategies
    def default_recovery(error, context):
        logger.info(f"Attempting default recovery for {type(error).__name__}")
        return True  # Assume recovery successful
    
    error_handler.register_recovery_strategy(RobotSystemError, default_recovery)
    
    logger.info("Logging and error handling infrastructure initialized")
    
    return logger, error_handler


# Context manager for safe execution with error handling
class SafeExecution:
    """Context manager for safe execution with automatic error handling"""
    
    def __init__(self, context="", reraise=True):
        self.context = context
        self.reraise = reraise
        self.error_handler = get_error_handler()
        self.logger = get_logger()
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        if exc_type is not None:
            self.error_handler.handle_error(exc_value, self.context)
            
            # Attempt recovery
            recovery_success = self.error_handler.attempt_recovery(
                exc_value, self.context
            )
            
            if self.reraise:
                return False  # Re-raise the exception
            else:
                return True  # Suppress the exception


# Example usage
if __name__ == "__main__":
    # Setup the logging and error handling infrastructure
    logger, error_handler = setup_logging_and_error_handling()
    
    # Example of logging
    logger.info("Starting robot system")
    logger.debug("Debug information")
    logger.warning("This is a warning")
    
    # Example of error handling
    try:
        # Simulate an error
        raise HardwareError("Motor controller not responding")
    except Exception as e:
        error_handler.handle_error(e, "Motor control system")
    
    # Example of safe execution
    def risky_function():
        # This function might fail
        raise CommunicationError("Cannot connect to sensor")
    
    result = error_handler.safe_execute(
        risky_function, 
        context="Sensor connection",
        default_return="default_value"
    )
    
    # Example of using the context manager
    with SafeExecution(context="Critical operation", reraise=False):
        raise SafetyError("Potential collision detected")
    
    # Print statistics
    print("Logging statistics:", logger.get_stats())