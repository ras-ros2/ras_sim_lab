#!/usr/bin/env python3

"""
Calibration Target Subscriber
This script subscribes to the calibration target data published via MQTT
and logs it to a file for later use.
Implements the same MQTT pattern as the test_mqtt scripts.
"""

import rclpy
from rclpy.node import Node
import json
import datetime
import os
import yaml
from pathlib import Path
from ras_logging.ras_logger import RasLogger
from ras_transport.interfaces.TransportWrapper import TransportMQTTSubscriber
from ras_common.globals import RAS_CONFIGS_PATH
import argparse

# Define the calibration data file path
CALIBRATION_FILE = os.path.join(RAS_CONFIGS_PATH, "calibration_data.json")

class CalibrationTargetSubscriber(Node):
    """ROS2 node for subscribing to calibration target data via MQTT."""
    
    def __init__(self, start_calibration=False, stop_calibration=False, keep_running=True):
        super().__init__('calibration_target_subscriber')
        
        # Initialize logger
        self.logger = RasLogger()
        self.logger.log_info("Calibration Target Subscriber node started")
        
        # Define the topic
        self.topic = "robot/calibration_target"
        
        # Initialize MQTT subscriber using TransportMQTTSubscriber
        self.logger.log_info(f"Initializing MQTT subscriber for topic: {self.topic}")
        self.mqtt_sub = TransportMQTTSubscriber(self.topic, self.calibration_target_callback)
        
        # Connect to MQTT broker
        self.connect_to_mqtt()
        
        # Store received messages
        self.received_messages = []
        
        # Create a timer to process MQTT messages and check the queue
        self.timer = self.create_timer(0.01, self.process_mqtt)
        
        # Print status periodically
        self.status_timer = self.create_timer(5.0, self.print_status)
        self.message_count = 0
        
        # Store the latest calibration target data
        self.latest_calibration_target = None
        
        # Flag to track if we've already saved calibration data
        self.calibration_data_saved = False
        
        # Handle calibration commands if provided
        if start_calibration:
            self.send_calibration_command("start")
            if not keep_running:
                self.get_logger().info("Sent start command, exiting...")
                rclpy.shutdown()
        elif stop_calibration:
            self.send_calibration_command("stop")
            if not keep_running:
                self.get_logger().info("Sent stop command, exiting...")
                rclpy.shutdown()
    
    def connect_to_mqtt(self):
        """Connect to the MQTT broker with retries"""
        self.mqtt_sub.connect_with_retries()
        self.logger.log_info(f"Connected to MQTT broker for topic: {self.topic}")
        print(f"\033[1;36m[SERVER] Subscribed to MQTT topic: {self.topic}\033[0m")
        
        # Store the latest calibration target data
        self.latest_calibration_target = None
    
    def process_mqtt(self):
        """Process MQTT messages and handle the queue"""
        # Call the MQTT loop to process messages
        self.mqtt_sub.loop()
        
        # Process any messages in our local queue
        if self.received_messages:
            # Get the oldest message
            payload = self.received_messages.pop(0)
            
            try:
                # Process the message
                self.process_calibration_data(payload)
            except Exception as e:
                self.logger.log_error(f"Error processing message: {e}")
                import traceback
                self.logger.log_error(traceback.format_exc())
    
    def print_status(self):
        """Print periodic status updates"""
        print(f"\033[1;36m[SERVER] Waiting for calibration data on topic: {self.topic} (received so far: {self.message_count})\033[0m")
    
    def calibration_target_callback(self, message):
        """Callback for MQTT messages"""
        try:
            # Handle different message types
            if isinstance(message, bytes):
                # Decode the message if it's bytes
                payload = message.decode("utf-8")
            elif isinstance(message, str):
                # Use as is if it's already a string
                payload = message
            else:
                # Convert to string for other types
                payload = str(message)
            
            # Store the message
            self.received_messages.append(payload)
            self.logger.log_info(f"Received message, total count: {len(self.received_messages)}")
            
        except Exception as e:
            self.logger.log_error(f"Error in calibration_target_callback: {e}")
            import traceback
            self.logger.log_error(traceback.format_exc())
    
    def process_calibration_data(self, payload):
        """Process the calibration data from the payload"""
        try:
            self.logger.log_info(f"Processing calibration target data")
            print(f"\033[1;32m[SERVER] Processing MQTT message: {payload[:100]}...\033[0m")
            
            # Parse the JSON data
            calibration_data = json.loads(payload)
            self.latest_calibration_target = calibration_data
            self.message_count += 1
            
            # Log the message type
            if 'type' in calibration_data:
                self.logger.log_info(f"Message type: {calibration_data['type']}")
            
            # Check if this is a test message
            is_test = calibration_data.get('test', False)
            if is_test:
                self.logger.log_info("This is a test message")
            
            # Extract workspace center coordinates if available
            if 'workspace_center_cm' in calibration_data:
                center = calibration_data['workspace_center_cm']
                self.logger.log_info(f"Workspace center at: x={center['x']:.1f}cm, y={center['y']:.1f}cm, z={center['z']:.1f}cm")
                
                # Print formatted output to console for easier viewing
                print(f"\033[1;34m[SERVER] Received workspace center (cm): X={center['x']:.1f}, Y={center['y']:.1f}, Z={center['z']:.1f}\033[0m")
            
            # Only save the calibration data if we haven't saved it yet
            if not self.calibration_data_saved:
                # Add received timestamp to the calibration data
                calibration_data['received_timestamp'] = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
                
                # Save the calibration data (overwrite the file)
                with open(CALIBRATION_FILE, 'w') as f:
                    json.dump(calibration_data, f, indent=2)
                self.logger.log_info(f"Saved calibration data to: {CALIBRATION_FILE}")
                
                # Set the flag to indicate we've saved the data
                self.calibration_data_saved = True
                print(f"\033[1;32m[SERVER] Calibration data saved. Further updates will be ignored.\033[0m")
            else:
                # Log that we're ignoring subsequent data
                self.logger.log_info("Ignoring subsequent calibration data (already saved)")
                
        except Exception as e:
            self.logger.log_error(f"Error processing calibration target data: {e}")
            import traceback
            self.logger.log_error(traceback.format_exc())
            print(f"\033[1;31m[SERVER] Error processing MQTT message: {e}\033[0m")

    def send_calibration_command(self, command_type):
        """Send a command to the robot to start or stop calibration"""
        from ras_transport.interfaces.TransportWrapper import TransportMQTTPublisher
        import time
        
        # Define the command topic
        command_topic = "robot/calibration_command"
        
        # Create a publisher
        command_pub = TransportMQTTPublisher(command_topic)
        command_pub.connect_with_retries()
        
        # Create the command message
        command = {
            "command": command_type,
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
        }
        
        # Convert to JSON string
        command_json = json.dumps(command)
        
        # Publish the command
        command_pub.publish(command_json)
        
        # Log the command
        self.logger.log_info(f"Sent calibration command: {command_type}")
        print(f"\033[1;32m[SERVER] Sent calibration command: {command_type}\033[0m")

def main(args=None):
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Calibration Target Subscriber')
    parser.add_argument('--start', action='store_true', help='Send start calibration command')
    parser.add_argument('--stop', action='store_true', help='Send stop calibration command')
    parser.add_argument('--no-keep-running', action='store_true', help='Exit after sending command')
    
    # Parse the arguments
    parsed_args = parser.parse_args()
    
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create the node with the specified options
    node = CalibrationTargetSubscriber(
        start_calibration=parsed_args.start,
        stop_calibration=parsed_args.stop,
        keep_running=not parsed_args.no_keep_running
    )
    
    # If we're just sending a command and not keeping running, the node will shut down itself
    if (parsed_args.start or parsed_args.stop) and parsed_args.no_keep_running:
        # Just let the node initialize and send the command
        rclpy.spin_once(node)
    else:
        # Otherwise, spin the node normally
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            print("\nShutting down Calibration Target Subscriber...")
        finally:
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
