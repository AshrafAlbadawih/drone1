#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import State
import serial
import time
import traceback
import sys
from math import sin, cos, pi

class DroneAutoTakeoff(Node):
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        super().__init__('drone_auto_takeoff_node')
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Starting Drone Auto Takeoff Node")
        self.get_logger().info("Auto Arm and Takeoff to 10m")
        self.get_logger().info("=" * 60)
        
        # Step 1: Initialize publishers
        self.pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.get_logger().info("✓ Publisher: /mavros/setpoint_position/local")
        
        # Step 2: Initialize subscribers
        self.state = None
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.get_logger().info("✓ Subscriber: /mavros/state")
        
        # Step 3: Initialize service clients
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.setmode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Step 4: Initialize serial connection
        self.ser = None
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.2)
            time.sleep(2)
            self.get_logger().info(f"✓ Serial connected: {port} @ {baudrate}")
            self.ser.reset_input_buffer()
        except Exception as e:
            self.get_logger().error(f"✗ Serial error: {e}")
            self.ser = None
        
        # Step 5: Initialize variables
        self.joy_min = 1000.0
        self.joy_max = 2000.0
        self.joy_center = 1500.0
        
        # Position ranges
        self.x_range = (-50.0, 50.0)
        self.y_range = (-50.0, 50.0)
        self.z_range = (5.0, 20.0)
        
        # Initialize goal pose
        self.goal = PoseStamped()
        self.goal.header.frame_id = "map"
        self.goal.pose.position.x = 0.0
        self.goal.pose.position.y = 0.0
        self.goal.pose.position.z = 10.0  # Takeoff altitude
        self.goal.pose.orientation.w = 1.0
        
        # State variables
        self.last_serial_time = time.time()
        self.joystick_control_enabled = False  # Flag to control when joystick input is processed
        self.initial_position_sent = False  # Flag to track if initial position has been sent
        
        # Auto takeoff sequence states
        self.sequence_state = "INIT"  # INIT, SET_MODE, PREARM, ARMING, TAKEOFF, HOLDING, NORMAL
        self.sequence_start_time = 0
        self.arm_attempted = False
        self.takeoff_attempted = False
        self.takeoff_altitude = 10.0  # meters
        self.takeoff_start_time = 0
        
        # Step 6: Create timers
        self.status_timer = self.create_timer(2.0, self.print_status)
        self.pub_timer = self.create_timer(0.1, self.publish_setpoint)
        self.serial_timer = self.create_timer(0.05, self.read_serial)
        self.sequence_timer = self.create_timer(0.5, self.run_sequence)  # Main sequence timer
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Auto Takeoff Node Ready")
        self.get_logger().info("Waiting for MAVROS connection...")
        self.get_logger().info("=" * 60)
    
    def map_range(self, value, in_min, in_max, out_min, out_max):
        if in_max == in_min:
            return out_min
        return (value - in_min) / (in_max - in_min) * (out_max - out_min) + out_min
    
    def state_cb(self, msg: State):
        old_state = self.state
        self.state = msg
        
        if old_state is None or \
           old_state.connected != msg.connected or \
           old_state.armed != msg.armed or \
           old_state.mode != msg.mode:
            
            self.get_logger().info(f"MAVROS State:")
            self.get_logger().info(f"  Connected: {msg.connected}")
            self.get_logger().info(f"  Armed: {msg.armed}")
            self.get_logger().info(f"  Mode: {msg.mode}")
            self.get_logger().info(f"  System Status: {msg.system_status}")
    
    def publish_setpoint(self):
        try:
            self.goal.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(self.goal)
        except Exception as e:
            self.get_logger().error(f"Error in publish_setpoint: {e}")
    
    def read_serial(self):
        """Read serial data but only process joystick input when enabled"""
        if self.ser is None:
            return
        
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                
                if not line:
                    return
                
                if 'READY' in line:
                    self.get_logger().info(f"Arduino: {line}")
                    return
                
                parts = line.split(',')
                
                if len(parts) >= 10:
                    try:
                        x_raw = float(parts[0])
                        y_raw = float(parts[1])
                        z_raw = float(parts[2])
                        roll_raw = float(parts[3])
                        pitch_raw = float(parts[4])
                        yaw_raw = float(parts[5])
                        
                        # Only update joystick values if joystick control is enabled
                        if self.joystick_control_enabled:
                            x_mapped = self.map_range(x_raw, self.joy_min, self.joy_max, *self.x_range)
                            y_mapped = self.map_range(y_raw, self.joy_min, self.joy_max, *self.y_range)
                            z_mapped = self.map_range(z_raw, self.joy_min, self.joy_max, *self.z_range)
                            
                            self.goal.pose.position.x = x_mapped
                            self.goal.pose.position.y = y_mapped
                            self.goal.pose.position.z = z_mapped
                        
                        self.last_serial_time = time.time()
                        
                    except ValueError:
                        return
                        
        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")
    
    def run_sequence(self):
        """Main auto takeoff sequence"""
        try:
            # Check if we have MAVROS state
            if self.state is None:
                self.get_logger().info("Waiting for MAVROS state...")
                return
            
            # Check if connected
            if not self.state.connected:
                self.get_logger().info("Waiting for MAVROS connection...")
                return
            
            # State machine for auto takeoff
            if self.sequence_state == "INIT":
                self.sequence_state = "SET_MODE"
                self.sequence_start_time = time.time()
                self.get_logger().info("Starting auto takeoff sequence...")
                
                # Set initial position to zero and disable joystick control
                self.goal.pose.position.x = 0.0
                self.goal.pose.position.y = 0.0
                self.goal.pose.position.z = 0.0
                self.joystick_control_enabled = False
            
            elif self.sequence_state == "SET_MODE":
                # Set mode to GUIDED for auto takeoff
                if self.state.mode != 'GUIDED':
                    self.set_mode('GUIDED')
                    self.get_logger().info("Setting mode to GUIDED...")
                
                # Wait a bit for mode change
                if time.time() - self.sequence_start_time > 3.0:
                    self.sequence_state = "PREARM"
                    self.sequence_start_time = time.time()
                    self.get_logger().info("Starting prearm setpoints...")
            
            elif self.sequence_state == "PREARM":
                # Publish setpoints at zero position before arming
                elapsed = time.time() - self.sequence_start_time
                
                # Keep position at zero during prearm
                self.goal.pose.position.x = 0.0
                self.goal.pose.position.y = 0.0
                self.goal.pose.position.z = 0.0
                
                if elapsed < 5.0:  # 5 seconds of prearm
                    if int(elapsed) % 2 == 0:
                        self.get_logger().info(f"Prearm: {5 - int(elapsed)}s remaining")
                else:
                    self.sequence_state = "ARMING"
                    self.sequence_start_time = time.time()
                    self.get_logger().info("Attempting to arm...")
            
            elif self.sequence_state == "ARMING":
                # Keep position at zero during arming
                self.goal.pose.position.x = 0.0
                self.goal.pose.position.y = 0.0
                self.goal.pose.position.z = 0.0
                
                # Attempt to arm
                if not self.state.armed and not self.arm_attempted:
                    self.arm_drone()
                    self.arm_attempted = True
                
                # Check if armed
                if self.state.armed:
                    self.sequence_state = "TAKEOFF"
                    self.sequence_start_time = time.time()
                    self.takeoff_start_time = time.time()
                    self.get_logger().info("✓ Armed! Preparing for takeoff...")
                elif time.time() - self.sequence_start_time > 10.0:
                    # Retry arming after 10 seconds
                    self.arm_attempted = False
                    self.get_logger().info("Retrying arming...")
            
            elif self.sequence_state == "TAKEOFF":
                # Set takeoff position (zero horizontal, 10m altitude)
                self.goal.pose.position.x = 0.0
                self.goal.pose.position.y = 0.0
                self.goal.pose.position.z = self.takeoff_altitude
                
                # Takeoff to 10 meters
                if not self.takeoff_attempted:
                    self.takeoff_drone(self.takeoff_altitude)
                    self.takeoff_attempted = True
                    self.get_logger().info(f"Taking off to {self.takeoff_altitude}m...")
                
                # Check altitude (simplified - wait for sufficient time)
                takeoff_duration = time.time() - self.takeoff_start_time
                if takeoff_duration > 10.0:  # Wait 10 seconds for takeoff
                    self.sequence_state = "HOLDING"
                    self.sequence_start_time = time.time()
                    self.get_logger().info("✓ Takeoff complete! Holding at 10m...")
            
            elif self.sequence_state == "HOLDING":
                # Hold at takeoff altitude for stabilization
                self.goal.pose.position.x = 0.0
                self.goal.pose.position.y = 0.0
                self.goal.pose.position.z = self.takeoff_altitude
                
                elapsed = time.time() - self.sequence_start_time
                if elapsed < 3.0:  # Hold for 3 seconds
                    self.get_logger().info(f"Holding at altitude: {3 - int(elapsed)}s remaining")
                else:
                    self.sequence_state = "NORMAL"
                    self.joystick_control_enabled = True  # Enable joystick control
                    self.get_logger().info("✓ Holding complete! Switching to joystick control...")
                    self.get_logger().info("Joystick control is now ACTIVE")
            
            elif self.sequence_state == "NORMAL":
                # Normal operation - joystick control is enabled
                pass
            
        except Exception as e:
            self.get_logger().error(f"Error in run_sequence: {e}")
            traceback.print_exc()
    
    def arm_drone(self):
        """Arm the drone"""
        if self.arm_client.wait_for_service(timeout_sec=1.0):
            req = CommandBool.Request()
            req.value = True
            future = self.arm_client.call_async(req)
            future.add_done_callback(self.arm_callback)
        else:
            self.get_logger().warn("Arm service not available")
    
    def arm_callback(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info("✓ Arming command accepted")
            else:
                self.get_logger().warn("✗ Arming command rejected")
        except Exception as e:
            self.get_logger().error(f"Arming error: {e}")
    
    def takeoff_drone(self, altitude):
        """Takeoff to specified altitude"""
        if self.takeoff_client.wait_for_service(timeout_sec=1.0):
            req = CommandTOL.Request()
            req.altitude = float(altitude)
            req.latitude = 0.0
            req.longitude = 0.0
            req.min_pitch = 0.0
            req.yaw = 0.0
            
            future = self.takeoff_client.call_async(req)
            future.add_done_callback(self.takeoff_callback)
        else:
            self.get_logger().warn("Takeoff service not available")
    
    def takeoff_callback(self, future):
        try:
            res = future.result()
            if hasattr(res, 'success') and res.success:
                self.get_logger().info("✓ Takeoff command accepted")
            else:
                self.get_logger().warn("✗ Takeoff command rejected")
        except Exception as e:
            self.get_logger().error(f"Takeoff error: {e}")
    
    def set_mode(self, mode_str):
        """Set flight mode"""
        if self.setmode_client.wait_for_service(timeout_sec=1.0):
            req = SetMode.Request()
            req.custom_mode = mode_str
            self.setmode_client.call_async(req)
    
    def print_status(self):
        """Print current status"""
        if self.state:
            self.get_logger().info("=" * 40)
            self.get_logger().info(f"State: {self.sequence_state}")
            self.get_logger().info(f"MAVROS: {'Connected' if self.state.connected else 'Disconnected'}")
            self.get_logger().info(f"Armed: {self.state.armed}")
            self.get_logger().info(f"Mode: {self.state.mode}")
            self.get_logger().info(f"Position: X={self.goal.pose.position.x:.1f}, Y={self.goal.pose.position.y:.1f}, Z={self.goal.pose.position.z:.1f}")
            self.get_logger().info(f"Joystick Control: {'ENABLED' if self.joystick_control_enabled else 'DISABLED'}")
            
            if self.ser:
                time_since = time.time() - self.last_serial_time
                self.get_logger().info(f"Serial: Active ({time_since:.1f}s since last data)")
            else:
                self.get_logger().info("Serial: Disconnected")
            
            self.get_logger().info("=" * 40)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DroneAutoTakeoff(port='/dev/ttyUSB0', baudrate=115200)
        
        node.get_logger().info("\n" + "="*60)
        node.get_logger().info("AUTO TAKEOFF SEQUENCE (MODIFIED)")
        node.get_logger().info("="*60)
        node.get_logger().info("Modified Sequence:")
        node.get_logger().info("1. Connect to MAVROS")
        node.get_logger().info("2. Set mode to GUIDED")
        node.get_logger().info("3. Publish ZERO setpoints for 5 seconds")
        node.get_logger().info("4. Arm drone")
        node.get_logger().info("5. Takeoff to 10 meters (joystick DISABLED)")
        node.get_logger().info("6. Hold at 10m for 3 seconds")
        node.get_logger().info("7. Enable joystick control")
        node.get_logger().info("="*60 + "\n")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info("\nShutting down...")
    except Exception as e:
        node.get_logger().error(f"Fatal error: {e}")
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()