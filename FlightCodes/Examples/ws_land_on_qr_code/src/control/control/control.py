#!/usr/bin/env python3

import rclpy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Float64MultiArray


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers for drone control
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers for drone state
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        
        # Create subscribers for camera estimated qrcode position
        self.qrcode_position_subscriber = self.create_subscription(
            Float64MultiArray, '/position', self.qrcode_position_callback, 10)
        

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.offboard_takeoff_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -4.5
        self.position = []
        self.time_landed = 0
        self.timer_interval = 0.5
        self.takeoff_pos = [0.0, 0.0, self.takeoff_height]
        self.velocity = [0.0, 0.0, 0.0]

        # Create a timer to publish control commands
        self.timer = self.create_timer(self.timer_interval, self.timer_callback)
        
    def qrcode_position_callback(self, data):
        self.position = data.data
        print(f"Received position: {self.position}")

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = True
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float, printFlag: bool = False):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        if printFlag:
            self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")
            
    def publish_velocity_setpoint(self, vx: float, vy: float, vz: float, printFlag: bool = False):
        """Publish the velocity setpoint."""
        msg = TrajectorySetpoint(
            velocity = [vx, vy, vz],
            position=[None, None, None],
            acceleration=[None, None, None],
            jerk=[None, None, None],
            yaw=0.0,
            timestamp = int(self.get_clock().now().nanoseconds / 1000))
        self.trajectory_setpoint_publisher.publish(msg)
        if printFlag:
            self.get_logger().info(f"Publishing velocity setpoints {[vx, vy, vz]}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()  
            
        self.get_logger().info(f"Vehicle status: {self.vehicle_status.nav_state}")
        self.get_logger().info(f"Vehicle local position (x, y, z): ({self.vehicle_local_position.x}, {self.vehicle_local_position.y}, {self.vehicle_local_position.z})")

        # Se o drone estiver em modo offboard e a altura for menor que a altura de decolagem, decolar
        if self.vehicle_local_position.z > self.takeoff_height and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if self.offboard_takeoff_counter < 10:
                self.offboard_takeoff_counter += 1
                self.get_logger().info(f"Takeoff counter: {self.offboard_takeoff_counter}")
                self.publish_velocity_setpoint(0.0, 0.0, -1.0, True)

        #Se o drone estiver em modo offboard e a altura for maior que a altura de decolagem, sobrevoa o qrcode
        elif self.vehicle_local_position.z <= (self.takeoff_height + 1) and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            print(f"Acima de takeoff_height, position é {self.position}")
            if len(self.position) > 0:
                    
                self.publish_position_setpoint(self.position[0], self.position[1], self.takeoff_height, True)

                    
                    # self.velocity[0] = -1.0
                    # self.velocity[1] = -1.0
                    # self.velocity[2] = 0.0
                    
                    # self.publish_velocity_setpoint(self.velocity[0], self.velocity[1], self.velocity[2], True)
                # else:
                #     self.velocity[0] = 0.0
                #     self.velocity[1] = 0.0
                #     self.velocity[2] = 0.0
                #     self.publish_velocity_setpoint(self.velocity[0], self.velocity[1], self.velocity[2], True)
                #     self.get_logger().info("Landing")
                #     self.land()
            
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1
            
        if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().info("Disarmed")
            self.time_landed += 1
            
        if (self.time_landed * self.timer_interval) > 5:
            self.get_logger().info("Landed for 5 seconds, taking off again")
            self.time_landed = 0
            self.offboard_setpoint_counter = 0
            self.offboard_takeoff_counter = 0
            self.position = []
            self.takeoff_pos = [0.0, 0.0, self.takeoff_height]

            

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
