# Copyright (c) 2022 Dunder Mifflin, Inc.
# All rights reserved.

import sys
from time import sleep

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import odrive
from odrive_interfaces.srv import AxisState

class OdriveROS2(Node):
    def __init__(self):
        super().__init__('odrive_ros2')
        self.get_logger().info('Instantiated odrive_ros2 node.')
        
        # Declaring objects
        self._odrive = None
        self._axis0_vel_ref_subscriber = None
        self._axis1_vel_ref_subscriber = None

        self._instantiate_services()
        self._instantiate_subscribers()
        if self._find_odrive():
            self._run_calibration_sequence()

    def _instantiate_services(self):
        self.request_state_service = self.create_service(AxisState,
                                                         'request_state',
                                                         self._request_state_callback)

    def _instantiate_subscribers(self):
        self._axis0_vel_ref_subscriber = self.create_subscription(Float32,
                                                                  'axis0_vel_ref',
                                                                  self._axis0_vel_ref_callback,
                                                                  10)
        self._axis1_vel_ref_subscriber = self.create_subscription(Float32,
                                                                  'axis1_vel_ref',
                                                                  self._axis1_vel_ref_callback,
                                                                  10)

    def _find_odrive(self):
        self.get_logger().info('Searching for odrives..')
        self._odrive = odrive.find_any(timeout=5.0)
        if self._odrive:
            self.get_logger().info('ODrive connected.')
            return True
        else:
            self.get_logger().info('Searching for ODrives timed out.')
            self.get_logger().info('No ODrives found.')
            return False

    def _run_calibration_sequence(self):
        self.get_logger().info('Running calibration sequence..')
        self._odrive.axis0.requested_state = 3
        self._odrive.axis0.watchdog_feed()
        self._odrive.axis1.requested_state = 3
        self._odrive.axis1.watchdog_feed()

        while (self._odrive.axis0.current_state != 1) or (self._odrive.axis1.current_state != 1):
            sleep(0.5)
        self.get_logger().info('Calibration sequence complete.')

    def _is_odrive_ready(self):
        self.get_logger().info('Checking if ODrive is ready..')
        if self._odrive:
            try:
                if self._odrive.user_config_loaded:
                    return True
                else:
                    self.get_logger().warn('ODrive user config not loaded.')
            except:
                self.get_logger().error('Unexpected error: ', sys.exc_info()[0])
                return False
        else:
            self.get_logger().warn('ODrive not connected.')

    """
    AXIS_STATE_UNDEFINED = 0
    AXIS_STATE_IDLE = 1
    AXIS_STATE_STARTUP_SEQUENCE = 2
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3
    AXIS_STATE_MOTOR_CALIBRATION = 4
    AXIS_STATE_SENSORLESS_CONTROL = 5
    AXIS_STATE_ENCODER_INDEX_SEARCH = 6
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
    AXIS_STATE_CLOSED_LOOP_CONTROL = 8
    AXIS_STATE_LOCKIN_SPIN = 9
    AXIS_STATE_ENCODER_DIR_FIND = 10
    """
    def _request_state_callback(self, request, response):
        if self._is_odrive_ready():
            if request.axis == 0:
                self._odrive.axis0.requested_state = request.state
                self._odrive.axis0.watchdog_feed()
                response.success = True
                response.state = self._odrive.axis0.current_state
                response.message = f'Success'
            elif request.axis == 1:
                self._odrive.axis1.requested_state = request.state
                self._odrive.axis1.watchdog_feed()
                response.success = True
                response.state = self._odrive.axis1.current_state
                response.message = f'Success'
            else:
                response.success = False
                response.message = f'Axis not exist'
        else:
            response.success = False
            response.message = f'ODrive not ready'
        
        return response

    def _axis0_vel_ref_callback(self, msg):
        self._odrive.axis0.controller.input_vel = msg.data

    def _axis1_vel_ref_callback(self, msg):
        self._odrive.axis1.controller.input_vel = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = OdriveROS2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()