#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright (C) 2016 Toyota Motor Corporation

import math
import os
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import yaml
import sys
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from hsrb_interface import Robot
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from tmc_manipulation_msgs.srv import SafeJointChange
from rclpy.qos import qos_profile_sensor_data

from grasping_pipeline_msgs.action import Handover


class HandoverServer(Node):
    def __init__(self):
        super().__init__('handover_server')
        self.get_logger().info("starting handover server")
        self.declare_parameter('grasping_pipeline.dataset', 'ycb_ichores')
        self.dataset = self.get_parameter('grasping_pipeline.dataset')
        try:
            package_path = get_package_share_directory('grasping_pipeline')
            config_path = Path(package_path + '/config/object_mapping.yaml')

            with open(config_path, 'r') as file:
                self.object_mapping = yaml.safe_load(file)

            self.get_logger().info("Object mapping loaded successfully")

        except Exception as e:
            self.get_logger().error(f"Failed to load placement config: {e}")
            self.object_mapping = {}

        self.server = ActionServer(self, Handover, '/handover', execute_callback=self.execute)
        self._force_data_x = 0.0
        self._force_data_y = 0.0
        self._force_data_z = 0.0

        self.force_thresh = 0.2
        self.position_reached = False
        self.finished = False

        #Robot initialization
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.tts = self.robot.try_get('default_tts')
        self.tts.language = self.tts.ENGLISH
        self.gripper = self.robot.get('gripper')
        
        # self.readjust_offset = self.create_client(Empty, '/hsrb/wrist_wrench/readjust_offset')
        # while not self.readjust_offset.wait_for_service(timeout_sec = 5):
        #     self.get_logger().info("waiting for service: /hsrb/wrist_wrench/readjust_offset")
        self.joint_control = self.create_client(SafeJointChange, '/change_joint')
        while not self.joint_control.wait_for_service(timeout_sec = 5):
            self.get_logger().info("waiting for service: /change_joint")
        # Subscribe force torque sensor data from HSRB
        ft_sensor_topic = '/wrist_wrench/raw'
        self._wrist_wrench_sub = self.create_subscription(WrenchStamped, ft_sensor_topic, callback = self.__ft_sensor_cb, qos_profile=qos_profile_sensor_data)
        self.declare_parameter('handover.use_fancy_handover', False)
        if self.has_parameter('handover.use_fancy_handover'):
            self.use_fancy_handover = self.get_parameter('handover.use_fancy_handover').value
        else:
            self.use_fancy_handover = True

        # Wait for connection
        while rclpy.ok() and self._force_data_x is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self._force_data_x is None:
            raise RuntimeError('force torque sensor not received')
        
        self.get_logger().info("Handover server started")

    def execute(self, goal_handle):
        self.get_logger().info('Received a new goal.')
        goal = goal_handle.request

        if goal.force_thresh > 0:
            self.force_thresh = goal.force_thresh
        #self.readjust_offset()
        self.move_to_handover_position(goal.object_name)
        while not self.finished:
            time.sleep(0.5)

        self.finished = False
        self.position_reached = False        
        goal_handle.succeed()
        self.get_logger().info('Handover was successfully executed.')
        result = Handover.Result()

        return result


    def get_current_force(self):
        return [self._force_data_x, self._force_data_y, self._force_data_z]
    
    def move_to_handover_position(self, object_name):

        if self.use_fancy_handover:
            self.whole_body.move_to_neutral()
            joint_goal = JointState()
            joint_goal.name.extend(['arm_flex_joint', 'arm_lift_joint', 'wrist_flex_joint', 
                                    'arm_roll_joint', 'wrist_roll_joint', 'head_pan_joint', 'head_tilt_joint'])
            joint_goal.position.extend([-0.3, 0.4, -1, 
                                        0, 0, 0, 0])
            req = SafeJointChange.Request()
            req.ref_joint_state = joint_goal
            
            future = self.joint_control.call_async(req)

            rclpy.spin_until_future_complete(self, future)

            res = future.result()

            if res is not None:
                self.position_reached = res.success
            else:
                self.get_logger().error(
                    'Failed to call joint control service.'
                )
                self.position_reached = False
            
        if len(object_name) > 0:

            try:
                if "obj_" in object_name:
                    # Getting real object name from the params
                    
                    object_name = self.object_mapping[self.dataset]
            except:
                self.get_logger().info("Object name not found in the params. Using the object name as it is.")

            if object_name[0].isdigit():
                object_name = object_name.split("_", 1)[-1]
            self.position_reached = True
            self.tts.say(f"You can take the {object_name} now.")
        else:
            self.tts.say('You can take the object now.')
        self.get_logger().info('You can take the object now.')


    def reset_offset(self):
        self.readjust_offset()

    def __ft_sensor_cb(self, data):
        self._force_data_x = data.wrench.force.x
        self._force_data_y = data.wrench.force.y
        self._force_data_z = data.wrench.force.z
        if (abs(self._force_data_x) > self.force_thresh or abs(self._force_data_y) > self.force_thresh or abs(self._force_data_z) > self.force_thresh) \
            and self.position_reached and not self.finished:
            self.gripper.command(1.0)
            self.finished = True
    
        
def main():
    rclpy.init()
    node = HandoverServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
