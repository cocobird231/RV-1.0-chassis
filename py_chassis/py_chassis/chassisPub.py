# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from vehicle_interfaces.msg import MotorAxle
from vehicle_interfaces.msg import MotorSteering
from pathlib import Path
from os.path import expanduser
import pickle
import io
import json

import RunClient
import threading
import time


class Params(Node):
    def __init__(self, nodeName : str):
        super().__init__(nodeName)
        self.topic_chassis_nodeName = 'chassis_publisher_node'
        self.topic_chassis_topicName = 'chassis_0'
        self.topic_chassis_pubInterval = 0.05
        self.mainNodeName = 'chassis_node'
        self.internalIDServerIP = '192.168.1.42'
        self.internalIDServerDeviceID = 11

        self.declare_parameter('topic_chassis_nodeName', self.topic_chassis_nodeName)
        self.declare_parameter('topic_chassis_topicName', self.topic_chassis_topicName)
        self.declare_parameter('topic_chassis_pubInterval', self.topic_chassis_pubInterval)
        self.declare_parameter('mainNodeName', self.mainNodeName)
        self.declare_parameter('internalIDServerIP', self.internalIDServerIP)
        self.declare_parameter('internalIDServerDeviceID', self.internalIDServerDeviceID)
        self._getParam()
    
    def _getParam(self):
        self.topic_chassis_nodeName = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_chassis_nodeName').get_parameter_value())
        self.topic_chassis_topicName = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_chassis_topicName').get_parameter_value())
        self.topic_chassis_pubInterval = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_chassis_pubInterval').get_parameter_value())
        self.mainNodeName = rclpy.parameter.parameter_value_to_python(self.get_parameter('mainNodeName').get_parameter_value())
        self.internalIDServerIP = rclpy.parameter.parameter_value_to_python(self.get_parameter('internalIDServerIP').get_parameter_value())
        self.internalIDServerDeviceID = rclpy.parameter.parameter_value_to_python(self.get_parameter('internalIDServerDeviceID').get_parameter_value())


class ChassisMotorPublisher(Node):
    def __init__(self, nodeName, topicName, interval_s):
        super().__init__(nodeName)
        self.nodeName_ = nodeName
        self.publisher_ = self.create_publisher(MotorAxle, topicName, 10)
        self.frame_id_ = 0
        timer_period = interval_s  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):        
        msg = MotorAxle()
        msg.header.priority = msg.header.PRIORITY_SENSOR
        msg.header.device_type = msg.header.DEVTYPE_MOTOR_AXLE
        msg.header.device_id = self.nodeName_
        msg.header.frame_id = self.frame_id_
        self.frame_id_ += 1
        msg.header.stamp_type = msg.header.STAMPTYPE_NO_SYNC
        msg.header.stamp = self.get_clock().now().to_msg()

        RunClient.ros2DictLock.acquire()
        tmp = RunClient.axleDict
        RunClient.ros2DictLock.release()
        
        msg.dir = tmp["dir"]
        msg.pwm = float(tmp["pwm"])
        msg.parking = tmp["parking"]
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d %f %d"' % (msg.dir, msg.pwm, msg.parking))
    

class ChassisSteeringPublisher(Node):
    def __init__(self, nodeName, topicName, interval_s):
        super().__init__(nodeName)
        self.nodeName_ = nodeName
        self.publisher_ = self.create_publisher(MotorSteering, topicName, 10)
        self.frame_id_ = 0
        timer_period = interval_s  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):        
        msg = MotorSteering()
        msg.header.priority = msg.header.PRIORITY_SENSOR
        msg.header.device_type = msg.header.DEVTYPE_MOTOR_STEERING
        msg.header.device_id = self.nodeName_
        msg.header.frame_id = self.frame_id_
        self.frame_id_ += 1
        msg.header.stamp_type = msg.header.STAMPTYPE_NO_SYNC
        msg.header.stamp = self.get_clock().now().to_msg()

        RunClient.ros2DictLock.acquire()
        tmp = RunClient.steeringDict
        RunClient.ros2DictLock.release()
        
        msg.unit_type = msg.UNIT_DIST
        msg.min = float(tmp["min"])
        msg.max = float(tmp["max"])
        msg.center = float(tmp["center"])
        msg.value = float(tmp["value"])
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "min: %f max: %f center: %f value: %f"' % (msg.min, msg.max, msg.center, msg.value))



def main(args=None):
    rclpy.init(args=args)
    params = Params('chassis_params_node')
    print(params.internalIDServerIP)
    print(params.internalIDServerDeviceID)

    ########################
    # Run client
    ########################
    RunClient.myID = params.internalIDServerDeviceID
    dataSock = RunClient.sockConnect(params.internalIDServerIP)
    RunClient.sendCommandTellId(dataSock)
    handler = threading.Thread(target=RunClient.LoopReceiveDataAndProcess, args=(dataSock, ))
    handler.start()

    imAliveSock = RunClient.sockConnectImAlive(params.internalIDServerIP)
    receivedData = RunClient.sendCommandTellId(imAliveSock)
    handler1 = threading.Thread(target=RunClient.LoopSendAlive, args=(imAliveSock, ))
    handler1.start()

    moduleProcessTh = threading.Thread(target=RunClient.ModuleProcess, args=(0.1, ))
    moduleProcessTh.start()

    ########################
    # Run ROS2 Publisher
    ########################
    if (params.internalIDServerDeviceID >= 11 and params.internalIDServerDeviceID <= 14):
        params.mainNodeName = 'chassis_motor_%d_node' %(params.internalIDServerDeviceID - 11)
        params.topic_chassis_topicName = 'chassis_motor_%d' %(params.internalIDServerDeviceID - 11)
        chassis_publisher = ChassisMotorPublisher(params.mainNodeName, params.topic_chassis_topicName, params.topic_chassis_pubInterval)
        
    elif (params.internalIDServerDeviceID >= 41 and params.internalIDServerDeviceID <= 44):
        params.mainNodeName = 'chassis_steering_%d_node' %(params.internalIDServerDeviceID - 41)
        params.topic_chassis_topicName = 'chassis_steering_%d' %(params.internalIDServerDeviceID - 41)
        chassis_publisher = ChassisSteeringPublisher(params.mainNodeName, params.topic_chassis_topicName, params.topic_chassis_pubInterval)
    else:
        while (True):
            if RunClient.LoopReceiveDataAndProcess.bWillStop or RunClient.LoopSendAlive.bWillStop:
                return 0
            time.sleep(1)
    
    rclpy.spin(chassis_publisher)
    chassis_publisher.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
