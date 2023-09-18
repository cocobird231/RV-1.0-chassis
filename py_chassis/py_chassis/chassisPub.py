import rclpy
from rclpy.node import Node

from vehicle_interfaces.msg import MotorAxle
from vehicle_interfaces.msg import MotorSteering
from vehicle_interfaces.params import GenericParams
from vehicle_interfaces.vehicle_interfaces import VehicleServiceNode

from pathlib import Path
from os.path import expanduser
import pickle
import io
import json

import RunClient
import threading
import time

class Params(GenericParams):
    def __init__(self, nodeName : str):
        super().__init__(nodeName)
        self.topic_chassis_nodeName = 'chassis_publisher_node'
        self.topic_chassis_topicName = 'chassis_0'
        self.topic_chassis_pubInterval = 0.05
        self.internalIDServerIP = '192.168.1.42'

        self.declare_parameter('topic_chassis_nodeName', self.topic_chassis_nodeName)
        self.declare_parameter('topic_chassis_topicName', self.topic_chassis_topicName)
        self.declare_parameter('topic_chassis_pubInterval', self.topic_chassis_pubInterval)
        self.declare_parameter('internalIDServerIP', self.internalIDServerIP)
        self._getParam()
    
    def _getParam(self):
        self.topic_chassis_nodeName = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_chassis_nodeName').get_parameter_value())
        self.topic_chassis_topicName = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_chassis_topicName').get_parameter_value())
        self.topic_chassis_pubInterval = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_chassis_pubInterval').get_parameter_value())
        self.internalIDServerIP = rclpy.parameter.parameter_value_to_python(self.get_parameter('internalIDServerIP').get_parameter_value())


class ChassisMotorPublisher(VehicleServiceNode):
    def __init__(self, params : Params):
        super().__init__(params)
        self.__params = params

        self.addQoSCallbackFunc(self.__qosCallback)

        prof = self.addQoSTracking(params.topic_chassis_topicName)
        if (prof != None):
            self.__publisher = self.create_publisher(MotorAxle, self.__params.topic_chassis_topicName, prof)
        else:
            self.__publisher = self.create_publisher(MotorAxle, self.__params.topic_chassis_topicName, 10)
        
        self.__frame_id = 0
        self.__timer = self.create_timer(self.__params.topic_chassis_pubInterval, self.__timerCallback)

    def __qosCallback(self, qmap):
        self.get_logger().info('[ChassisMotorPublisher.__qosCallback] Get qmap size: %d' %len(qmap))
        for topic in qmap:
            self.get_logger().info('[ChassisMotorPublisher.__qosCallback] Get qmap[%s]' %topic)

    def __timerCallback(self): 
        msg = MotorAxle()
        msg.header.priority = msg.header.PRIORITY_SENSOR
        msg.header.device_type = msg.header.DEVTYPE_MOTOR_AXLE
        msg.header.device_id = self.__params.nodeName
        msg.header.frame_id = self.__frame_id
        self.__frame_id += 1
        msg.header.stamp_type = self.getTimestampType()
        msg.header.stamp = self.getTimestamp().to_msg()
        msg.header.stamp_offset = self.getCorrectDuration().nanoseconds
        msg.header.ref_publish_time_ms = self.__params.topic_chassis_pubInterval * 1000.0

        RunClient.ros2DictLock.acquire()
        tmp = RunClient.axleDict
        RunClient.ros2DictLock.release()
        
        msg.dir = tmp["dir"]
        msg.pwm = float(tmp["pwm"])
        msg.parking = tmp["parking"]
        
        self.__publisher.publish(msg)
        self.get_logger().info('Publishing: "%d %f %d"' % (msg.dir, msg.pwm, msg.parking))
    

class ChassisSteeringPublisher(VehicleServiceNode):
    def __init__(self, params : Params):
        super().__init__(params)
        self.__params = params

        self.addQoSCallbackFunc(self.__qosCallback)

        prof = self.addQoSTracking(params.topic_chassis_topicName)
        if (prof != None):
            self.__publisher = self.create_publisher(MotorSteering, self.__params.topic_chassis_topicName, prof)
        else:
            self.__publisher = self.create_publisher(MotorSteering, self.__params.topic_chassis_topicName, 10)
        
        self.__frame_id = 0
        self.__timer = self.create_timer(self.__params.topic_chassis_pubInterval, self.__timerCallback)

    def __qosCallback(self, qmap):
        self.get_logger().info('[ChassisSteeringPublisher.__qosCallback] Get qmap size: %d' %len(qmap))
        for topic in qmap:
            self.get_logger().info('[ChassisSteeringPublisher.__qosCallback] Get qmap[%s]' %topic)
    
    def __timerCallback(self):        
        msg = MotorSteering()
        msg.header.priority = msg.header.PRIORITY_SENSOR
        msg.header.device_type = msg.header.DEVTYPE_MOTOR_STEERING
        msg.header.device_id = self.__params.nodeName
        msg.header.frame_id = self.__frame_id
        self.__frame_id += 1
        msg.header.stamp_type = self.getTimestampType()
        msg.header.stamp = self.getTimestamp().to_msg()
        msg.header.stamp_offset = self.getCorrectDuration().nanoseconds
        msg.header.ref_publish_time_ms = self.__params.topic_chassis_pubInterval * 1000.0

        RunClient.ros2DictLock.acquire()
        tmp = RunClient.steeringDict
        RunClient.ros2DictLock.release()
        
        msg.unit_type = msg.UNIT_DIST
        msg.min = float(tmp["min"])
        msg.max = float(tmp["max"])
        msg.center = float(tmp["center"])
        msg.value = float(tmp["value"])
        
        self.__publisher.publish(msg)
        self.get_logger().info('Publishing: "min: %f max: %f center: %f value: %f"' % (msg.min, msg.max, msg.center, msg.value))



def main(args=None):
    rclpy.init(args=args)
    params = Params('chassis_params_node')
    print(params.internalIDServerIP)
    print(params.id)

    ########################
    # Run Jim's ID client
    ########################
    RunClient.myID = params.id
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
    if (params.id >= 11 and params.id <= 14):
        params.nodeName = 'chassis_motor_%d_node' %(params.id - 11)
        params.topic_chassis_topicName = 'chassis_motor_%d' %(params.id - 11)
        chassis_publisher = ChassisMotorPublisher(params)
        
    elif (params.id >= 41 and params.id <= 44):
        params.nodeName = 'chassis_steering_%d_node' %(params.id - 41)
        params.topic_chassis_topicName = 'chassis_steering_%d' %(params.id - 41)
        chassis_publisher = ChassisSteeringPublisher(params)
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
