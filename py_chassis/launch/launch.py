#ver=1.2
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import yaml

def generate_launch_description():
    commonFilePath = os.path.join(get_package_share_directory('py_chassis'), 'launch/common.yaml')
    with open(commonFilePath, 'r') as f:
        data = yaml.load(f)
    return LaunchDescription([
        Node(
            package="py_chassis",
            namespace=data['generic_prop']['namespace'],
            executable="pub",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "topic_chassis_nodeName" : data['topic_chassis']['nodeName'] + '_' + str(data['generic_prop']['id']) + '_node', 
                    "topic_chassis_topicName" : data['topic_chassis']['topicName'] + '_' + str(data['generic_prop']['id']), 
                    "topic_chassis_pubInterval" : data['topic_chassis']['publishInterval'], 
                    "internalIDServerIP" : data['internal_prop']['hostIP'], 

                    # Settings for Params class under vehicle_interfaces/params.h
                    # Do not change the settings rashly
                    "nodeName" : data['generic_prop']['nodeName'] + '_' + str(data['generic_prop']['id']) + '_node', 
                    "id" : data['generic_prop']['id'], 
                    "devInfoService" : data['generic_prop']['devInfoService'], 
                    "devInterface" : data['generic_prop']['devInterface'], 
                    "qosService" : data['generic_prop']['qosService'], 
                    "qosDirPath" : data['generic_prop']['qosDirPath'], 
                    "safetyService" : data['generic_prop']['safetyService'], 
                    "timesyncService" : data['generic_prop']['timesyncService'], 
                    "timesyncPeriod_ms" : data['generic_prop']['timesyncPeriod_ms'], 
                    "timesyncAccuracy_ms" : data['generic_prop']['timesyncAccuracy_ms'], 
                }
            ]
        )
    ])
