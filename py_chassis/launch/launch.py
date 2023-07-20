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
            namespace=data['node_prop']['namespace'],
            executable="pub",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "topic_chassis_nodeName" : data['topic_chassis']['nodeName'], 
                    "topic_chassis_topicName" : data['topic_chassis']['topicName'], 
                    "topic_chassis_pubInterval" : data['topic_chassis']['publishInterval'], 
                    "mainNodeName" : data['node_prop']['nodeName'], 
                    "internalIDServerIP" : data['internal_prop']['hostIP'], 
                    "internalIDServerDeviceID" : data['internal_prop']['ID'], 
                }
            ]
        )
    ])
