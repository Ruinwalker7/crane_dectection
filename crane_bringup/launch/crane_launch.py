import launch
import launch_ros.actions
from launch_ros.actions import Node


def generate_launch_description():
    camera_node = Node(
            package='camera',
            executable='camera_node',
            name='camera')
    
    detector_node = Node(
        package='detector',
        executable='detector',
        name='detector'
    )
    serial_node = Node(
        package='serial_driver',
        executable='serial_driver_node',
        name='serial_driver_node'
    )
    return launch.LaunchDescription([
        camera_node,
        detector_node,
        serial_node
  ])