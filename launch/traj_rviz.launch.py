from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
 
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rvizvisualisation',
            arguments="-d $(find grad_traj_optimization)/rviz/traj.rviz"
        ),
    ])
