import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='grad_traj_optimization',
            executable='text_input',
            output='screen',
            name='traj_opti_node1',
            parameters=[
                {'/traj_opti_node1/waypoint_num': 2},
                {'/traj_opti_node1/waypoint_x_1': 0.0},
                {'/traj_opti_node1/waypoint_y_1': 0.0},
                {'/traj_opti_node1/waypoint_z_1': 2.0},
                {'/traj_opti_node1/waypoint_x_2': 10.0},
                {'/traj_opti_node1/waypoint_y_2': 30.0},
                {'/traj_opti_node1/waypoint_z_2': 2.0},
                {'/traj_opti_node1/alg': 24},
                {'/traj_opti_node1/offset': 1.5},
                {'/traj_opti_node1/retry_offset': 0.2},
                {'/traj_opti_node1/time_limit_1': 0.04},
                {'/traj_opti_node1/time_limit_2': 0.06},
                {'/traj_opti_node1/try_limit': 0.01},
                {'/traj_opti_node1/dt': 0.2},

                {'/traj_opti_node1/ws': 200.0},
                {'/traj_opti_node1/wc': 0.1},

                # {'/traj_opti_node1/time': 2.0},
                {'/traj_opti_node1/segment_time': 1.0},
                {'/traj_opti_node1/mean_v': 1.0},
                {'/traj_opti_node1/init_time': 0.0},

                {'/traj_opti_node1/alpha': 5.0},
                {'/traj_opti_node1/d0': 0.7},
                {'/traj_opti_node1/r': 1.0},

                {'/traj_opti_node1/alphav': 0.1},
                {'/traj_opti_node1/v0': 0.5},
                {'/traj_opti_node1/rv': 0.5},

                {'/traj_opti_node1/alphaa': 0.1},
                {'/traj_opti_node1/a0': 0.5},
                {'/traj_opti_node1/ra': 0.5},

                {'/traj_opti_node1/bos': 3.0},
                {'/traj_opti_node1/vos': 8.0},
                {'/traj_opti_node1/aos': 10.0},

                {'/traj_opti_node1/gd_value': 5.0},
                {'/traj_opti_node1/gd_type': 1},
            ],
          arguments = "-d /home/bugday/Projects/colcon_ws/src/grad_traj_optimization/rviz/traj.rviz"
        )
    ])
