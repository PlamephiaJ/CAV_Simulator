from launch import LaunchDescription
from launch_ros.actions import Node
import math

def generate_launch_description():
    ld = LaunchDescription()
    '''
    --------------------------------------------CUSTOMIZED NODES--------------------------------------------
    '''
    '''
    Debug
    '''
    debug_node = Node(
        package='debug_cpp',
        executable='debug_node',
        name='debug_node'
    )
    
    '''
    Automatic emergency braking
    '''
    safety_node_car0 = Node(
        package='safety_node',
        executable='safety_node',
        name='safe_node',
        namespace='car0',
        parameters=[
            {'ittc': 1.0}
        ]
    )

    '''
    Wall follow: select wall_follow_side.
    '''
    # right wall following
    right_wall_follow_node_car0 = Node(
        package='wall_follow',
        executable='right_wall_follow_node',
        name='right_wall_follow_node',
        namespace='car0',
        parameters=[
            {'direction_a': -math.pi / 4},
            {'desired_distance_to_wall': 1.0},
            {'distance_look_ahead': 1.0},
            {'kp': 10.0},
            {'kd': 5.0},
            {"ki": 5.0}
        ]
    )

    # left wall following
    left_wall_follow_node_car0 = Node(
        package='wall_follow',
        executable='left_wall_follow_node',
        name='left_wall_follow_node',
        namespace='car0',
        parameters=[
            {'direction_a': math.pi / 4},
            {'desired_distance_to_wall': 1.0},
            {'distance_look_ahead': 1.0},
            {'kp': 10.0},
            {'kd': 5.0},
            {"ki": 5.0}
        ]
    )

    '''
    Follow the gap
    '''
    reactive_node_car0 = Node(
        package='gap_follow',
        executable='reactive_node',
        name='gap_follow',
        namespace='car0',
        parameters=[
            {'max_window': 2.4},
            {'vehicle_width': 0.6},
            {'disparity_threshold': 1.4}
        ]
    )

    '''
    Auxiliary
    '''
    # waypoint recorder
    # if you are using manual mode, please use run command instead of launch file:
    '''
    ros2 run auxiliary waypoint_recorder_sim --ros-args \
    -p mode:=manual \
    -p map_frame_name:=map \
    -p vehicle_base_frame_name:=ego_racecar/base_link \
    -p sample_interval_second:=1.0 \
    -p enable_speed:=True \
    -p manual_record_key:=q
    '''
    waypoint_recorder_sim_node_car0 = Node(
        package='auxiliary',
        executable='waypoint_recorder_sim',
        name='waypoint_recorder_sim_node',
        namespace='car0',
        parameters=[
            {'mode': 'auto'},
            {'map_frame_name': 'map'},
            {'sample_interval_second': 0.005},
            {'enable_speed': True}
        ]
    )

    # display waypoints in the rviz2
    waypoints_displayer_node_0 = Node(
        package='auxiliary',
        executable='waypoints_displayer_sim',
        name='waypoints_displayer_sim_node',
        namespace='car0',
        parameters=[
            {'map_frame_name': 'map'},
            # {'waypoint_file_path': '/sim_ws/waypoint/levine_blocked/waypoints_0.0s_Mon_Jul__1_10:55:05_2024.csv'},
            # {'waypoint_file_path': '/sim_ws/waypoint/Spielberg_map/anticlockwise_waypoints_0.0s_Fri_Jun_21_10:09:43_2024.csv'},
            {'waypoint_file_path': '/sim_ws/waypoint/straight_greyscale/straight.csv'},
            {'color_r': 0.0},
            {'color_g': 0.0},
            {'color_b': 1.0}
        ]
    )

    '''
    Pure pursuit
    '''
    pure_pursuit_sim_node_car0 = Node(
        package='pure_pursuit',
        executable='pure_pursuit_node_sim',
        name='pure_pursuit_sim_node',
        namespace='car0',
        parameters=[
            {'look_ahead_distance': 5.0},
            {'PID_P': 0.2},
            {'half_ring_width': 1.0},
            {'enable_speed_tracking': True}, # waypoint file must contain speed information if select True
            {'waypoint_file_path': '/sim_ws/waypoint/straight_greyscale/straight.csv'},
            {'is_emergency_brake_publisher': True},
            {'AEB_MODE': 'TTC'}, # 'DISTANCE' or 'TTC'
            {'AEB_DISTANCE_THRESHOLD': 10.0},
            {'AEB_TTC_THRESHOLD': 0.7},
            {'ENABLE_DELAY_CHECK': True}
        ]
    )

    '''
    RRT
    '''
    rrt_sim_node_car0 = Node(
        package='motion_planning',
        executable='rrt_node_sim',
        name='rrt_sim_node',
        namespace='car0',
        parameters = [
            {'MARGIN': 0.2},
            {'DISTANCE_GOAL_AHEAD': 3.5},
            {'SCAN_RANGE': 4.0},
            {'DETECTED_OBS_MARGIN': 0.22},
            {'MAX_RRT_ITERATIONS': 1200},
            {'MIN_RRT_ITERATIONS': 1000},
            {'STD': 1.5},
            {'STEP_SIZE': 0.3},
            {'NEAR_RANGE': 1.0},
            {'GOAL_TOLERANCE': 0.1},
            {'RRT_WAYPOINT_INTERVAL': 0.2},
            {'DISTANCE_LOOK_AHEAD': 0.4},
            {'PID_P': 0.25},
            {'odom_topic': '/odom'},
            {'map_topic': '/map'},
            {'scan_topic': '/scan'},
            {'drive_topic': '/drive'},
            {'waypoint_file_path': '/sim_ws/waypoint/Spielberg_map/anticlockwise_waypoints_0.0s_Fri_Jun_21_10:09:43_2024.csv'}
        ]
    )

    # ld.add_action(safety_node_car0)
    # ld.add_action(left_wall_follow_node_car0)
    # ld.add_action(right_wall_follow_node_car0)
    # ld.add_action(reactive_node_car0)
    # ld.add_action(waypoint_recorder_sim_node_car0)
    ld.add_action(waypoints_displayer_node_0)
    ld.add_action(pure_pursuit_sim_node_car0)
    # ld.add_action(rrt_sim_node_car0)

    return ld
