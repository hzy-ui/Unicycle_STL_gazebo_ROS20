import launch
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    # Convert dictionaries into flat arrays for ROS 2 parameters
    params = {
        # Vehicle position
        "pos_x": 0,
        "pos_y": 0,

        # Goals represented as flat arrays
        "goals": [
            4., 3., 0.,   40.,  37.,           # Goal 1: x, y, start, end, expect, remember to add ., or the data will NOT be loaded
            0., 4., 40.,  60.,  57.,           # Goal 2: x, y, start, end, expect
            4., 0., 60.,  90.,  87.,           # Goal 3: x, y, start, end, expect
            1., 1., 110., 130., 127.           # Goal 4: x, y, start, end, expect, IT SEEMS TO DIFFICULT TO REACH THIS POSITION
        ],

        # Obstacles represented as flat arrays
        "obstacles": [
            3., 2.5,    0.75,  0., 0.,         # Obstacle 1: x, y, r, vx, vy
            1., 4.,     0.25,   0., 0.,        # Obstacle 2
            2., 1.,     0.25,   0,  0.,        # Obstacle 3
            1., 1.75,   0.25,   0,  0.,        # Obstacle 4, should be set in the goal_4's location and block the paths along with the other obstacles
            4., 1.25,   1.15,   0,  0.,        # Obstacle 5, should be set in the goal_3's location
            2., 3.,     0.5,    0,  0.,        # Obstacle 6,
            0., 3.,     0.375,  0,  0.,        # Obstacle 6                        
        ],

        # Is avoiding Obstacles
        "is_avoiding_obstacle" : 1,         # 1: True, 0: False

        # Other parameters
        "dist_goal": 0.2,
        "dist_obs": 0.15,
    }

    # Log vehicle position
    log_vehicle_position = LogInfo(
        msg=f"Vehicle position -> x: {params['pos_x']}, y: {params['pos_y']}"
    )

    # Log goals
    log_goals = LogInfo(
        msg="Goals: " + ", ".join(
            [f"[x={params['goals'][i]}, y={params['goals'][i+1]}, start={params['goals'][i+2]}, end={params['goals'][i+3]}, expect={params['goals'][i+4]}]"
             for i in range(0, len(params['goals']), 5)]
        )
    )

    # Log obstacles
    log_obstacles = LogInfo(
        msg="Obstacles: " + ", ".join(
            [f"[x={params['obstacles'][i]}, y={params['obstacles'][i+1]}, r={params['obstacles'][i+2]}, vx={params['obstacles'][i+3]}, vy={params['obstacles'][i+4]}]"
             for i in range(0, len(params['obstacles']), 5)]
        )
    )  

    # Log other parameters
    log_other_params = [
        LogInfo(msg=f"Is avoiding obstacles: {params['is_avoiding_obstacle'] == 1}"),
        LogInfo(msg=f"Distance to goal: {params['dist_goal']}"),
        LogInfo(msg=f"Distance to obstacle: {params['dist_obs']}")
    ]

    return LaunchDescription([
        log_vehicle_position,
        log_goals,
        log_obstacles,
        *log_other_params,
        Node(
            package='turtlebot_stl_ros2',  # Replace with your package name
            executable='stl_control_car.py',  # Replace with your node executable
            name='stl_control_car',
            output='screen',
            parameters=[params]
        ),
        Node(
            package='turtlebot_stl_ros2',
            executable='gazebo_controller.py',
            name='gazebo_controller',
            output='screen',
            parameters=[params]
        )
    ])
