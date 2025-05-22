#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class ParamSetter(Node):
    def __init__(self):
        super().__init__('set_param')

        # vehicle_position
        self.declare_and_set_param('pos_x', 0)
        self.declare_and_set_param('pos_y', 0)

        # goal_position
        self.declare_and_set_param('goal_count', 3)
        
        self.declare_and_set_param('goal1_x', 4)
        self.declare_and_set_param('goal1_y', 3)
        self.declare_and_set_param('goal2_x', 0)
        self.declare_and_set_param('goal2_y', 4)
        self.declare_and_set_param('goal3_x', 4)
        self.declare_and_set_param('goal3_y', 0)

        # goal_time_limitation
        self.declare_and_set_param('goal1_start', 0)
        self.declare_and_set_param('goal1_end', 30)
        self.declare_and_set_param('goal1_expect', 27)

        self.declare_and_set_param('goal2_start', 30)
        self.declare_and_set_param('goal2_end', 50)
        self.declare_and_set_param('goal2_expect', 47)

        self.declare_and_set_param('goal3_start', 50)
        self.declare_and_set_param('goal3_end', 80)
        self.declare_and_set_param('goal3_expect', 77)

        # obstacles_position
        self.declare_and_set_param('obs_count', 3)

        self.declare_and_set_param('obs1_x', 3)
        self.declare_and_set_param('obs1_y', 2.5)
        self.declare_and_set_param('obs1_r', 0.375)
        self.declare_and_set_param('obs1_vx', 0)
        self.declare_and_set_param('obs1_vy', 0)

        self.declare_and_set_param('obs2_x', 1)
        self.declare_and_set_param('obs2_y', 4)
        self.declare_and_set_param('obs2_r', 0.25)
        self.declare_and_set_param('obs2_vx', 0)
        self.declare_and_set_param('obs2_vy', 0)

        self.declare_and_set_param('obs3_x', 2)
        self.declare_and_set_param('obs3_y', 1)
        self.declare_and_set_param('obs3_r', 0.5)
        self.declare_and_set_param('obs3_vx', 0)
        self.declare_and_set_param('obs3_vy', 0)

        # else params
        self.declare_and_set_param('dist_goal', 0.2)
        self.declare_and_set_param('dist_obs', 0.15)

        self.get_logger().info("All params are loaded!")

    def declare_and_set_param(self, name, value):
        self.declare_parameter(name, value)
        self.set_parameters([rclpy.parameter.Parameter(name, rclpy.Parameter.Type.DOUBLE if isinstance(value, float) else rclpy.Parameter.Type.INTEGER, value)])

def main(args=None):
    rclpy.init(args=args)
    node = ParamSetter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

