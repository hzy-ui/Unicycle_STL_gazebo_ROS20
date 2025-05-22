#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity     #, ResetWorld
from std_srvs.srv import Empty              # ResetSimulation 实际上是 Empty 类型
from geometry_msgs.msg import Pose
#
from util import print_c, format_color_log
import os

class GazeboController(Node):
    def __init__(self):
        super().__init__('gazebo_controller')
        
        # 声明参数（与launch文件中的结构匹配）
        self.declare_parameters(
            namespace='',
            parameters=[
                ('pos_x', 0.0),
                ('pos_y', 0.0),
                ('goals', []),
                ('obstacles', []),
                ('is_avoiding_obstacle', 1),
                ('dist_goal', 0.2),
                ('dist_obs', 0.15)
            ]
        )
        
        # 1. 重置世界
        self.get_logger().info(format_color_log(f"RESETING world ...", color="red", style="bold"))
        self.reset_world()                     # Added, commented
        
        # 2. 从参数生成目标点和障碍物
        self.get_logger().info(format_color_log(f"SPAWNING target / obstacle locations ...", color="red", style="bold"))
        self.spawn_objects()

    def reset_world(self):
        """重置Gazebo世界"""
        self.get_logger().info("Resetting Gazebo world...")
        reset_client = self.create_client(Empty, '/reset_simulation')
        while not reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('reset_simulation service not available, waiting...')
        
        future = reset_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("World reset complete")

    def spawn_objects(self):
        """从参数生成目标点和障碍物"""
        # 获取参数值
        goals = self.get_parameter('goals').value
        obstacles = self.get_parameter('obstacles').value
        dist_goal = self.get_parameter('dist_goal').value
        dist_obs = self.get_parameter('dist_obs').value
        
        spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        while not spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('spawn_entity service not available, waiting...')
        self.get_logger().info(format_color_log(f"SPAWNING target / obstacle locations complete...", color="green", style="bold"))            

        # 生成目标点（绿色圆柱体）
        for i in range(0, len(goals), 5):
            x = goals[i]
            y = goals[i+1]
            self.get_logger().info(format_color_log(f"Adding Goal: {x}, {y}", color="green", style="italic"))
            self.spawn_cylinder(
                spawn_client,
                name=f"goal_{i//5}",
                x=x, y=y, z=0.05,
                radius=dist_goal,
                color='0 1 0 0.65'  # 绿色
            )

        # 生成障碍物（红色圆柱体）
        for i in range(0, len(obstacles), 5):
            x = obstacles[i]
            y = obstacles[i+1]
            radius = obstacles[i+2]
            self.get_logger().info(format_color_log(f"Adding Obstacle: {x}, {y}, {radius}", color="green", style="italic"))            
            self.spawn_cylinder(
                spawn_client,
                name=f"obstacle_{i//5}",
                x=x, y=y, z=0.05,
                radius=radius,
                color='1 0 0 0.65'  # 红色
            )

    def spawn_cylinder(self, client, name, x, y, z, radius, color):
        """生成单个圆柱体"""
        cylinder_sdf = f"""
        <?xml version="1.0"?>
        <sdf version="1.6">
          <model name="{name}">
            <static>true</static>
            <pose>{x} {y} {z} 0 0 0</pose>
            <link name="link">
              <visual name="visual">
                <geometry>
                  <cylinder>
                    <radius>{radius}</radius>
                    <length>0.5</length>
                  </cylinder>
                </geometry>
                <material>
                  <ambient>{color}</ambient>
                  <diffuse>{color}</diffuse>
                </material>
              </visual>
              <collision name="collision">
              <geometry>
                  <box>
                  <size>0 0 0</size>  <!-- 将碰撞体积的尺寸设置为零 -->
                  </box>
              </geometry>
              <surface>
                  <contact>
                  <collide_without_contact>1</collide_without_contact> <!-- 禁止发生物理接触 -->
                  </contact>
              </surface>
              </collision>
            </link>
          </model>
        </sdf>
        """
        
        req = SpawnEntity.Request()
        req.name = name
        req.xml = cylinder_sdf
        req.initial_pose = Pose()
        req.initial_pose.position.x = 0.        # float(x)      alternative pos_x
        req.initial_pose.position.y = 0.        # float(y)                  pos_y
        req.initial_pose.position.z = 0.        # float(z)
        
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None and future.result().success:
            self.get_logger().info(f"Spawned {name} at ({x}, {y})")
        else:
            self.get_logger().error(f"Failed to spawn {name} at ({x}, {y})")

def main(args=None):
    rclpy.init(args=args)
    controller = GazeboController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()