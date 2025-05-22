#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from turtlebot_stl_ros2.msg import STLInfo
from tf_transformations import euler_from_quaternion
import math
import sys, select, termios, tty
from cvxopt import solvers, matrix
import numpy as np

from util import print_c, format_color_log

class STLFinish(Node):
    def __init__(self):
        super().__init__('parameter_reader')

        # Declare parameters
        self.declare_parameter('pos_x', 0.0)
        self.declare_parameter('pos_y', 0.0)
        self.declare_parameter('goals', [])
        self.declare_parameter('obstacles', [])
        self.declare_parameter('is_avoiding_obstacle', 0)
        self.declare_parameter('dist_goal', 0.2)
        self.declare_parameter('dist_obs', 0.15)

        # Read parameters
        pos_x = self.get_parameter('pos_x').get_parameter_value().double_value
        pos_y = self.get_parameter('pos_y').get_parameter_value().double_value

        goals = self.get_parameter('goals').get_parameter_value().double_array_value
        obstacles = self.get_parameter('obstacles').get_parameter_value().double_array_value

        self.is_avoiding_obstacle = self.get_parameter('is_avoiding_obstacle').get_parameter_value().integer_value
        self.dist_goal = self.get_parameter('dist_goal').get_parameter_value().double_value
        self.dist_obs = self.get_parameter('dist_obs').get_parameter_value().double_value

        # Parse and log parameters
        self.get_logger().info(f"Vehicle Position -> x: {pos_x}, y: {pos_y}")

        # Parse goals
        parsed_goals = [
            {"x": goals[i], "y": goals[i+1], "start": goals[i+2], "end": goals[i+3], "expect": goals[i+4]}
            for i in range(0, len(goals), 5)
        ]
        # self.get_logger().info(f"Goals: {parsed_goals}")

        # Parse obstacles
        parsed_obstacles = [
            {"x": obstacles[i], "y": obstacles[i+1], "r": obstacles[i+2], "vx": obstacles[i+3], "vy": obstacles[i+4]}
            for i in range(0, len(obstacles), 5)
        ]
        # self.get_logger().info(f"Obstacles: {parsed_obstacles}")

        self.goallist   = self.get_goallist(parsed_goals)
        self.obslist    = self.get_obslist(parsed_obstacles, self.dist_obs)

        # Log other parameters
        self.get_logger().info(format_color_log(f"Is Avoiding Obstacles: {bool(self.is_avoiding_obstacle)}", color="green", style="italic"))
        self.get_logger().info(format_color_log(f"Distance to Goal: {self.dist_goal}", color="green", style="italic"))
        self.get_logger().info(format_color_log(f"Distance to Obstacle: {self.dist_obs}", color="green", style="italic"))


        # Subscribers and Publishers
        #   use robot pose instead
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.callback_read_current_position,                    # main function HERE
            1
        )
        self.navigation_input = self.create_publisher(Twist,   '/cmd_vel',     1)
        self.stl_information  = self.create_publisher(STLInfo, '/information', 10)

        # Internal Variables
        self.count = 0
        self.gamma = self.t_last = 0
        self.gamma_0 = self.gamma_inf = self.ls = 0
        self.init_x = self.init_y = 0                           # TODO
        self.init_time = self.get_clock().now().seconds_nanoseconds()[0]


        self.index = 0          # TODO

    def callback_read_current_position(self, data):                 # main function

        pose = self.get_pose(data)
        if pose is None:
            return
        x,y,yaw = pose

        self.index += 1
        if self.index % 50 == 0:
            self.get_logger().info(f"Position: x={x}, y={y}, yaw={yaw}")

        now_time = self.get_clock().now().seconds_nanoseconds()[0] - self.init_time         # in seconds
        # set the initial position as origin point
        if self.count == 0:
            self.first_time = now_time
            self.count      += 1
            self.init_x     = x
            self.init_y     = y
        now_time -= self.first_time

        x   -= self.init_x
        y   -= self.init_y
        goal_x,goal_y,goal_start,goal_expect = self.goal_get(now_time,self.goal_count,self.goallist)

        hx  = self.dist_goal - math.sqrt((x-goal_x)*(x-goal_x) + (y-goal_y)*(y-goal_y))

        # get new STL task
        if now_time - goal_start < 0.05:
            self.gamma   =  (self.gamma_0 - self.gamma_inf) * math.exp(-self.ls * ( now_time - self.t_last ) ) + self.gamma_inf

        self.gamma_0,self.gamma_inf,self.ls,self.gamma,self.t_last = self.gamma_update(now_time, goal_expect, hx, self.gamma, self.dist_goal, self.t_last)

        v   = self.v_control(x,y,goal_x,goal_y,now_time,self.t_last,self.ls,self.gamma_0,self.gamma_inf,hx,self.gamma,yaw,goal_start)
        w   = self.w_control(x,y,goal_x,goal_y,yaw,v,self.obslist)

        # if now_time <= 31.15:
        vel_msg = Twist()

        vel_msg.linear.x = v
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0

        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = w
        self.navigation_input.publish(vel_msg)


        info = STLInfo()
        info.hx_output      = hx
        info.gamma_output   = self.gamma
        info.v_output       = v
        info.w_output       = w
        self.stl_information.publish(info)
        # rate = rospy.Rate(50)
        # rate.sleep()


    def get_pose(self, msg):
        try:
            #
            pose_t = msg.pose.pose
            #
            x = pose_t.position.x
            y = pose_t.position.y
            qx = pose_t.orientation.x
            qy = pose_t.orientation.y
            qz = pose_t.orientation.z
            qw = pose_t.orientation.w
            quaternion = (qx, qy, qz, qw)
            euler = euler_from_quaternion(quaternion)
            yaw = euler[2]
            return float(x), float(y), float(yaw)
        except ValueError as e:
            # self.get_logger().warn("Model 'jackal' not found in /gazebo/model_states!")
            self.get_logger().warn(e)

    def get_goallist(self, parsed_goals):
        goallist = []
        for i in range(0, parsed_goals.__len__()):
            goal_x = parsed_goals[i]['x']
            goal_y = parsed_goals[i]['y']
            goal_start = parsed_goals[i]['start']
            goal_expect = parsed_goals[i]['expect']
            goal_end = parsed_goals[i]['end']
            goallist.extend([goal_x, goal_y, goal_start, goal_expect, goal_end])

        self.get_logger().info(format_color_log("GOAL list", color="cyan", style="bold"))
        for i in range(0, len(goallist), 5):
            self.get_logger().info(format_color_log(f"x: {goallist[i]}, y: {goallist[i + 1]} start: {goallist[i + 2]} expect: {goallist[i + 3]} end: {goallist[i + 4]}", color="cyan"))

        self.goal_count = parsed_goals.__len__()
        return goallist

    def get_obslist(self, parsed_obstacles, dist_obs):
        obslist = []
        for i in range(0, parsed_obstacles.__len__()):
            x_obs  = parsed_obstacles[i]['x']
            y_obs  = parsed_obstacles[i]['y']
            r_obs  = parsed_obstacles[i]['r'] + dist_obs               # TODO to check it out
            vx_obs = parsed_obstacles[i]['vx']
            vy_obs = parsed_obstacles[i]['vy']
            obslist.extend([x_obs, y_obs, r_obs, vx_obs, vy_obs])


        self.get_logger().info(format_color_log("OBSTACLE list", color="blue", style="bold"))
        for i in range(0, len(obslist), 5):
            self.get_logger().info(format_color_log(f"x: {obslist[i]}, y: {obslist[i + 1]} r: {obslist[i + 2]} vx: {obslist[i + 3]} vy: {obslist[i + 4]}", color="blue"))
        
        self.obs_count = parsed_obstacles.__len__()        
        return obslist


    def goal_get(self,t,count,goallist):
        goal_x=goal_y=goal_start=goal_expect=goal_end=temp_end = 0.0
        for i in range(1,count+1):
            temp_end    =   goallist[5*i-1]
            if t <= temp_end or i == count:
                goal_x      =   goallist[5*i-5]
                goal_y      =   goallist[5*i-4]
                goal_start  =   goallist[5*i-3]
                goal_expect =   goallist[5*i-2]
                goal_end    =   goallist[5*i-1]
                break

        return goal_x,goal_y,goal_start,goal_expect

    def gamma_build(self,hx,t_star,dist_goal):
        delta   = 0.12
        r       = 0.08
        ls_max  = 0.5

        gamma_0     = hx - delta
        gamma_inf   = dist_goal / 2
        if t_star > 0:
            ls = - math.log((r - gamma_inf)/(gamma_0 - gamma_inf)) / t_star
            if ls > ls_max:
                ls = ls_max
        else:
            ls = ls_max

        return gamma_0,gamma_inf,ls


    def gamma_update(self,t,goal_expect,hx,gamma,dist_goal,t_last):

        gamma_0,gamma_inf,ls = self.gamma_build(hx,goal_expect-t_last,dist_goal)
        gamma  = (gamma_0 - gamma_inf) * math.exp(-(t - t_last) * ls)  + gamma_inf

        if hx- gamma < 0 :
            t_last = t
            gamma_0,gamma_inf,ls = self.gamma_build(hx,goal_expect-t_last,dist_goal)
            gamma  = (gamma_0 - gamma_inf) * math.exp(-(t - t_last) * ls)  + gamma_inf

        return gamma_0,gamma_inf,ls,gamma,t_last



    def v_control(self,x,y,goal_x,goal_y,t,t_last,ls,gamma_0,gamma_inf,hx,gamma,theta,t_start):

        part_d_x   = -1 / math.sqrt((x-goal_x)*(x-goal_x) + (y-goal_y)*(y-goal_y))  * (x - goal_x)
        part_d_y   = -1 / math.sqrt((x-goal_x)*(x-goal_x) + (y-goal_y)*(y-goal_y))  * (y - goal_y)
        part_d_t   = (gamma_0-gamma_inf) * math.exp(-ls * (t-t_last)) * ls
        b          = hx - gamma

        k = 0.1
        p = matrix([[1.0,0.0],[0.0,1.0]])
        q = matrix([0.0,0.0])
        g = matrix([[-1.0*part_d_x],[-1.0*part_d_y]])
        h = matrix([k * b + part_d_t - 0.0009])
        solvers.options['show_progress'] = False
        result  = solvers.qp(p,q,-g,h,)
        x_opt   = list(result['x'])[0]
        y_opt   = list(result['x'])[1]

        theta_xd    = (goal_x - x) / math.sqrt((x-goal_x)*(x-goal_x) + (y-goal_y)*(y-goal_y) )
        theta_yd    = (goal_y - y) / math.sqrt((x-goal_x)*(x-goal_x) + (y-goal_y)*(y-goal_y) )
        v = (theta_xd * math.cos(theta) + theta_yd * math.sin(theta)) * ( math.sqrt(x_opt*x_opt + y_opt*y_opt ))

        return v
    '''

    '''
    def w_control(self,x,y,goal_x,goal_y,theta,v,obslist):
        ''' No obstacle avoidance '''
        kw      = 6
        bound   = 1

        d       = math.sqrt((x-goal_x)*(x-goal_x) + (y-goal_y)*(y-goal_y))
        theta_xd    = (goal_x - x) / d  # cos(vd)
        theta_yd    = (goal_y - y) / d  # sin(vd)

        w_max   =   v/d/2 + 1
        # yaw轴方向向量 [sin(theta) cos(theta)] 逆时针旋转90度的垂直向量 []
        # w   = kw * (-math.cos(theta)*theta_yd + math.sin(theta)*theta_xd )
        # #要求乘的是其垂直向量，由于为[theta_xd theta_yd] 其为单位向量，对应的垂直向量为[theta_yd -theta_xd]  //该向量为逆时针90度 [error]!!!!
        w   = kw * (theta_yd * math.cos(theta) - theta_xd * math.sin(theta))

        if w <= -bound:
            w   = -w_max
        elif w >= bound:
            w   = w_max
        else:
            w   = w*(w_max/bound)

        ''' obstacle avoidance '''
        if self.is_avoiding_obstacle == 1:
            h_obs       = []
            h_obs_1     = []
            a_obs       = []
            b_obs       = []
            sum_exp_hx =h_d1 = 0.0
            a  =   b    = 0.0
            b1 =   b2   = 0.0
            eta         = 0.5
            for i in range(1,self.obs_count+1):
                #  obslist 每五个一组： [x_obs y_obs r_obs vx_obs vy_obs]
                x_obs   = obslist[5*i-5]
                y_obs   = obslist[5*i-4]
                r_obs   = obslist[5*i-3]
                vx_obs  = obslist[5*i-2]
                vy_obs  = obslist[5*i-1]
                # rospy.loginfo("get the obs's information : %0.2f %0.2f %0.2f",x_obs,y_obs,r_obs)

                h_obs_now   = (x-x_obs)*(x-x_obs)+(y-y_obs)*(y-y_obs)-r_obs*r_obs
                h_obs.append(h_obs_now)
                h_obs_1_now = 2*(x - x_obs)*v*math.cos(theta) + 2*(y - y_obs)*v*math.sin(theta) - 2*(x - x_obs)*vx_obs - 2*(y - y_obs)*vy_obs
                h_obs_1.append(h_obs_1_now)
                a_obs_now   = (-2)*v*(x - x_obs)*math.sin(theta) + 2 *(y - y_obs)*v*math.cos(theta)
                a_obs.append(a_obs_now)
                b_obs_now   = 2*(v*math.cos(theta) - vx_obs)*(v*math.cos(theta) - vx_obs) + 2*(v*math.sin(theta) - vy_obs)*(v*math.sin(theta)  - vy_obs)
                b_obs.append(b_obs_now)

                sum_exp_hx  += math.exp(-eta * h_obs_now)

            for i in range(1,self.obs_count+1):
                h_d1    += -eta * math.exp(-eta*h_obs[i-1]) * h_obs_1[i-1]
                a       += -eta * math.exp(-eta*h_obs[i-1]) * a_obs[i-1]
                b1      += (eta * eta * math.exp(-eta*h_obs[i-1]) *h_obs_1[i-1] *h_obs_1[i-1] - eta * math.exp(-eta*h_obs[i-1]) * b_obs[i-1]) * sum_exp_hx
                b2      += -eta * math.exp(-eta*h_obs[i-1]) * h_obs_1[i-1]

            h_d1    /=  (-eta * sum_exp_hx)
            a       /=  (-eta * sum_exp_hx)
            b       =   (b1 - b2 * b2) / (-eta * sum_exp_hx * sum_exp_hx)

            k1 = 0.1
            k2 = 1.2

            p   = matrix([[1.0]])
            q   = matrix([[-1*w]])
            g   = matrix([[-a]])
            H   = matrix([[b + k1 * math.log(sum_exp_hx) / (-eta) + k2 * h_d1 ]])

            result = solvers.qp(p,q,g,H)
            w   = list(result['x'])[0]

        return w

def main(args=None):
    rclpy.init(args=args)
    node = STLFinish()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
