#!/usr/bin/env python3
#coding=utf-8
import rospy
import math
from cvxopt import solvers
from cvxopt import matrix
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from stl_control.msg import STL_info
import sys, select, termios, tty
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates


class stl_finish:
    def __init__(self):
        self.count      = 0
        self.goallist   = self.get_goallist()
        self.obslist    = self.get_obslist()
        self.goal_count = rospy.get_param("goal_count")
        self.dist_goal  = rospy.get_param("dist_goal")

        self.Pose = []
        self.current_pose       = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback_read_current_position, queue_size=1)
        self.navigation_input   = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.stl_information    = rospy.Publisher('/information',STL_info,queue_size=1)
        self.gamma      = self.t_last   = 0
        self.gamma_0    = self.gamma_inf = self.ls = 0
        self.init_x     = self.init_y   = 0
        self.init_time  = rospy.Time.now().to_sec()

    def get_pose(self,data):
        try:
            idx = data.name.index("jackal")
            pose = data.pose[idx]

            x = pose.position.x
            y = pose.position.y
            qx = pose.orientation.x
            qy = pose.orientation.y
            qz = pose.orientation.z
            qw = pose.orientation.w
            quaternion = (qx,qy,qz,qw)
            euler = euler_from_quaternion(quaternion)
            yaw = euler[2]
            return float(x), float(y), float(yaw)
        except ValueError:
            rospy.logwarn("Model 'jackal' not found in /gazebo/model_states!")



    def get_goallist(self):
        goallist=[]
        count = rospy.get_param("goal_count")
        for i in range (1,count+1):
            goal_x      =   rospy.get_param("goal" + str(i) + "_x")
            goal_y      =   rospy.get_param("goal" + str(i) + "_y")
            goal_start  =   rospy.get_param("goal" + str(i) + "_start")
            goal_expect =   rospy.get_param("goal" + str(i) + "_expect")
            goal_end    =   rospy.get_param("goal" + str(i) + "_end")
            templist    =   [goal_x,goal_y,goal_start,goal_expect,goal_end]
            goallist.extend(templist)
        return goallist

    def get_obslist(self):
        obslist=[]
        count = rospy.get_param("obs_count")
        for i in range(1,count+1):
            x_obs       = rospy.get_param("obs"+str(i)+"_x")
            y_obs   = rospy.get_param("obs"+str(i)+"_y")
            r_obs   = rospy.get_param("obs"+str(i)+"_r") + rospy.get_param("dist_obs",0)
            vx_obs  = rospy.get_param("obs"+str(i)+"_vx",0)
            vy_obs  = rospy.get_param("obs"+str(i)+"_vy",0)
            templist = [x_obs,y_obs,r_obs,vx_obs,vy_obs]
            obslist.extend(templist)
        return obslist

    def callback_read_current_position(self, data):
        if not rospy.is_shutdown():

            x,y,yaw= self.get_pose(data)

            now_time = rospy.Time.now().to_sec() - self.init_time
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


        info = STL_info()
        info.hx_output      = hx
        info.gamma_output   = self.gamma
        info.v_output       = v
        info.w_output       = w
        self.stl_information.publish(info)
        rate = rospy.Rate(50)
        rate.sleep()


    def goal_get(self,t,count,goallist):
        goal_x=goal_y=goal_start=goal_expect=goal_end=temp_end = 0.0
        for i in range(1,count+1):
            temp_end    =   goallist[5*i-1]
            if t <= temp_end or i == count:
                goal_x        =   goallist[5*i-5]
                goal_y        =   goallist[5*i-4]
                goal_start    =   goallist[5*i-3]
                goal_expect   =   goallist[5*i-2]
                # goal_end    =   goallist[5*i-1]
                break

        return goal_x, goal_y, goal_start, goal_expect

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
        if is_avoiding_obstacle == 1:
            count_obs   = rospy.get_param("obs_count",0)
            h_obs       = []
            h_obs_1     = []
            a_obs       = []
            b_obs       = []
            sum_exp_hx =h_d1 = 0.0
            a  =   b    = 0.0
            b1 =   b2   = 0.0
            eta         = 0.5
            for i in range(1,count_obs+1):
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

            for i in range(1,count_obs+1):
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


def getKey():
   tty.setraw(sys.stdin.fileno())
   select.select([sys.stdin], [], [], 0)
   key = sys.stdin.read(1)
   termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
   return key


def check_obstacle():
    get_start = 0
    global is_avoiding_obstacle
    while is_avoiding_obstacle == -1:
            rospy.loginfo("Are there any obstacles in the scene? [y/n] (Press 'q/Q' to quit)")
            temp   = getKey()
            if temp == "y" or temp == "Y":
                is_avoiding_obstacle = 1
            elif temp == "n" or temp == "N":
                is_avoiding_obstacle = 0
            elif temp == "q" or temp == "Q":  # exit
                rospy.loginfo("Exiting the program.")
                rospy.signal_shutdown("User requested shutdown.")
                return
            else:
                rospy.loginfo(f"Invalid input: {temp}")
    while get_start == 0 :
        rospy.loginfo("[START] please press any key to get start")
        temp   = getKey()
        rospy.loginfo("start!!!")
        get_start = 1

is_avoiding_obstacle = -1

if __name__ == "__main__":
    rospy.init_node("control")
    settings    = termios.tcgetattr(sys.stdin)
    mode = 0

    while not rospy.is_shutdown():

        check_obstacle()

        stl_finish()
        # rate.sleep()
        rospy.spin()

