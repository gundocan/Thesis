#!/usr/bin/env python


import rospy
import numpy as np
import ros_numpy
from numpy import transpose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

from geometry_msgs.msg import Twist
import time
import tf
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from copy import copy, deepcopy
from geometry_msgs.msg import PoseStamped
import geometry_msgs.msg
import tf2_ros
from docutils.nodes import sidebar
from operator import itemgetter, attrgetter
'''
glo al status
global sensor_list
global sensor_dicti
'''

class dwa():

    def __init__(self):
        rospy.init_node("dwa", anonymous=True)
        ############# Global variables #############

        self.w_curr = 0
        self.v_curr = 0
        self.pairs_list = []
        self.yaw =0
        self.status = "I'm done moving"
        self.odom = rospy.Subscriber("/odom", Odometry, self.pose_callback)  # odometry
        self.rviz_goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.destcallback)
        self.trajec_vis_pub = rospy.Publisher('/path_trajectory', Marker , queue_size=1)
        self.marker_pub = rospy.Publisher('/circle_publisher', MarkerArray, queue_size=300)
        self.odometry_frame = rospy.get_param("~odom_frame", "odom")  # get odom frame
        #self.baseFootprint_frame = rospy.get_param("~base_footprint_frame","base_footprint")  # get base_footprint frame
        self.baseScan_frame = rospy.get_param("~base_scan_frame", "base_scan")
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.wait_for_message("/move_base_simple/goal", PoseStamped)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.scan_data = rospy.Subscriber("/scan", LaserScan, self.scan_cb)  # listen to Scan
        self.max_vel = 2.0
        self.current_vel = 0
        self.radius = 0
        self.current_angvel = 0
        self.robot_diameter = 0.35
        self.r = 0.17
        self.laserdata = []
        self.angles = []
        self.obs_circles = []
        self.i = 0
        self.vel_msg = Twist()
        time.sleep(5.0)




    def target_direction(self, x, y ):

        direction = math.atan2((y - self.y_robot), abs(x - self.x_robot))
        #if direction < 0 and x - self.x_robot< 0:
        #    direction = -(3.14159265359 + direction)
        #if direction > 0 and x - self.x_robot < 0:
         #   direction = (3.14159265359 - direction)
        #print('mY second goal :', self.x_goal ,self.y_goal)
        #print('current velocity :', self.v_curr)
        #print('Goal Point :', self.y_goal, self.x_goal)
        #print('Goal DIRECTION :',direction -self.yaw, 'Degrees2::', math.degrees(direction -self.yaw))
        return direction - self.yaw

    def destcallback(self, goal_data):

        # Get Rviz 2D-nav goal x and y coordinates

        self.x_goal = goal_data.pose.position.x
        self.y_goal = goal_data.pose.position.y

        print('my location ', self.x_robot ,self.y_robot)
        return self.x_goal ,self.y_goal

    def scan_cb(self, scan_data):
        # get laser scan data
        self.i= self.i+1
        self.window_range =[]
        self.window_angle=[]
        self.window_angle_rad = []
        self.laserdata = scan_data.ranges
        rf1 = (sum(self.laserdata[0:10]) )/10 # avg distance
        rf2 = (sum(self.laserdata[10:20]) )/10
        rf3 = (sum(self.laserdata[20:30]) )/10
        rf4 = (sum(self.laserdata[30:40]) )/10
        rf5 = (sum(self.laserdata[40:50]) )/10
        rf6 = (sum(self.laserdata[50:60]) )/10
        rf7 = (sum(self.laserdata[60:70]) )/10
        rf8 = (sum(self.laserdata[70:80]) )/10
        rf9 = (sum(self.laserdata[80:90]) )/10
        rb1 = (sum(self.laserdata[90:100]) )/10
        rb2 = (sum(self.laserdata[100:110]) )/10
        rb3 = (sum(self.laserdata[110:120]) )/10
        rb4 = (sum(self.laserdata[120:130]) )/10
        rb5 = (sum(self.laserdata[130:140]) )/10
        rb6 = (sum(self.laserdata[140:150]) )/10
        rb7 = (sum(self.laserdata[150:160]) )/10
        rb8 = (sum(self.laserdata[160:170]) )/10
        rb9 = (sum(self.laserdata[170:180]) )/10
        lf1 = (sum(self.laserdata[180:190]) )/10
        lf2 = (sum(self.laserdata[190:200]) )/10
        lf3 = (sum(self.laserdata[200:210]) )/10
        lf4 = (sum(self.laserdata[210:220]) )/10
        lf5 = (sum(self.laserdata[220:230]) )/10
        lf6 = (sum(self.laserdata[230:240]) )/10
        lf7 = (sum(self.laserdata[240:250]) )/10
        lf8 = (sum(self.laserdata[250:260]) )/10
        lf9 = (sum(self.laserdata[260:270]) )/10
        lb1 = (sum(self.laserdata[270:280]) )/10
        lb2 = (sum(self.laserdata[280:290]) )/10
        lb3 = (sum(self.laserdata[290:300]) )/10
        lb4 = (sum(self.laserdata[300:310]) )/10
        lb5 = (sum(self.laserdata[310:320]) )/10
        lb6 = (sum(self.laserdata[320:330]) )/10
        lb7 = (sum(self.laserdata[330:340]) )/10
        lb8 = (sum(self.laserdata[340:350]) )/10
        lb9 = (sum(self.laserdata[350:360]) )/10
        self.window_range = [rf1,rf2,rf3,rf4,rf5,rf6,rf7,rf8,rf9,rb1,rb2,rb3,rb4,rb5,rb6,rb7,rb8,rb9,lf1,lf2,lf3,lf4,lf5,lf6,lf7,lf8,lf9,lb1,lb2,lb3,lb4,lb5,lb6,lb7,lb8,lb9]
        #self.window_angle= [5 , 15, 25, 35, 45,55,65,75,85,95,105,115,125,135,145,155,165,175,185,195,205,215,225,235,245,255,265,275,285,295,305,315,325,335,345,355]
        #self.window_angle = [85, 75 , 65, 55 ,45,35,25,15,5,355 ,345, 335, 325, 315, 305 , 295, 285, 275, 265,255,245,235,225,215,205,195,185,175,165, 155,145,135,125,115,105,95,85,75,65,55,45,35,25,15,5,]
        #self.window_angle = [ 355 ,345, 335, 325, 315, 305 , 295, 285, 275, 265,255,245,235,225,215,205,195,185,175,165, 155,145,135,125,115,105,95,85,75,65,55,45,35,25,15,5,85, 75 , 65, 55 ,45,35,25,15,5]
        self.window_angle = [95 , 105 , 115, 125, 135, 145, 155,165, 175, 185, 195, 205, 215 ,225, 235, 245, 255, 265, 275, 285, 295, 305, 315, 325, 335, 345, 355, 5, 15,25,35,45,55,65,75,85 ]
        #self.window_angle = [95 + self.yaw, 105+ self.yaw, 115+ self.yaw, 125+ self.yaw, 135+ self.yaw, 145+ self.yaw, 155+ self.yaw, 165+ self.yaw, 175+ self.yaw, 185+ self.yaw, 195+ self.yaw, 205+ self.yaw, 215+ self.yaw, 225+ self.yaw, 235+ self.yaw, 245+ self.yaw, 255+ self.yaw, 265+ self.yaw , 275+ self.yaw, 285+ self.yaw, 295+ self.yaw, 305+ self.yaw, 315+ self.yaw, 325+ self.yaw, 335+ self.yaw, 345+ self.yaw, 355+ self.yaw, 5+ self.yaw, 15+ self.yaw, 25+ self.yaw, 35+ self.yaw, 45+ self.yaw, 55+ self.yaw, 65+ self.yaw, 75+ self.yaw, 85+ self.yaw]

        for x in range(len(self.window_angle)) :
            rad = math.radians(self.window_angle[x])    #(self.window_angle[x]* 180)/ math.pi
            self.window_angle_rad.append(rad)
        print("Window range :", self.window_range[0])


        a=self.obs_circle(self.window_range,self.window_angle)
        b = self.vss()
        #print('window angle mixed list ::', a)
        self.circle_visiualization(a)
        c= self.get_intersections(a,b)
        d=self.ang_parameter(c)
        e =self.v_parameter(c)
        v,w,x,y =self.optimal_trajectory(c,e,d)
        self.velocity_command(v ,w)
        self.trajectory_visiualization(v,w ,x,y)


    def obs_circle(self, windows, angles):
        self.obs_circles = [ ]
        di = windows
        Lr = angles
        #print('Laser Range : ', di)
        #print('Angles: ', Lr)
        #print('mylocation ;',self.x_robot ,self.y_robot)
        r_plus = []
        Na = 36
        R = []

        try:   #transformation from base_footprint to odom frame
           transform_goal = self.tf_buffer.lookup_transform( self.odometry_frame,self.baseScan_frame  ,rospy.Time(), rospy.Duration(0.5))
           print("Transformation working")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
           rospy.logwarn("Cannot get the goal position! ")

        self.T = ros_numpy.numpify(transform_goal.transform)
        print('T', self.T)

        for x in range(len(windows)):
            r_plus.append(di[x] * ((Lr[x] * math.pi) / (360*Na)))
        #print('R plus', r_plus)
        for z in range(len(windows)):
            R = (r_plus[z] + (di[z] - 0.5))
            x_partial = math.sin(math.radians(Lr[z]))  * di[z]
            y_partial = math.cos(math.radians(Lr[z])) * di[z]


            y = self.y_robot + y_partial    #self.y_robot -y_partial
            x = self.x_robot + x_partial          #self.x_robot +x_partial
            z = 0
            arr = np.array([x,y,z,1])
            #arr.reshape(1,4)
            new_point =  self.T @ arr
            new_point = new_point/new_point[-1]
            print('NEWWW POINT :', new_point)
            x = new_point[0]
            y= new_point[1]
            print('x:', x)
        #    print('di[z]:', di[z], 'sin(lr(z):', math.sin(math.radians(Lr[z])),'<<Z', z,'; LR[z]:', Lr[z] )
            print('y_partial', y_partial , 'x_partial ', x_partial)
            self.obs_circles.append((x,y,R,di[z],Lr[z],z))
        print('Obstacle Parameter>> X:', self.obs_circles[0][0],'Y:', self.obs_circles[0][1], 'LASER RANGE ;', self.obs_circles[0][3], 'ANGLE:', self.obs_circles[0][4])

        return self.obs_circles





    def get_intersections(self, obstacle_circles ,robot_circles) :
        # circle 1: (x0, y0), radius r0
        # circle 2: (x1, y1), radius r1


            pairs= []
            ID_robot =[]

            non_collision_list = []
            for x in range(len(robot_circles)):
                ID_robot.append(x)
            #db_list =[g[3] for g in robot_circles]
            x0_list =[g[3] for g in robot_circles]
            y0_list = [g[4] for g in robot_circles]
            r0_list = [g[2] for g in robot_circles]
           # robot_ID_list = [g[ ] for g in robot_circles]
            x1_list = [z[0] for z in obstacle_circles]
            y1_list = [z[1] for z in obstacle_circles]
            r1_list = [z[2] for z in obstacle_circles]
            obs_ID_list = [z[5] for z in obstacle_circles]



            for i in range(len(x0_list)) :
                x0= x0_list[i]
                y0 = y0_list[i]
                r0 = r0_list[i]
                for z in range(len(x1_list)) :
                    x1 = x1_list[z]
                    y1 = y1_list[z]
                    r1 = r1_list[z]
                    d = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
                    #print('D :',d)
                    # non intersecting
                    if d > r0 + r1:
                        if ID_robot[i] not in non_collision_list:
                            non_collision_list.append(ID_robot[i])


                    # One circle within other( Velocity bigger)
                    if d < r1 - r0:
                        if ID_robot[i] not in non_collision_list:
                            non_collision_list.append(ID_robot[i])

                    # One circle within other( OBS bigger)
                    if d < r0 - r1 :
                        #print("The circles intersect at a single point")

                        pass
                    # coincident circles
                    if d == 0 and r0 == r1:
                        #print("The circles intersect at a single point")
                        pass
                    if d == r1 + r0 or d == r1 - r0:
                        #print("The circles intersect at a single point")
                        pass
                    '''
                    else:

                        a = (r0 ** 2 - r1 ** 2 + d ** 2) / (2 * d)
                        h = math.sqrt(r0 ** 2 - a ** 2)
                        x2 = x0 + a * (x1 - x0) / d
                        y2 = y0 + a * (y1 - y0) / d
                        x3 = x2 + h * (y1 - y0) / d
                        y3 = y2 - h * (x1 - x0) / d

                        x4 = x2 - h * (y1 - y0) / d
                        y4 = y2 + h * (x1 - x0) / d
                        distance = math.sqrt( ((x3-x4)**2)+((y3-y4)**2) )
                    '''
            #print('non_collision list ; ', non_collision_list)
            for x in range(len(non_collision_list)) :
                a = non_collision_list[x]
                pairs.append(self.pairs_list[a])
            #print(pairs)
            return pairs

    def velocity_command(self,v, w):

        self.vel_msg.angular.z = w/ 10 #0 #w
        self.vel_msg.linear.x = v/10  #0 #v
        self.v_curr = v
        if abs(self.x_robot - self.x_goal) < 0.09 and abs(self.y_robot - self.y_goal) < 0.09 :
            self.vel_msg.angular.z = 0.0
            self.vel_msg.linear.x = 0.0
            self.velocity_publisher.publish(self.vel_msg)

        else:
            self.velocity_publisher.publish(self.vel_msg)

        #print('My current speed is :',self.vel_msg)


    def pose_callback(self, pose_data):
        # Get robot pose
        self.x_robot = pose_data.pose.pose.position.x
        self.y_robot = pose_data.pose.pose.position.y
       # self.current_vel =pose_data.pose.pose.linear.x
       # self.current_angvel = pose_data.pose.pose.angular.z


        quaternion = (
        pose_data.pose.pose.orientation.x,
        pose_data.pose.pose.orientation.y,
        pose_data.pose.pose.orientation.z,
        pose_data.pose.pose.orientation.w)
        rpy = tf.transformations.euler_from_quaternion(quaternion)
        self.yaw = rpy[2]




    def optimal_trajectory(self , pair_list , v_score , w_score ):

         #v_clear =
         lamda_ang = 2
         lambda_vel = 2
         sum =[]
         order = []
         x_x = [ z[3] for z in pair_list]
         y_y =[ z[4] for z in pair_list]
         score_sorted = []
         final_points = []
         ordr = 0

         for x in range(len(v_score)):
             score = lamda_ang * w_score[x] + lambda_vel * v_score[x]
             order.append(x)
             sum.append((score , order[x] ,x_x[x], y_y[x] ))
         #print('Sum of all scores :', sum)
         score_sorted = sorted(sum,key=itemgetter(0), reverse=False)
         #print('scores are sorted list : ',score_sorted)
         #print('score list :', score_sorted)

         ordr = [g[1] for g in score_sorted] # order of element numbers with lowest general score
         #print('Order >>>', ordr)
         ordr2 =ordr[0]  # take the first element of the lowest general score list


         #print('Order2 <<<',ordr2) # ordr2 represents the index of the v,w pair in the pairs list
         #print('Pairlisrt order : v<',pair_list[ordr2][0],' w<', pair_list[ordr2][1],pair_list[ordr2][2],pair_list[ordr2][3])

         #print('Order : ' , ordr2)
         #print("pairrrss ::",pair_list[ordr2])
         #print("pairrrss222 ::", pair_list[ordr2][1])
         #print("Robot position :", self.x_robot, self.y_robot)
         #print('Robot YAW : ', self.yaw, 'Degrees::', math.degrees(self.yaw))
         #print('Selected Parameters :', pair_list[ordr2][0], pair_list[ordr2][1],pair_list[ordr2][2],pair_list[ordr2][3], self.x_robot ,self.y_robot)



         return    pair_list[ordr2][0], pair_list[ordr2][1] ,pair_list[ordr2][2],pair_list[ordr2][3]




    def v_parameter(self, pair_list):
        #print('Pairs_list 111 ==>>', pair_list)
        vel = [g[0] for g in pair_list]
        order = []
        comb = []
        comb_sorted = []
        score= []
        v_par = []
        fin = []

        v_last = []
        orders_sorted = []
        #print('vel :', vel)
        for x in range(len(vel)):
            order.append(x)
            comb.append((vel[x],order[x]))
        #print('combination :', comb)
        #print('Comb List(v_parameter) :::', comb)
        comb_sorted=sorted(comb , key=itemgetter(0), reverse=True)
        #print('Comb SORTED List(v_parameter >>>', comb)
        orders_sorted = [ z[1] for z in comb_sorted ]
        #print('order Sorted', orders_sorted)
        num_of_elements = len(vel)
        threshold = num_of_elements / 5
        #print('Threshold <<',threshold , ' Num of elements <<',num_of_elements)
        for x in range(len(vel)):
            if x <= int(threshold/2):
                score.append(0.00)
            if int(threshold/2) <= x <= int(threshold):
                score.append(0.20)
            if int(threshold) < x <= int(threshold*2):
                score.append(0.40)
            if int(threshold*2) < x<= int(threshold*3):
                score.append(0.60)
            if int(threshold*3) < x<= int(threshold*4):
                score.append(0.80)
            elif int(threshold*4) < x :
                score.append(1.0)
        #print('SCOREEEE Velocity :', score)

        for x in range(len(vel)):
            v_par.append(( orders_sorted[x] , score[x]))
        #print('v_par list ==', v_par)
        fin = sorted(v_par, key=itemgetter(0), reverse=False )

        best_v_candidates = [ z[1] for z in fin ]
        #print('finn: ', fin)
        #print('best candidates :', best_v_candidates)
        return best_v_candidates




    def circle_visiualization(self,pairs_list) :
        #print('Im working now')

        r = [g[2] for g in pairs_list]
        x0_list = [g[0] for g in pairs_list]
        y0_list = [g[1] for g in pairs_list]

        self.marker_array_msg = MarkerArray()

        for x in range(4):

            marker = Marker()
            marker.header.frame_id = "odom"  #'base_scan'
            marker.id = 100
            marker.type = marker.CYLINDER
            marker.action = marker.ADD
            marker.pose.position.x = x0_list[0]    #self.laserdata[0]     #x0_list[0]

            marker.pose.position.y = y0_list[0]   #self.y_robot         #0     #y0_list[0]
            marker.pose.position.z = 0
            marker.scale.x = r[0]

            marker.scale.y = r[0]
            marker.scale.z = 0.01
            marker.color.a = 0.9
            marker.color.r = 0.6
            marker.color.g = 0.4
            marker.color.b = 0.2
            self.marker_array_msg.markers.append(marker)
        self.marker_pub.publish(self.marker_array_msg)


    def trajectory_visiualization(self, v , w ,x,y):

        r = v/w


        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.id = 10
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.pose.position.x = self.x_robot     # -x     #self.x_robot
        marker.pose.position.y =  self.y_robot + r/2      # -y + r           # self.y_robot + r
        marker.pose.position.z = 0
        marker.scale.x = r
        marker.scale.y = r
        marker.scale.z = 0.01
        marker.color.a = 1
        marker.color.r = 0.0
        marker.color.g = 0.1
        marker.color.b = 0.9
        self.trajec_vis_pub.publish(marker)



    def ang_parameter(self, pair_list):
        #print("pair kist :", pair_list)
        #print('Pairs_list Ang ==>>', pair_list)
        angular = [g[1] for g in pair_list]
        #print('ANGULAR W :', angular)
        must_be_ang = self.target_direction(self.x_goal, self.y_goal)
        order = []
        comb = []
        comb_sorted = []
        orders_sorted = []
        score = []
        w_par = []
        w_last =[]
        fin = []
        for x in range(len(angular)):
            order.append(x)
            d=abs(angular[x]-must_be_ang)
            comb.append((angular[x],order[x],d))
        #print('Comb List (angular ):::', comb)
        comb_sorted = sorted(comb , key=itemgetter(2), reverse = False )
        #print('Comb SORTED List (angular) >>>', comb_sorted)
        orders_sorted = [z[1] for z in comb_sorted]
        #print('orders sorted :', orders_sorted)
        num_of_elements = len(angular)
        threshold = num_of_elements / 5
        for x in range(len(angular)):
            if x <= int(threshold / 2):
                score.append(0.00)
            if int(threshold / 2) <= x <= int(threshold):
                score.append(0.20)
            if int(threshold) < x <= int(threshold * 2):
                score.append(0.40)
            if int(threshold * 2) < x <= int(threshold * 3):
                score.append(0.60)
            if int(threshold * 3) < x <= int(threshold * 4):
                score.append(0.80)
            elif int(threshold * 4) < x:
                score.append(1.0)

        #print('SCOREEEE ANGULAR :', score)
        for x in range(len(angular)):
            w_par.append(( orders_sorted[x] , score[x]))
        #print('w_par :', w_par)
        fin = sorted(w_par, key=itemgetter(0), reverse=False )
        #print('FIN ;', fin)
        w_last = [z[1] for z in fin]

        #print('w_last: ', w_last)
        return w_last





    def vss(self) :
        #Initialize
        self.v_list =[]
        self.w_list = []
        self.mix_pairs = []
        self.pairs_list = []
        v_maxx = 0.8
        v_minn = 0.09
        w_maxx = 3.0
        w_minn = -3.0
        max_tr = 0.1 #3.2
        del_tr = 0.5
        max_rot = 0.6
        del_rot = 0.6

        #v_try = [0.9, 0.85, 0.80, 0.75, 0.70, 0.65, 0.60, 0.55, 0.50, 0.55, 0.40, 0.35, 0.30, 0, 25, 0.20, 0.15]
        v_try = [  0.35, 0.3, 0.25  ,0.20, 0.15]
        w_try =[ ]
        c=2.1
        for x in range( 40 ):
            c = c - 0.05
            w_try.append(round(c,2))
            w_try.append(round(-c,2))

        w_try.append(0.001)
        w_try.append(-0.001)
        #print('v_list :', v_try)
        #print('W_list :' , w_try)
        '''    
            
        if self.v_curr + max_tr >= v_maxx :
            v_max = v_maxx
        else :
            v_max = self.v_curr + max_tr
        if self.v_curr - del_tr <= v_minn:
            v_min = v_minn
        else :
            v_min = self.v_curr- del_tr
        if self.w_curr + max_rot >= w_maxx:
            w_max = w_maxx
        else:
            w_max = self.w_curr + max_rot
        if self.v_curr - del_rot <= w_minn:
            w_min = w_minn
        else:
            w_min = self.w_curr - del_rot
    
    
    
        #v_max = self.v_curr + max_tr
        #v_min = self.v_curr - del_tr
        #w_max = self.w_curr + max_rot
        #w_min = self.w_curr - del_rot
        dummy_list =[]
        #print('V_max :', v_max)
        #print('V_min :', v_min)
        #print('W_max :', w_max)
        #print('W_min :', w_min)
        num_example_v =3
        num_example_w = 18
        #Generate numbers in the range for pairing
        for i in range(num_example_v):
           v_generate =np.random.uniform(v_min, v_max )
           self.v_list.append(v_generate)
        for i in range(num_example_w):
            w_generate = np.random.uniform(w_min, w_max)
            self.w_list.append(w_generate)
        #Create v, w pairs that include more info
        '''


        #print('VSS v :', self.v_list, 'VSS w :', self.w_list)
        for x in range(len(v_try)):           #range(len(v_try)):           #range(num_example_v):

            for z in range(len(w_try)):                 #range(len(w_try))                #range(num_example_w):
                radius =  v_try[x] / w_try[z]          #v_try[x] / w_try[z]                  #self.v_list[x]/ self.w_list[z]
               # if num_example_w[z]  0.00:
                #   print

                #db = self.current_vel  - del_tr
                center_x = self.x_robot
               # center_x2 = self.x_robot - radius
                center_y = self.y_robot + radius/2 # radius

                self.pairs_list.append((v_try[x],w_try[z], radius,center_x,center_y ) )                        ##((self.v_list[x],self.w_list[z], radius,db ,center_x,center_y ) )
                #print('Vss :', self.pairs_list)
        return self.pairs_list





if __name__ == '__main__':
   d = dwa()
   rospy.spin()


