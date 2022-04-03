#!/usr/bin/python

# Estimates the  pose of a robot in a map based on the position of the AprilTag markers in the robot's camera field of view
# Broadcasts the transform of odom w.r.t. map to correct odometry drift and errors
#Author : Can Gundogdu
import math
from math import cos

from numpy import sin
from numpy.linalg import linalg
import csv
import rospy
from nav_msgs.msg import Odometry
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Point, Quaternion,PoseWithCovarianceStamped,PoseWithCovariance ,Twist,TwistStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion, translation_from_matrix, quaternion_from_matrix, compose_matrix, quaternion_matrix, rotation_matrix
import numpy as np

import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
import tf
## Global variables
nrTfRetrys = 1
retryTime = 0.05
rospy.init_node('robot_pose_broadcaster', log_level=rospy.INFO, anonymous=False)

# Initializes a tf2 listener
tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0)) #tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)


#belki değiştirebilirsin

with open('data_set.csv', mode='w') as dataset_file:
    data_writer = csv.writer(dataset_file, delimiter=',', quotechar='"')
    #fields = [0.0, -0.4 , 0.0 , 0.0, 0.0, 0.0, 0.0]
    #data_writer.writerow(fields)
    dataset_file.close()

Q =np.array([[0.01, 0],
           [0, 0.01]])


R = np.array([[1, 0, 0],
     [0, 1, 0],
     [0, 0, 0.5]])

ekf_r = np.array([0.0,0.0,0.0])
tag_detected = False
P = np.array([[0.1, 0, 0],
     [0, 0.1, 0],
     [0, 0, 0.1]])

last_odometry = np.array([-0.4 ,0.0 ,0.0])

last_position= np.array([-0.4 , 0.0, 0,0])

last_cmd_info = np.array([0.0 , 0.0])

apriltag_measurement =np.array([0.0,0.0,0.0])
#                 v     w
gold = np.array([0.0 , 0.0])

# Initializes a tf2 broadcaster for our map(parent)->odom(child) == odom w.r.t. map transform
br_odom_wrt_map = tf2_ros.TransformBroadcaster()

# Initializes an empty TransformStamped object for our map(parent)->odom(child) == odom w.r.t. map transform
ts_odom_wrt_map = TransformStamped()

r = rospy.Rate(10) # 10hz

# Initializes an empty PoseStamped object the pose of the robot_base w.r.t map
robot_pose = PoseStamped()
apriltag_pose = PoseStamped()
new_translation = []
new_rotation = []

velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

x_state = np.array([-0.4 , 0.0, 0,0, 0.0, 0.0])


def strip_forward_slash(frame_id):

    #to remove forward slash for tf2 to work

    if frame_id[0] == "/":
        new_frame_id = frame_id[1:]
    else:
        new_frame_id = frame_id
    return new_frame_id

# get the robot's base frame
if rospy.has_param("~base_frame"):
    # forward slash must be removed to work with tf2
    base_frame = strip_forward_slash(rospy.get_param("~base_frame"))
else:
    base_frame = "base_footprint"
    rospy.logwarn("base_footprint frame is set to default: base_footprint")

# get odom frame, the for example noisy odometry
if rospy.has_param("~odom_frame"):
    odom_frame = strip_forward_slash(rospy.get_param("~odom_frame"))
else:
    odom_frame = "odom"
    rospy.logwarn("odometry frame of reference is set to default: odom")

# get world_fixed_frame
if rospy.has_param("~world_fixed_frame"):
    world_fixed_frame = strip_forward_slash(rospy.get_param("~world_fixed_frame"))
else:
    world_fixed_frame = "map"
    rospy.logwarn("world_fixed_frame frame is set to default: map")

# get the camera frame
if rospy.has_param("~camera_frame"):
    camera_frame = strip_forward_slash(rospy.get_param("~camera_frame"))
else:
    camera_frame = "camera_rgb_optical_frame"
    rospy.logwarn("camera frame of reference is set to default: camera_rgb_optical_frame")

def main():
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, apriltag_callback, queue_size = 1)
    rospy.Subscriber("/odom",Odometry, odom_callback, queue_size=50)
    
    rospy.Subscriber('/cmd_vel', Twist, calculate_pos, queue_size= 1)
    rospy.loginfo("Started node to broadcast odom wrt map transform!")

    global prev_call , x_state, gold , tag_detected
    prev_call = rospy.Time.now()

    #move_in_circles()
    rospy.sleep(3)
    # get base w.r.t. odom transform
    while not rospy.is_shutdown():
        move_in_circles()

        try:
            # Look for the odom->base_footprint transform
            rospy.logdebug("Looking for the odom->base_footprint transform")
            tag_detected = False
            ts_base_wrt_odom = tf_buffer.lookup_transform(odom_frame, base_frame, rospy.Time(), rospy.Duration(1.0)) # wait 1s for transform to become available
            #x_state = calc_trajectory(x_state,gold, 1.2 )
            #rospy.loginfo("ts_base_wrt_odom: %s", ts_base_wrt_odom)
            # note: robot_pose (base_footprint wrt map) is calculated every time the subscriber callback is executed
            #broadcast_last_transform()
            write_csv()
            #move_in_circles()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rospy.logerr(ex)
        r.sleep()

def pose2poselist(poser):
    ''' Transforms a pose object into the form of a python list'''
    return [poser.pose.pose.position.x, poser.pose.pose.position.y, poser.pose.pose.position.z, poser.pose.pose.orientation.x, poser.pose.pose.orientation.y, poser.pose.pose.orientation.z, poser.pose.pose.orientation.w]

def transformPose(pose, sourceFrame, target_frame):
    '''
    Converts a pose represented as a list in the source_frame
    to a pose represented as a list in the target_frame
    '''
    _pose = PoseStamped()
    _pose.header.frame_id = sourceFrame
    if len(pose) == 6:
        pose.append(0)
        pose[3:7] = quaternion_from_euler(pose[3], pose[4], pose[5]).tolist()

    _pose.pose.position.x = pose[0]
    _pose.pose.position.y = pose[1]
    _pose.pose.position.z = pose[2]
    _pose.pose.orientation.x = pose[3]
    _pose.pose.orientation.y = pose[4]
    _pose.pose.orientation.z = pose[5]
    _pose.pose.orientation.w = pose[6]

    for i in range(nrTfRetrys):
        try:
            t = rospy.Time(0)
            _pose.header.stamp = t
            # converts a Pose object from its reference frame to a Pose object in the frame target_frame
            transform = tf_buffer.lookup_transform(target_frame, 
                                                   _pose.header.frame_id, #source frame is the current object's frame of reference
                                                   rospy.Time(0), #get the tf at first available time
                                                   rospy.Duration(1.0)) #wait for 1 second
            pose_transformed = tf2_geometry_msgs.do_transform_pose(_pose, transform)
            p = pose_transformed.pose.position
            o = pose_transformed.pose.orientation
            return [p.x, p.y, p.z, o.x, o.y, o.z, o.w]
        except (tf2_ros.LookupException) as e1:
            print("ERROR: LookupException!")
            rospy.logerr(e1)
            rospy.logwarn("No tf frame with name %s found. Check that the detected tag ID is part of the transforms that are being broadcasted by the static transform broadcaster.", target_frame)
            continue
        except (tf2_ros.ConnectivityException) as e2:
            rospy.logwarn(e2)
            rospy.logerr("ERROR: ConnectivityException!")
            continue
        except (tf2_ros.ExtrapolationException) as e3:
            rospy.logwarn(e3)
            rospy.logerr("ERROR: ExtrapolationException!")
            continue
        except Exception as e4:
            rospy.logwarn(e4)
            rospy.logerr("Unexpected error when transforming Pose")
        finally:
            rospy.sleep(retryTime)
    return None

def xyzquat_from_matrix(matrix):
    return translation_from_matrix(matrix).tolist() + quaternion_from_matrix(matrix).tolist()

def matrix_from_xyzquat(arg1, arg2=None):
    return matrix_from_xyzquat_np_array(arg1, arg2).tolist()

def matrix_from_xyzquat_np_array(arg1, arg2=None):
    if arg2 is not None:
        translate = arg1
        quaternion = arg2
    else:
        translate = arg1[0:3]
        quaternion = arg1[3:7]

    return np.dot(compose_matrix(translate=translate),quaternion_matrix(quaternion))

def invPoselist(poselist):
    return xyzquat_from_matrix(np.linalg.inv(matrix_from_xyzquat(poselist)))


def odom_callback(odom_data):
    global last_odometry
    odomx =odom_data.pose.pose.position.x
    
    odomy =odom_data.pose.pose.position.y
    quaternion = (
    odom_data.pose.pose.orientation.x,
    odom_data.pose.pose.orientation.y,
    odom_data.pose.pose.orientation.z,
    odom_data.pose.pose.orientation.w)
    rpy = tf.transformations.euler_from_quaternion(quaternion)
    odom_yaw = rpy[2]
    x_T= np.array([odomx ,odomy,odom_yaw])
    last_odometry[0]=  odomx
    last_odometry[1] = odomy
    last_odometry[2] = odom_yaw
    rospy.loginfo("My odom robot position %s;", odomx )
    rospy.loginfo("My odom robot position 2 %s;",odomy )
    rospy.loginfo("My odom robot position yaw %s;",odom_yaw )

def base_wrt_map_pose(pose=[0,0,0,0,0,0,1], child_frame_id='obj', parent_frame_id = world_fixed_frame, npub=1):
    '''
    Converts from a representation of a pose as a list to a TransformStamped object (translation and rotation (Quaternion) representation)
    Then keeps that as a TransformStamped object
    Note:
    In Rviz it will be shown as an arrow from the robot base (child) to the map (parent)
    In RQT it will be shown as an arrow from the map (parent) to the robot base (child)
    '''
    global robot_pose, apriltag_measurement

    if len(pose) == 7:
        quaternion = tuple(pose[3:7])
    elif len(pose) == 6:
        quaternion = quaternion_from_euler(*pose[3:6])
    else:
        rospy.logerr("Bad length of pose")
        return None

    position = tuple(pose[0:3])
    # Fill in PoseStamped object: stamps the transform with the current time
    robot_pose.header.stamp = rospy.Time.now()
    # Sets the frame ID of the transform to the map frame
    robot_pose.header.frame_id = parent_frame_id
    # Fill in coordinates
    if len(pose) == 6:
        pose.append(0)
        pose[3:7] = quaternion_from_euler(pose[3], pose[4], pose[5]).tolist()

    robot_pose.pose.position.x = pose[0]
    robot_pose.pose.position.y = pose[1]
    robot_pose.pose.position.z = 0 # fixate the z value of the robot base to avoid that it jumps up and down 0 , or  pose[2] could be used for the detected value
    robot_pose.pose.orientation.x = pose[3]
    robot_pose.pose.orientation.y = pose[4]
    robot_pose.pose.orientation.z = pose[5]
    robot_pose.pose.orientation.w = pose[6]
    quaternion = (
    robot_pose.pose.orientation.x,
    robot_pose.pose.orientation.y,
    robot_pose.pose.orientation.z,
    robot_pose.pose.orientation.w)
    rpye = tf.transformations.euler_from_quaternion(quaternion)
    apriltag_yaw = rpye[2]
    apriltag_measurement[0] =pose[0]

    apriltag_measurement[1] = pose[1]
    apriltag_measurement[2] = apriltag_yaw


    
    
    #rospy.loginfo("Position of the base_wrt_map %s;", robot_pose)
    #rospy.loginfo("Theta apriltag of the base_wrt_map %s;", apriltag_yaw)
def averagePose(pose_list):
    '''
    Calculates the average pose from a list of poses
    Position is the average of all estimated positions
    Orientation uses the orientation of the first detected marker
    '''
    avg_pose = []
    avg_pose.append(pose_list[0][0])							#np.mean([pose[0] for pose in pose_list]))
    avg_pose.append(pose_list[0][1])							#np.mean([pose[1] for pose in pose_list]))
    avg_pose.append(pose_list[0][2])						#np.mean([pose[2] for pose in pose_list]))
    # Use the orientation of the first detected marker
    # Maybe improvements could be made
    avg_pose.extend(pose_list[0][3:7])
    return avg_pose

def apriltag_callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    global num_tags,Q, tag_detected
    distance_tags = []
    if data.detections:
        counter = 0
        poselist_base_wrt_map = []
        for detection in data.detections:
            counter = counter + 1
            tag_id = detection.id[0]  # tag id
            #rospy.loginfo("Tag ID detected: %s \n", tag_id)
            child_frame_id = "tag_" + str(tag_id)
            tag_detected = True
            #rospy.loginfo("\n Child frame ID : %s \n", child_frame_id)

            # Convert the deteced tag Pose object to tag pose representation as a list, only for convinience

            poselist_tag_wrt_camera = pose2poselist(detection.pose)
            rospy.logdebug("poselist_tag_wrt_camera: \n %s \n", poselist_tag_wrt_camera)

            #rospy.loginfo("\n Distanceeee of the apriltag to the camera: %s \n", distance_of_tag)


            # Calculate transform of tag w.r.t. robot base (in Rviz arrow points from tag (child) to robot base(parent))
            poselist_tag_wrt_base = transformPose(poselist_tag_wrt_camera, camera_frame, base_frame)
            rospy.logdebug("transformPose(poselist_tag_wrt_camera, 'camera', 'robot_footprint'): \n %s \n", poselist_tag_wrt_base)
            #distance_of_tag = math.sqrt(poselist_tag_wrt_base[0] ** 2 - poselist_tag_wrt_base[1] ** 2)
          



            # Calculate transform of robot base w.r.t. tag (in Rviz arrow points from robot base (child) to tag(parent))
            poselist_base_wrt_tag = invPoselist(poselist_tag_wrt_base)
            rospy.logdebug("invPoselist( poselist_tag_wrt_base): \n %s \n", poselist_base_wrt_tag)

            # Calculate transform of robot base w.r.t. map (in Rviz arrow points from robot base (child) to map (parent)), returns pose of robot in the map coordinates
            poselist_base_wrt_map.append(transformPose(poselist_base_wrt_tag, child_frame_id, target_frame = world_fixed_frame))
            rospy.logdebug("transformPose(poselist_base_wrt_tag, sourceFrame = '%s', target_frame = 'map'): \n %s \n", child_frame_id, poselist_base_wrt_map[-1])
            #distance_tags.append(distance_of_tag)

        #for counter, robot_pose in enumerate(poselist_base_wrt_map):
        #    rospy.logdebug("\n Robot pose estimation nr. %s: %s \n",str(counter), robot_pose)
        rospy.loginfo("\n apriltagPOSEESSSS:  \n")
        num_tags = counter
        estimated_avg_pose = averagePose(poselist_base_wrt_map)

        rospy.logdebug("\n Robot's estimated avg. pose from all Apriltag  detected:\n %s \n", estimated_avg_pose)
        #rospy.loginfo("My robot position  %s", estimated_avg_pose)
        # Calculate transform of robot base w.r.t. map or pose of robot in the map coordinates

        base_wrt_map_pose(pose = estimated_avg_pose, child_frame_id = base_frame, parent_frame_id = world_fixed_frame)
        ekf_result =ekf()
       
        rospy.loginfo("EKFF RESULT %s", ekf_result)
        #apriltag_map_poses(poselist_base_wrt_map,child_frame_id = base_frame, parent_frame_id = world_fixed_frame )
        # ekf()
        map_to_odom_transform()

def convert_pose_inverse_transform(input_pose):
        """ Helper method to invert a transform (this is built into the tf C++ classes, but ommitted for Python) """
        translation = np.zeros((4,1))
        translation[0] = -input_pose.pose.position.x
        translation[1] = -input_pose.pose.position.y
        translation[2] = -input_pose.pose.position.z
        translation[3] = 1.0

        rotation = (input_pose.pose.orientation.x, input_pose.pose.orientation.y, input_pose.pose.orientation.z, input_pose.pose.orientation.w)
        euler_angle = euler_from_quaternion(rotation)
        rotation = np.transpose(rotation_matrix(euler_angle[2], [0,0,1]))       # the angle is a yaw
        transformed_translation = rotation.dot(translation)

        translation = (transformed_translation[0], transformed_translation[1], transformed_translation[2])
        rotation = quaternion_from_matrix(rotation)
        return (translation, rotation)

def convert_translation_rotation_to_pose(translation, rotation):
        """ Convert from representation of a pose as translation and rotation (Quaternion) tuples to a geometry_msgs/Pose message """
        return Pose(position=Point(x=translation[0],y=translation[1],z=translation[2]), orientation=Quaternion(x=rotation[0],y=rotation[1],z=rotation[2],w=rotation[3]))


def ekf():
    global P, Q, R ,last_position,num_tags,last_cmd_info , apriltag_measurement, last_cmd_info, last_odometry,ekf_r,x_state

    #rospy.loginfo("last position testing output 222222: %s", last_position)
    x , P_new = ekf_predict(last_position, last_cmd_info, P ,Q)   # uncomment for naive imp. last_cmd_info, P, Q)
    P = P_new
    last_position = x


    x, P_new = ekf_update(apriltag_measurement, last_odometry, last_position, P, R)
    P = P_new
    last_position = x

    ekf_r[0] = last_position[0]
    ekf_r[1] = last_position[1]
    ekf_r[2] = last_position[2]
    return x



def ekf_predict(x_t, u_t, P_t, Q):
    global  last_odometry 
    delta_s = u_t[0]
    delta_omega = u_t[1]
    rospy.loginfo("Ekf predict input check  ut: %s", u_t)
    rospy.loginfo("Delta S (input ut)) : %s", delta_s)
    rospy.loginfo("Delta Omega (input ut)) : %s",delta_omega)

    #delta_s = (u_t[1] + u_t[2]) / 2; # mean distance traveled by the wheels
    #delta_omega = (-u_t[1] + u_t[2]) / 0.160 #  distance between the two wheels
    # plot icin sil alttakini
    #x_t = last_odometry
    #x= np.array([u_t[0], u_t[1],u_t[2]])
    rospy.loginfo("xt before predict output: %s", x_t)
    #uncomment below
    x = np.array([x_t[0] + delta_s * cos(x_t[2] + delta_omega),  x_t[1] + delta_s * sin(x_t[2] + delta_omega), normalizeAngle(x_t[2] + delta_omega)])
    rospy.loginfo(">>xt after predict : %s", x)
    

    #x[0] = x_t[0] + delta_s * cos(x_t[2] + delta_omega / 2);
    #x[1] = x_t[1] + delta_s * sin(x_t[2] + delta_omega / 2);
    #x[2] = x_t[2] + delta_omega;

    F = np.array([[1, 0, - delta_s * sin(x_t[2] + delta_omega / 2)],
        [0, 1, delta_s * cos(x_t[2] + delta_omega / 2)],
        [0, 0, 1]])

    alpha_last = x_t[2];
    #d_l = u_t[1];
    #d_r = u_t[0];
    var1= delta_s * 2
    var2 =delta_omega * 0.160
    d_l = (var1 - var2)/2
    d_r = (delta_s * 2) - d_l

    G = np.array([[cos(alpha_last + (d_l- d_r) /4) / 2 + sin(alpha_last + (d_l- d_r) /4) * (d_l / 8 + d_r / 8),
          cos(alpha_last + (d_l- d_r) /4) / 2 - (sin(alpha_last + (d_l- d_r) /4) * (d_l / 2 + d_r / 2)) / 4],
         [sin(alpha_last + (d_l- d_r) /4) / 2 - (cos(alpha_last + (d_l- d_r) /4) * (d_l / 2 + d_r / 2)) / 4,
          sin(
              alpha_last + (d_l- d_r) /4) / 2 + cos(alpha_last + (d_l- d_r) /4) * (d_l / 8 + d_r / 8)],
         [-0.5, 0.5]])

    P = F @ P_t @ np.array(F).T + G @ Q @ np.array(G).T

    return x, P


def ekf_update(z,y,x_t, P_t, R):
      global num_tags
      
      #for i in range(num_tags):   #loop through the whole apriltag detections
      H=np.array([[1, 0, 0],
         [0, 1, 0],
         [0, 0, 1]])
      innovation = z - x_t     #z[i]-y
      S = H @ P_t @ np.array(H).T + R
      K = P_t @ np.array(H).T @ linalg.inv(S)
      x = y + K @ innovation
      I = np.eye(3)
      P = (I - K * H) * P_t
      return x, P


def calculate_pos(vel_data):

    #curr_call = rospy.Time.now()

    global last_cmd_info
    #global prev_call
    linear_velocity = vel_data.linear.x
    angular_velocity = vel_data.angular.z
    rospy.loginfo("linear velocity input  : %s", linear_velocity)
    rospy.loginfo("Angular velocity input  : %s", angular_velocity)
    duration = 0.24
    #duration = (curr_call - prev_call).to_sec()

    rospy.loginfo("duration input  : %s", duration)
    delta_linear = duration * linear_velocity
    delta_angular = duration * angular_velocity
    #prev_call = curr_call
    last_cmd_info = [delta_linear, delta_angular]
    rospy.loginfo("last cmd info  output: %s", last_cmd_info)

    return delta_linear, delta_angular




def map_to_odom_transform():
    """ This method constantly updates the offset of the map and
        odometry coordinate systems based on the latest results from
        the localization
    """
    global new_translation, new_rotation
    (translation, rotation) = convert_pose_inverse_transform(robot_pose)
    map_wrt_base_pose = PoseStamped(pose=convert_translation_rotation_to_pose(translation, rotation))
    map_wrt_base_pose.header.stamp = rospy.Time.now()
    map_wrt_base_pose.header.frame_id = base_frame
    # access/look-up the transform odom (target/new frame) wrt base(source/current frame):
    # tf_buffer.lookup_transform(target_frame, source_frame, query_time, timeout_secs)
    ts_odom_wrt_base = tf_buffer.lookup_transform(odom_frame, base_frame, rospy.Time(),
                                                  rospy.Duration(1.0))  # wait 1s for transform to become available
    map_wrt_odom_pose = tf2_geometry_msgs.do_transform_pose(map_wrt_base_pose, ts_odom_wrt_base)
    # invert map_wrt_odom_to get odom_wrt_map and store as tuple position, quaternion
    (new_translation, new_rotation) = convert_pose_inverse_transform(map_wrt_odom_pose)

"""
def control_input(vel_data):
    #This function listens to cmd_vel topic and tries to the locate the robot by using the inputs to the cmd_vel topic 

    curr_call = rospy.Time.now()

    global last_cmd_info
    global prev_call
    linear_velocity = vel_data.linear.x
    angular_velocity = vel_data.angular.z
    rospy.loginfo("linear velocity input  : %s", linear_velocity)
    rospy.loginfo("Angular velocity input  : %s", angular_velocity)
    #duration = 0.1
    duration = (curr_call-prev_call).to_sec()
    
    rospy.loginfo("duration input  : %s", duration)
    delta_linear = duration * linear_velocity
    delta_angular = duration * angular_velocity
    prev_call = curr_call
    last_cmd_info = [delta_linear, delta_angular]
    rospy.loginfo("last cmd info  output: %s", last_cmd_info)

    return delta_linear, delta_angular
"""

def move_in_circles():
   global gold ,x_state 
   rospy.loginfo("MOVING IN CIRCLES")
   vel_msg = Twist()
   vel_msg.linear.x = 0.1
   vel_msg.angular.z = 0.3
   velocity_publisher.publish(vel_msg)
   gold[0] = vel_msg.linear.x
   gold[1] = vel_msg.angular.z
   x_state[3] = vel_msg.linear.x
   x_state[4] = vel_msg.angular.z
   return
   

def motion(x, u, dt):
    # motion model
    # x = [x(m), y(m), theta(rad), v(m/s), omega(rad/s)]
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[2] += u[1] * dt *dt
    x[2] = normalizeAngle(x[2])
    x[3] = u[0]
    x[4] = u[1]

    return x



def calc_trajectory(xinit, v, Ptime):
    global gold, x_state
    dt = 0.1
    x = np.array(xinit)
    traj = np.array(x)  # many motion models stored per trajectory
    time = 0
    while time <= Ptime:
        # store each motion model along a trajectory
        x = motion(x_state , gold , dt)
        traj = np.vstack((traj, x))
        time += dt # next sample
    rospy.loginfo("trajectory: %s", traj[-1])     
    return traj[-1]

def normalizeAngle(angle):
    """
    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle 


def write_csv():
    global apriltag_measurement, last_odometry, ekf_r

    t = rospy.Time.now()
    with open('data_set.csv', mode='a') as dataset_file:
        data_writer = csv.writer(dataset_file, delimiter=',', quotechar='"')
        write_line =[t,last_odometry[0],last_odometry[1],last_odometry[2],apriltag_measurement[0],apriltag_measurement[1], apriltag_measurement[2],ekf_r[0], ekf_r[1],ekf_r[2]]
       # rospy.loginfo("...DATASET: %s", write_line)
        #rospy.loginfo("...DATASET ODOMETRY: %s", last_odometry[0])
        data_writer.writerow(write_line)
        #dataset_file.close()


def broadcast_last_transform():
        """ Making sure that always broadcasting the last
            map to odom transformation.  This is necessary so
            that  move_base can work properly. """
        # Sets the frame ID of the transform to the map frame
        ts_odom_wrt_map.header.frame_id = world_fixed_frame
        # Stamps the transform with the current time
        ts_odom_wrt_map.header.stamp = rospy.Time.now()
        # Sets the child frame ID to odom
        ts_odom_wrt_map.child_frame_id = odom_frame
        if new_translation:
            # Fill in coordinates
            list(new_translation) # convert an np array to an ordinary list
            ts_odom_wrt_map.transform.translation.x = new_translation[0][0]
            ts_odom_wrt_map.transform.translation.y = new_translation[1][0]
            ts_odom_wrt_map.transform.translation.z = new_translation[2][0]
            ts_odom_wrt_map.transform.rotation.x = new_rotation[0]
            ts_odom_wrt_map.transform.rotation.y = new_rotation[1]
            ts_odom_wrt_map.transform.rotation.z = new_rotation[2]
            ts_odom_wrt_map.transform.rotation.w = new_rotation[3]
            try:
                br_odom_wrt_map.sendTransform(ts_odom_wrt_map)
                rospy.loginfo("Broadcasted odom to odom_calc wrt map transform!")
            except Exception as exc:
                rospy.logwarn(exc)
                rospy.logerr("Unexpected error in broadcast of map to odom transformation")

if __name__=='__main__':
    main()
