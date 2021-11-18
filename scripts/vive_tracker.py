#!/usr/bin/env python3
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped
import rospy
from std_msgs.msg import String
import triad_openvr
import time
import sys
import tf
from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_multiply
import numpy as np
import math
import pdb

def vive_tracker():
    rospy.init_node('vive_tracker_frame')
    broadcaster = { }
    publisher = { }
    listener = tf.TransformListener()
    rate = rospy.Rate(500) # 100hz]
    deviceCount = 0
    

    try:
      v = triad_openvr.triad_openvr()
    except Exception as ex:
      if (type(ex).__name__ == 'OpenVRError' and ex.args[0] == 'VRInitError_Init_HmdNotFoundPresenceFailed (error number 126)'):
        print('Cannot find the tracker.')
        print('Is SteamVR running?')
        print('Is the Vive Tracker turned on, connected, and paired with SteamVR?')
        print('Are the Lighthouse Base Stations powered and in view of the Tracker?\n\n')
      else:
        template = "An exception of type {0} occurred. Arguments:\n{1!r}"
        message = template.format(type(ex).__name__, ex.args)
        print(message)
      #print(ex.args)
      quit()
    
    v_buf={}
    for deviceName in v.devices:
        v_buf[deviceName] = {'position':{'x': 0.0, 'y': 0.0, 'z': 0.0}, 'orientation':{'x': 0.0, 'y': 0.0, 'z': 0.0, 'w':0.0}}
    #for devieName
    
    v.print_discovered_objects()
    P = np.mat([[1e-6, 0, 0], [0, 1e-6, 0], [0, 0, 1e-3]])
    p_cov = np.zeros((6, 6))
    # position covariance
    p_cov[0:2,0:2] = P[0:2,0:2]
    # orientation covariance for Yaw
    # x and Yaw
    p_cov[5,0] = p_cov[0,5] = P[2,0]
    # y and Yaw
    p_cov[5,1] = p_cov[1,5] = P[2,1]
    # Yaw and Yaw
    p_cov[5,5] = P[2,2]

    p_cov[0,:] =   [0.0000349162103240595,  -0.0000018202960310455,  -0.0000339898160507969,  -0.0000081126791170800,   0.0000001353045808767,   0.0000032202291901186]
    p_cov[1,:] =    [-0.0000018202960310455,   0.0000011910722363973,   0.0000020423436706964,   0.0000010961526869235,  -0.0000000333091396801,  -0.0000001408541892558]
    p_cov[2,:] =    [-0.0000339898160507969,   0.0000020423436706964,   0.0000341312090595451,   0.0000060715616751347,  -0.0000000237628610568,  -0.0000029217229365340]
    p_cov[3,:] =    [-0.0000081126791170800,   0.0000010961526869235,   0.0000060715616751347,   0.0000165832615351042,  -0.0000004759697840205,  -0.0000024486872043021]
    p_cov[4,:] =    [0.0000001353045808767,  -0.0000000333091396801,  -0.0000000237628610568,  -0.0000004759697840205,   0.0000003366392930324,  -0.0000000030521109214]
    p_cov[5,:] =    [0.0000032202291901186,  -0.0000001408541892558,  -0.0000029217229365340,  -0.0000024486872043021,  -0.0000000030521109214,   0.0000007445433570531]
    # rospy.loginfo(p_cov)
    
    #print("v devices",v.devices)
    while not rospy.is_shutdown():
        #if (deviceCount != v.get_device_count()):
        #    v = triad_openvr.triad_openvr()
        #    deviceCount = v.get_device_count()
        # For each Vive Device
        for deviceName in v.devices:
            #print(v_buf[deviceName].get_serial())
            if 'Null' in v.devices[deviceName].get_serial(): # To remove the null device.
              del v.devices[deviceName]
              break
            publish_name_str = v.devices[deviceName].get_serial().replace("-","_")
            
            # Broadcast the TF as a quaternion
            [x, y, z, qx, qy, qz, qw] = v.devices[deviceName].get_pose_quaternion()
            if "LHB" in v.devices[deviceName].get_serial():
                # Base station orientation calibration
                [qx, qy, qz, qw] = tf.transformations.quaternion_multiply([qx,qy,qz,qw], tf.transformations.quaternion_from_euler(0,1.5709,1.5709))
                # [qx, qy, qz, qw] = tf.transformations.quaternion_multiply([qx,qy,qz,qw], tf.transformations.quaternion_from_euler(0,0,0))
                

            #print(deviceName,v.devices[deviceName].get_pose_quaternion())
            time = rospy.Time.now()
            if deviceName not in broadcaster:
                broadcaster[deviceName] = tf.TransformBroadcaster()

            # Rotate Vive Trackers 180, so Z+ comes out of the top of the Tracker
            if "LHR" in v.devices[deviceName].get_serial():
                ### Tracker orientation calibration
                if x==0.0 and y==0.0 and z==0.0:
                    [x,y,z] = [v_buf[deviceName]['position']['x'],v_buf[deviceName]['position']['y'],v_buf[deviceName]['position']['z']]
                    [qx,qy,qz,qw] = [v_buf[deviceName]['orientation']['x'],v_buf[deviceName]['orientation']['y'],v_buf[deviceName]['orientation']['z'],v_buf[deviceName]['orientation']['w']]
                else:
                    [qx, qy, qz, qw] = tf.transformations.quaternion_multiply([qx, qy, qz, qw], tf.transformations.quaternion_from_euler(3.141592,0,1.5709))
                    v_buf[deviceName]['position']['x'] = x
                    v_buf[deviceName]['position']['y'] = y
                    v_buf[deviceName]['position']['z'] = z
                    v_buf[deviceName]['orientation']['x'] = qx
                    v_buf[deviceName]['orientation']['y'] = qy
                    v_buf[deviceName]['orientation']['z'] = qz
                    v_buf[deviceName]['orientation']['w'] = qw
                    
                #print(x,y,z,tf.transformations.euler_from_quaternion([qx, qy, qz, qw]))
                """
                T_s = tf.transformations.euler_matrix(0,0,0)
                T_b = tf.transformations.quaternion_matrix([qx,qy,qz,qw])
                T_b[0:3,3] = [x,y,z]
                T = T_s@T_b
                [x,y,z] = T[0:3,3]
                [qx,qy,qz,qw] = tf.transformations.quaternion_from_matrix(T)
                """
                #[qx, qy, qz, qw] = tf.transformations.quaternion_multiply([qx, qy, qz, qw], tf.transformations.quaternion_from_euler(0,0,0))
                """
                Ts_ = tf.transformations.inverse_matrix(tf.transformations.euler_matrix(0,1.5709,1.5709))
                Tb = tf.transformations.quaternion_matrix([qx,qy,qz,qw])
                Tb[0:3,3] = [x,y,x]
                T = Ts_@Tb
                [x,y,z]=T[0:3,3]
                [qx,qy,qz,qw] = tf.transformations.quaternion_from_matrix(T)
                """
                
            broadcaster[deviceName].sendTransform((x,y,z),
                            (qx,qy,qz,qw),
                            time,
                            publish_name_str,
                            "vive_world")

            # Publish a topic as euler angles
            [x,y,z,roll,pitch,yaw] = v.devices[deviceName].get_pose_euler()
            #print(x,y,z,roll,pitch,yaw)
            y_rot = math.radians(pitch)
            
            if "LHB" in v.devices[deviceName].get_serial():
                # Base station orientation calibration
                quat_tmp = tf.transformations.quaternion_multiply(tf.transformations.quaternion_from_euler(roll,pitch,yaw), tf.transformations.quaternion_from_euler(0,1.5709,1.5709))
                [roll,pitch,yaw] = tf.transformations.euler_from_quaternion(quat_tmp)                
            if "LHR" in v.devices[deviceName].get_serial():
                ### Tracker orientation calibration
                if x==0.0 and y==0.0 and z==0.0:
                    [x,y,z] = [v_buf[deviceName]['position']['x'],v_buf[deviceName]['position']['y'],v_buf[deviceName]['position']['z']]
                    [qx,qy,qz,qw] = [v_buf[deviceName]['orientation']['x'],v_buf[deviceName]['orientation']['y'],v_buf[deviceName]['orientation']['z'],v_buf[deviceName]['orientation']['w']]
                else:
                    rotmat=tf.transformations.inverse_matrix(tf.transformations.euler_matrix(0,1.5709,1.5709))
                    #print(rotmat)
                    #quat_inv = tf.transformations.quaternion_inverse([qx,qy,qz,qw])
                    quat_inv = tf.transformations.quaternion_from_matrix(rotmat)
                    quat_tmp = tf.transformations.quaternion_multiply(tf.transformations.quaternion_from_euler(roll,pitch,yaw), tf.transformations.quaternion_from_euler(3.141592,0,1.5709))
                    quat_trc = tf.transformations.quaternion_multiply(quat_inv,quat_tmp)
                    [roll,pitch,yaw] = tf.transformations.euler_from_quaternion(quat_trc)
                    [qx,qy,qz,qw] = quat_trc
                    #print([x,y,z])
                    #print(tf.transformations.euler_matrix(roll,pitch,yaw)[0:3,0:3])
                    [x,y,z] = [-z,-x,y]



                    # x+=0; y+=-1.352; 
                    z+=0.8


                    v_buf[deviceName]['position']['x'] = x
                    v_buf[deviceName]['position']['y'] = y
                    v_buf[deviceName]['position']['z'] = z
                    v_buf[deviceName]['orientation']['x'] = quat_trc[0]
                    v_buf[deviceName]['orientation']['y'] = quat_trc[1]
                    v_buf[deviceName]['orientation']['z'] = quat_trc[2]
                    v_buf[deviceName]['orientation']['w'] = quat_trc[3]
                    
                #print([x,y,z])
            if deviceName not in publisher:
                publisher[deviceName] = rospy.Publisher(publish_name_str, String, queue_size=10)


            publisher[deviceName].publish('  X: ' + str(x) + '  Y: ' + str(y) + '  Z: ' + str(z) + '  Roll: ' + str(roll) + '  Pitch: ' + str(pitch) + '  Yaw: ' + str(yaw))
            
            # In the case of the base station, it publish the own position and Euler angles
            # Otherwise, Odometry, pose, etc. are published w.r.t. Tracker
            if "reference" not in deviceName:
                if deviceName + "_odom" not in publisher:
                    publisher["ring_odom"] = rospy.Publisher("ring_odom", Odometry, queue_size=50)
                    publisher[deviceName + "_odom"] = rospy.Publisher(publish_name_str + "_odom", Odometry, queue_size=50)
                    publisher[deviceName + "_inverted_odom"] = rospy.Publisher(publish_name_str + "_inverted_odom", Odometry, queue_size=50)
                    publisher[deviceName + "_pose"] = rospy.Publisher(publish_name_str + "_pose",PoseWithCovarianceStamped,queue_size=10)
                # next, we'll publish the odometry message over ROS
                
                odom = Odometry()
                odom.header.stamp = time
                odom.header.frame_id = "vive_world"
                # set the position
                odom.pose.pose = Pose(Point(x, y, z), Quaternion(qx,qy,qz,qw))

                inv_odom = Odometry()
                inv_odom.header.stamp = time
                inv_odom.header.frame_id = "vive_world"
                # set the position
                [inv_qx, inv_qy, inv_qz, inv_qw] = tf.transformations.quaternion_multiply([qx, qy, qz, qw], [-0.707, 0.707, 0, 0])
                q = Quaternion(inv_qx,inv_qy,inv_qz,inv_qw)
                # pdb.set_trace()
                # q_new = q * q_inv
                inv_odom.pose.pose = Pose(Point(x, y+0.05, z), q)

                # set the velocity
                odom.child_frame_id = "ak1_base_link"
                [vx, vy, vz, v_roll, v_pitch, v_yaw] = v.devices[deviceName].get_velocities()
                
                if "LHR" in v.devices[deviceName].get_serial():
                    ### Tracker orientation calibration
                    
                    [v_roll, v_pitch, v_yaw] = [-v_yaw, -v_roll, v_pitch]
                    #print([x,y,z])
                    #print(tf.transformations.euler_matrix(roll,pitch,yaw)[0:3,0:3])
                    [vx,vy,vz] = [-vz,-vx,vy]
                
                
                odom.twist.twist = Twist(Vector3(vx, vy, vz), Vector3(v_roll, v_pitch, v_yaw))
                # This is all wrong but close enough for now
                # np.mat([1[1e-6, 0, 0], [0, 1e-6, 0], [0, 0, 1e-3]])
                odom.pose.covariance = tuple(p_cov.ravel().tolist())
                odom.twist.covariance = tuple(p_cov.ravel().tolist())
                # rospy.loginfo(p_cov)

                # Create a pose with covariance stamped topic
                pose = PoseWithCovarianceStamped()
                pose.pose.pose = Pose(Point(x, y, z), Quaternion(qx,qy,qz,qw))
                pose.pose.covariance = tuple(p_cov.ravel().tolist())
                if not (x == 0.0 and y == 0.0 and z == 0.0 and vx == 0.0 and vy == 0.0 and vz == 0.0 and v_roll == 0.0 and v_pitch == 0.0 and v_yaw == 0.0):
                    # publish the message
                    publisher[deviceName + "_odom"].publish(odom)
                    publisher[deviceName + "_inverted_odom"].publish(inv_odom)
                    publisher[deviceName + "_pose"].publish(pose)
                    # Publish the ring position
                    try:
                        (ring_trans,ring_rot) = listener.lookupTransform("vive_world", "ring" , time)
                        odom.pose.pose = Pose(Point(ring_trans[0],ring_trans[1],ring_trans[2]), Quaternion(qx, qy, qz, qw))
                        publisher["ring_odom"].publish(odom)
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        continue

        rate.sleep()


if __name__ == '__main__':
    try:
        vive_tracker()
    except rospy.ROSInterruptException:
        pass
