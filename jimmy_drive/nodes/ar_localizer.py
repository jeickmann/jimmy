#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from apriltags_ros.msg import AprilTagDetectionArray, AprilTagDetection
import tf2_geometry_msgs
from geometry_msgs.msg import Pose
import geometry_msgs
import tf
import tf2_kdl
import tf2_ros
from tf import transformations

#def matrix(transform):
#    trans = transformations.translation_matrix(transform.transform.translation.)
#    rot = transformations.quaternion_matrix(map_to_marker.transform.rotation)
#    return transformations.concatenate_matrices(trans, rot)

def tag_callback(detections):
    for detection in detections.detections:
        id = detection.id
        #rospy.loginfo("Found marker " + str(id))
        
        found_frame_id = "tag_" + str(id)
        position_frame_id = "tagposition_" + str(id)
        map_to_marker = tfBuffer.lookup_transform("map", position_frame_id, rospy.Time(0))
        odom_to_marker = tfBuffer.lookup_transform("odom", found_frame_id, rospy.Time(0))
        marker_to_odom = tfBuffer.lookup_transform(found_frame_id, "odom", rospy.Time(0))
        marker_to_base = tfBuffer.lookup_transform(found_frame_id, "base_link", rospy.Time(0))
        
        map2mark_kdl = tf2_kdl.tf2_kdl.transform_to_kdl(map_to_marker)
        mark2odom_kdl = tf2_kdl.tf2_kdl.transform_to_kdl(marker_to_odom)
        mark2base_kdl= tf2_kdl.tf2_kdl.transform_to_kdl(marker_to_base)
        map2odom_kdl = map2mark_kdl * mark2odom_kdl
        map2base_kdl = map2mark_kdl * mark2base_kdl
        
        global dataT
        global dataR
        global indexT
        global indexR
        dataT[indexT] = [map2odom_kdl.p.x(), map2odom_kdl.p.y(), 0]
        dataR[indexR] = tf.transformations.euler_from_quaternion(map2odom_kdl.M.GetQuaternion())

        indexT = (indexT + 1) % maxT
        indexR = (indexR + 1) % maxR

        #t.header.stamp = rospy.Time.now()
        #t.transform.translation.x = map2odom_kdl.p.x()
        #t.transform.translation.y = map2odom_kdl.p.y()         
        
        #(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w) = map2odom_kdl.M.GetQuaternion()
    pass

def init():
    #pub = rospy.Publisher('ar_pose', geometry_msgs, queue_size=10)
    rospy.init_node('ar_localization', anonymous=True)
    global tfBuffer
    global t
    global dataT
    global indexT
    global dataR
    global indexR
    global maxT
    global maxR

    maxT = 1
    maxR = 1

    initialT = [rospy.get_param("~initial_x", 0), rospy.get_param("~initial_y", 0), 0]
    dataT = []
    for i in range(maxT):
        dataT.append(initialT)
    indexT = 0
    dataR = []
    for i in range(maxR):
        dataR.append([0,0,0])
    indexR = 0
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "odom"
    t.transform.translation.x = rospy.get_param("~initial_x", 0)
    t.transform.translation.y = rospy.get_param("~initial_y", 0)

    t.transform.translation.z = 0
        
    (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w) = (0,0,0,1)
    br = tf2_ros.TransformBroadcaster()
    
    try:
        tfBuffer.lookup_transform("tagposition_0", "map", rospy.Time(0), rospy.Duration(10))
    except:
        rospy.logwarn("Unable to find position of Tag 0, no marker transforms published?")
    try:
        tfBuffer.lookup_transform("base_link", "odom", rospy.Time(0), rospy.Duration(10))
    except:
        rospy.logwarn("Unable to find odom to base_link, no odometry transforms published?")

    rospy.Subscriber("tag_detections", AprilTagDetectionArray, tag_callback)
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        t.header.stamp = rospy.Time.now()
        tAvg = [0,0,0]
        for i in dataT:
            tAvg[0] = tAvg[0] + i[0]
            tAvg[1] = tAvg[1] + i[1]
            tAvg[2] = tAvg[2] + i[2]
        
        tAvg[0] = tAvg[0]/len(dataT)
        tAvg[1] = tAvg[1]/len(dataT)
        tAvg[2] = tAvg[2]/len(dataT)

        t.transform.translation.x = tAvg[0]
        t.transform.translation.y = tAvg[1]
        t.transform.translation.z = tAvg[2]

        rAvg = [0,0,0]
        for i in dataR:
            #rAvg[0] = rAvg[0] + i[0]
            #rAvg[1] = rAvg[1] + i[1]
            rAvg[2] = rAvg[2] + i[2]
        
        rAvg[0] = rAvg[0]/len(dataR)
        rAvg[1] = rAvg[1]/len(dataR)
        rAvg[2] = rAvg[2]/len(dataR)

        quat = tf.transformations.quaternion_from_euler(rAvg[0], rAvg[1], rAvg[2])
        
        t.transform.translation.x = tAvg[0]
        t.transform.translation.y = tAvg[1]
        t.transform.translation.z = tAvg[2]

        (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w) = (quat[0],quat[1],quat[2],quat[3])
        
        br.sendTransform(t);
        rate.sleep()
        

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
