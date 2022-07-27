#!/usr/bin/env python

import rospy
import message_filters
import rosbag

from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2

from datetime import date
from datetime import datetime


def make_img(image):

    img_data = Image()

    img_data.width = image.width
    img_data.height = image.height
    img_data.encoding = image.encoding
    img_data.data = image.data

    return img_data


def callback(image, joints, laser, cloud):
    global bag

    bag.write('img', image)
    bag.write('joints', joints)
    bag.write('laser', laser)
    bag.write('cloud', cloud)


global bag

if __name__ == '__main__':
    while True:

        cmd = str(input())
        if(cmd == 'indoor'):

            while not rospy.is_shutdown():

                today = date.today()
                now = datetime.now()

                time = now.strftime("%H-%M-%S")
                day = today.strftime("%m-%d-%y")

                rospy.init_node('data_collection')

                bag = rosbag.Bag(
                    '/media/sneh/Heracleia/Robot_Data/Indoor_Data/Data-' + day+'-Time-' + time + '.bag', 'w')

                img_sub = message_filters.Subscriber(
                    '/robot/front_rgbd_camera/rgb/image_raw', Image)
                joint_sub = message_filters.Subscriber(
                    '/robot/joint_states', JointState)
                laser_sub = message_filters.Subscriber(
                    '/robot/front_laser/filtered_scan', LaserScan)
                cloud_sub = message_filters.Subscriber(
                    '/robot/front_rgbd_camera/depth/points', PointCloud2)

                ts = message_filters.ApproximateTimeSynchronizer(
                    [img_sub, joint_sub, laser_sub, cloud_sub], 10, 1, allow_headerless=False)
                ts.registerCallback(callback)

                rospy.spin()
            bag.close()

        if(cmd == 'outdoor'):

            while not rospy.is_shutdown():

                today = date.today()
                now = datetime.now()

                time = now.strftime("%H-%M-%S")
                day = today.strftime("%m-%d-%y")

                rospy.init_node('data_collection')

                bag = rosbag.Bag(
                    '/media/sneh/Heracleia/Robot_Data/Outdoor_Data/Data-' + day+'-Time-' + time + '.bag', 'w')

                img_sub = message_filters.Subscriber(
                    '/robot/front_rgbd_camera/rgb/image_raw', Image)
                joint_sub = message_filters.Subscriber(
                    '/robot/joint_states', JointState)
                laser_sub = message_filters.Subscriber(
                    '/robot/front_laser/filtered_scan', LaserScan)

                ts = message_filters.ApproximateTimeSynchronizer(
                    [img_sub, joint_sub, laser_sub], 10, 1, allow_headerless=False)
                ts.registerCallback(callback)

                rospy.spin()
            bag.close()
