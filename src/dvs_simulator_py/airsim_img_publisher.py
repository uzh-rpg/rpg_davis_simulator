import rospy
import sys
import cv2
import time

# load airsim files
from AirSim.PythonClient import *


# ROS message imports
from std_msgs.msg import String
from sensor_msgs.msg import Image

# ROS package imports
from dvs_simulator_py import dataset_utils
from cv_bridge import CvBridge, CvBridgeError

def main():
    rospy.init_node('airsim_img_publisher', anonymous=False)
    airsim_server_ip = rospy.get_param('airsim_server_ip', '192.168.1.145')

    print('connecting to airsim...', airsim_server_ip)
    client = AirSimClient(airsim_server_ip)
    time.sleep(1) # give it time to connect

    showhelp_once = False # helper to not spam the log when airsim is misconfigured

    # Prepare publishers
    bridge = CvBridge()
    image_pub = rospy.Publisher("/image/gray", Image, queue_size=0)
    # rate = rospy.Rate(10) # 10hz  rate.sleep()


    while not rospy.is_shutdown():
        # get depth image
        # result = client.setImageTypeForCamera(0, AirSimImageType.Depth)
        # because this method returns std::vector<uint8>, msgpack decides to encode it as a string unfortunately.
        asim_in = client.getImageForCamera(0, AirSimImageType.Scene)

        if (asim_in == "\0"):
            if (not showhelp_once):
                showhelp_once = True
                rospy.loginfo("Please press '1' in the AirSim view to enable the Depth camera view")
        else:
            rawimg = np.fromstring(asim_in, np.int8)
            cv2gray = cv2.imdecode(rawimg, cv2.IMREAD_GRAYSCALE)
            img_msg = bridge.cv2_to_imgmsg(np.uint8(cv2gray * 255.0), 'mono8')
            img_msg.header.stamp = rospy.Time.now()
            image_pub.publish(img_msg)
            rospy.loginfo("processed image from airsim (shape: %s)", np.shape(cv2gray))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
