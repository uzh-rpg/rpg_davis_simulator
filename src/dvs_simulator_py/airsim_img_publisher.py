import rospy
import sys
import cv2
import time
import rospkg
import os.path
import glob

# load airsim files
from AirSim.PythonClient import *


# ROS message imports
from std_msgs.msg import String, Empty
from sensor_msgs.msg import Image

# ROS package imports
from dvs_simulator_py import dataset_utils
from cv_bridge import CvBridge, CvBridgeError


class DataSetFeeder:
    def __init__(self, pub, bridge):

        self.image_pub = pub
        self.bridge = bridge

        self.subscriber = rospy.Subscriber("/requests/nextimage",
            Empty, self.callback,  queue_size = None)

        dataset_name = rospy.get_param('airsim_dataset_name', 'imgs')

        rospack = rospkg.RosPack()
        package_dir = rospack.get_path('dvs_simulator_py')

        # Parse dataset
        dataset_dir = os.path.join(package_dir, 'datasets', 'airsim', dataset_name)

        if not os.path.exists(dataset_dir):
            rospy.logfatal('Could not find dataset {} in folder {}. Aborting.'.format(dataset_name, dataset_dir))
            sys.exit()
        else:
            rospy.loginfo('Loading frames from {}'.format(dataset_dir))

            self.dataset_frames = glob.glob(dataset_dir + '/*.png')
            self.dataset_size = len(self.dataset_frames)
            self.dataset_index = 0

            rospy.loginfo('Found {} frames to process.'.format(self.dataset_size))
            rospy.loginfo('Processing frames alphanumerically: (first: {} second: {} ...)'
                .format(os.path.basename(self.dataset_frames[0]),
                        os.path.basename(self.dataset_frames[1])))

    def callback(self, ros_data):    # Prepare publishers
        if (self.dataset_index < self.dataset_size):
            path = self.dataset_frames[self.dataset_index]
            rospy.loginfo('Publishing frame {}'.format(path))
            im = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            img_msg = self.bridge.cv2_to_imgmsg(np.uint8(im * 255.0), 'mono8')
            img_msg.header.stamp = rospy.Time.now()
            self.image_pub.publish(img_msg)
            rospy.loginfo("processed image from airsim (shape: %s)", np.shape(im))
            self.dataset_index = self.dataset_index + 1
        else:
            rospy.loginfo("no more images to publish()")


def main():
    rospy.init_node('airsim_img_publisher', anonymous=False)

    # values: dataset or stream
    origin = rospy.get_param('airsim_input_origin', 'dataset')

    # Prepare publishers
    bridge = CvBridge()
    image_pub = rospy.Publisher("/image/gray", Image, queue_size=0)

    if (origin == 'dataset'):
        dataset_feeder = DataSetFeeder(image_pub, bridge)

    elif (origin == 'stream'):
        airsim_server_ip = rospy.get_param('airsim_server_ip', '192.168.1.45')
        print('connecting to airsim...', airsim_server_ip)
        client = AirSimClient(airsim_server_ip)
        time.sleep(1) # give it time to connect
    else:
        rospy.logfatal('Parameter airsim_input_origin: origin {} does not exist'.format(origin))
        sys.exit()


    showhelp_once = False # helper to not spam the log when airsim is misconfigured
    if (origin == 'dataset'):
        dataset_feeder.callback('') # manually publish the first image
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print "Shutting down after KeyboardInterrupt"

    elif (origin == 'stream'):
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
