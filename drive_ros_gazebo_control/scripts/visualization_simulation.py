#!/usr/bin/env python2

# NOTE: This script must be executed using the Python 2 interpreter
# since ROS does not support Python 3.


from __future__ import print_function
if __debug__:
    import cProfile as profile
import cv2 as cv
import getopt
import numpy as np
import copy
import os.path

# NOTE: Handling failing imports of ROS since this script maybe used in
# a non-ROS environment.
try:
    # TODO: Import ROS packages.
    import roslib
    import rospkg
    import rospy
    from std_msgs.msg import String
    from std_msgs.msg import Float32MultiArray
    from itmoves_msgs.msg import Trajectory
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge, CvBridgeError
    pass
except:
    pass

import sys

""" 
        date : 31.05.2020
        developer: Fabian Klein
        visualization script for simulation
"""


def process_image(birdsview, frontview, x, y, paracoeff):
      
    ### birdsview trajectory visualization - kalman lane detection
    
    # Recast the x and y points into usable format for cv2.polylines()
    pts_left = np.array([np.transpose(np.vstack([(320/2)-y, 240-x]))])
    
    isClosed = False
    thickness = 2
    color_lightblue = (2,179, 228)
    birdsview = cv.polylines(birdsview, np.int_([pts_left]), isClosed, color_lightblue, thickness)

    ## birdsview trajectory visualization - parabel lane detection
    
    # Generate x and y values for plotting
    y_paralane = np.linspace(0, birdsview.shape[0], birdsview.shape[0])
    y_paralane = y_paralane[90:]
    x_paralane = paracoeff[0]*y_paralane**2 + paracoeff[1]*y_paralane + paracoeff[2]
    # Recast the x and y points into usable format for cv2.polylines()
    pts_left_paralane = np.array([np.transpose(np.vstack([x_paralane, y_paralane]))])

    color_springgreen = (0,255,127)
    birdsview = cv.polylines(birdsview, np.int_([pts_left_paralane]), isClosed, color_springgreen, thickness)

    
    ### frontview trajectory visualization - kalman lane detection
    
    # Recast the x and y points into usable format for cv2.fillPoly()
    pts_right = np.array([np.flipud(np.transpose(np.vstack([(320/2)-y+40, 240-x])))])
    pts = np.hstack((pts_left, pts_right))
    
    # Create an image to draw the lines on (first birdsview, which will be warped to front)
    birdsview_gray = cv.cvtColor(birdsview, cv.COLOR_BGR2GRAY)
    warp_zero = np.zeros_like(birdsview_gray).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
    # Draw the lane onto the warped blank image
    cv.fillPoly(color_warp, np.int_([pts]), color_lightblue)
        
    # Warp the blank back to original image space using inverse perspective matrix (H_inv)
    newwarp = cv.warpPerspective(color_warp, params['birdseye-H_inv'], (frontview.shape[1], frontview.shape[0])) 
    # Combine the result with the original image
    frontview_kalman = np.copy(frontview)
    frontview_kalman = cv.addWeighted(frontview_kalman, 1, newwarp, 0.3, 0)

    ### frontview trajectory visualization - parabel lane detection

    # Recast the x and y points into usable format for cv2.fillPoly()
    pts_right_paralane = np.array([np.flipud(np.transpose(np.vstack([x_paralane+40, y_paralane])))])
    pts = np.hstack((pts_left_paralane, pts_right_paralane))

    color_warp_paralane = np.dstack((warp_zero, warp_zero, warp_zero))
    cv.fillPoly(color_warp_paralane, np.int_([pts]), color_springgreen)
        
    # Warp the blank back to original image space using inverse perspective matrix (H_inv)
    newwarp = cv.warpPerspective(color_warp_paralane, params['birdseye-H_inv'], (frontview.shape[1], frontview.shape[0])) 
    # Combine the result with the original image
    frontview_parabel = np.copy(frontview)
    frontview_parabel = cv.addWeighted(frontview_parabel, 1, newwarp, 0.3, 0)

    return birdsview, frontview_kalman, frontview_parabel

class VisualizationModel:

    def __init__(self):
        # Variables for callbacks
        self.frontview = []
        self.birdsview = []
        self.trajectory = []
        self.paralane = []

        self.init_subscribers()
        self.init_publishers()

    def cb_frontview(self, rx):
        image_bridge = CvBridge()
        rx = image_bridge.imgmsg_to_cv2(rx, 'bgr8')
        self.frontview = rx
        self.frontview_new = True

    def cb_birdsview(self, rx):
        image_bridge = CvBridge()
        rx = image_bridge.imgmsg_to_cv2(rx, 'bgr8')
        self.birdsview = rx
        self.birdsview_new = True

    def cb_trajectory(self, rx):
        self.trajectory = rx
        self.trajectory_new = True

    def cb_paralane(self, rx):
        self.paralane = rx.data
        self.paralane_new = True

    def init_subscribers(self):
        self.sub_frontview = rospy.Subscriber('/camera/image_raw', Image, callback=self.cb_frontview)
        self.sub_birdsview = rospy.Subscriber('/Camera/Image/birdsview/obstacle', Image, callback=self.cb_birdsview)
        self.sub_trajectory = rospy.Subscriber("/lane_detection_kalman/trajectory", Trajectory, callback=self.cb_trajectory)
        self.sub_paralane = rospy.Subscriber("/camera/parabel/coefficients", Float32MultiArray, callback=self.cb_paralane)

    def init_publishers(self):
        self.image_publish_birdsview_visualization = rospy.Publisher("/visualization/birdsview_kalman_and_parabel_lanedetection", Image, queue_size=30)
        self.image_publish_frontview_visualization_kalman = rospy.Publisher("/visualization/frontview_lanedetection_kalman", Image, queue_size=30)
        self.image_publish_frontview_visualization_parabel = rospy.Publisher("/visualization/frontview_lanedetection_parabel", Image, queue_size=30)

    def run(self):
        while not rospy.is_shutdown():
            # Copy signals to avoid multi threading problems
            frontview = copy.copy(self.frontview)
            birdsview = copy.copy(self.birdsview)
            trajectory = copy.copy(self.trajectory)
            paralane = copy.copy(self.paralane)
            if trajectory and len(birdsview) > 0 and len(frontview) > 0:
               trajectory_x = np.asarray(trajectory.x)
               trajectory_y = np.asarray(trajectory.y)
               paralane = np.asarray(paralane)
               birdsview, frontview_kalman, frontview_parabel = process_image(birdsview, frontview, trajectory_x, trajectory_y, paralane)

               image_bridge = CvBridge()
               message = image_bridge.cv2_to_imgmsg(birdsview, "rgb8")
               message.header.stamp = rospy.Time.now()
               self.image_publish_birdsview_visualization.publish(message)

               image_bridge = CvBridge()
               message = image_bridge.cv2_to_imgmsg(frontview_kalman, "rgb8")
               message.header.stamp = rospy.Time.now()
               self.image_publish_frontview_visualization_kalman.publish(message)

               image_bridge = CvBridge()
               message = image_bridge.cv2_to_imgmsg(frontview_parabel, "rgb8")
               message.header.stamp = rospy.Time.now()
               self.image_publish_frontview_visualization_parabel.publish(message)
 
def parse_args():
    opts, args = getopt.getopt(sys.argv[1:], 'c:hp:Pa')
    args = rospy.myargv(argv=args)
    print(args)
    opts = dict(opts)

    if len(args) > 0:
        sys.stderr.write('Invalid command-line argument(s). Exiting.\n')
        sys.exit(1)

    if '-h' in opts:
        print('USAGE: {} -p <file>'.format(PROG_NAME),
              'OPTIONS:',
              '  -h            Print this help text and exit.',
              '  -p <str>      Path to parameters file.',
              sep='\n')
        sys.exit(0)

    if '-p' not in opts:
        sys.stderr.write('Argument -p missing to specify file-path of simulation camera calibration json file. Exiting.\n')
        sys.exit(1)

    return opts['-p']

def read_params(params_file_path):
    fs = cv.FileStorage(params_file_path, cv.FILE_STORAGE_FORMAT_JSON +
                        cv.FILE_STORAGE_READ, 'UTF-8')
    if not fs.isOpened():
        sys.stderr.write('Could not open parameters file. Exiting.\n')
        sys.exit(1)

    params = dict()

    # NOTE: Extracting nodes of type int.
    for id in ['birdseye-H',
               'birdseye-H_inv',]:
        node = fs.getNode(id)
        if not node.isMap():
            sys.stderr.write('Parameter file is malformed. Exiting.\n')
            sys.exit(1)

        params[id] = node.mat()

    return params

if __name__ == '__main__':
    params_file_path = parse_args()
    params = read_params(params_file_path)
    
    # Init ROS node
    rospy.init_node("visualization_itmoves")
    viz = VisualizationModel()
    viz.run()
    rospy.spin()



    



