#!/usr/bin/env python
#
#python publishrgbdvideo.py -c 
# see also https://gist.github.com/jensenb/7303362
import numpy as np
import cv2
import sys
import os
import argparse
import json
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
try:
    import xnencdec
except:
    pass
import ctypes
import struct

class xnVideoCapture:
    def __init__(self,filename):
        self.infile = open(filename,"rb")
        self.size = struct.unpack("ii",self.infile.read(8))
        self.decoded = np.empty(self.size,dtype=np.uint16)
        self.pbuffer = self.decoded.ctypes.data_as(ctypes.POINTER(ctypes.c_uint16))
        print "image depth size",self.size,filename
    def release(self):
        self.infile.close()

    def read(self):
        w = self.infile.read(4)
        if w == "":
            return 0, None
        else:
            size = struct.unpack("i",w)[0]
            rb = self.infile.read(size)
            xnencdec.doXnStreamUncompressDepth16ZWithEmbTablePTR(rb,self.pbuffer,self.decoded.size*2)
            return 1,self.decoded

class pngVideoCapture:
    def __init__(self,filename):
        self.infile = open(filename,"rb")
        self.size = struct.unpack("ii",self.infile.read(8))
    def release(self):
        self.infile.close()

    def read(self):
        w = self.infile.read(4)
        if w == "":
            return 0, None
        else:
            size = struct.unpack("i",w)[0]
            rb = self.infile.read(size)
            r = cv2.imdecode(np.fromstring(rb,dtype=np.uint8), cv2.IMREAD_UNCHANGED);            
            return 1,r

#        img[:,:,1] = np.bitwise_and(dimg,0xFF).astype(np.uint8)
#        img[:,:,0] = np.right_shift(dimg,8).astype(np.uint8)
#        img[:,:,2] = img[:,:,0]
def twincolor2depth(img):
    return np.bitwise_or(np.left_shift(img[:,:,0].astype(np.uint16),8),img[:,:,1].astype(np.uint16))
    
def main(a,b,CI,rate):
    print "running",a,b,CI,rate
    if CI is not None:
        if CI.endswith(".yaml"):
            camera_info = parse_calibration_yaml(CI)
        else:
            camera_info = parse_calibration_json(CI)
        print "camera_info is",camera_info


    image_pubC = rospy.Publisher("image",Image)
    if b is not None:
        image_pubD = rospy.Publisher("depth",Image)
    ci_pub = rospy.Publisher("camera_info",CameraInfo)
    bridge = CvBridge()
    capC = cv2.VideoCapture(a)
    if b is not None:
        if b.endswith(".xn16"):
            capD = xnVideoCapture(b)
        elif b.endswith(".xpng"):
            capD = pngVideoCapture(b)
        else:
            capD = cv2.VideoCapture(b)

    rate = rospy.Rate(args.rate) # 10hz
    while(not rospy.is_shutdown()):
        # Capture frame-by-frame
        retC, frameC = capC.read()
        if b is not None:
            retD, frameD = capD.read()

        if frameC is None:
            print "ended",retC
            break
        if b is not None and frameD is None:
            print "ended depth before color",retD
            break

        now = rospy.get_rostime()
        image_messageC = bridge.cv2_to_imgmsg(frameC, encoding="bgr8")
        image_messageC.header.stamp = now

        if b is not None:
            if frameD.dtype == np.uint16:
                dd = frameD
            else:
                dd = twincolor2depth(frameD)
            image_messageD = bridge.cv2_to_imgmsg(dd, encoding="16UC1")
            image_messageD.header.stamp = now
        image_pubC.publish(image_messageC)
        if b is not None:
            image_pubD.publish(image_messageD)
        if CI is not None:
            camera_info.header.stamp = now
            ci_pub.publish(camera_info)
        rate.sleep()

    # When everything done, release the capture
    capC.release()
    if b is not None:
        capD.release()

def parse_calibration_yaml(calib_file):
    with file(calib_file, 'rb') as f:
        params = yaml.load(f)

    cam_info = CameraInfo()
    cam_info.height = params['image_height']
    cam_info.width = params['image_width']
    cam_info.distortion_model = params['distortion_model']
    cam_info.K = params['camera_matrix']['data']
    cam_info.D = params['distortion_coefficients']['data']
    cam_info.R = params['rectification_matrix']['data']
    cam_info.P = params['projection_matrix']['data']

    return cam_info


def parse_calibration_json(calib_file):
    with file(calib_file, 'r') as f:
        params = json.load(f)

    cam_info = CameraInfo()
    cam_info.height = params['height']
    cam_info.width = params['width']
    cam_info.distortion_model = params['distortion_model']
    cam_info.K = params['K']
    cam_info.D = params['D']
    #cam_info.R = params['rectification_matrix']['data']
    cam_info.P = params['P']

    return cam_info


if __name__ == '__main__':
    rospy.init_node('image_converter', anonymous=True)

    parser = argparse.ArgumentParser(description='Plays two videos in sync')
    parser.add_argument('-c',"--color",required=True)
    parser.add_argument('-d',"--depth",default=None)
    parser.add_argument('-C',"--camera-info",default=None,required=False)
    parser.add_argument('-r',"--rate",default=30,type=int,help="data rate")

    args = parser.parse_args()
    if args.depth is None and args.color is not None:
        a,b = os.path.splitext(args.color)
        args.depth = a + "-D" + b
        if not os.path.isfile(args.depth):
            args.depth = a + "-D" + ".xn16"
            if not os.path.isfile(args.depth):
                args.depth = a + "-D" + ".xpng"
                if not os.path.isfile(args.depth):
                    print "not using",args.depth
                    args.depth = None
        if args.depth is not None:
            print "using",args.depth

    if args.camera_info is None and args.color is not None:
        a,b = os.path.splitext(args.color)
        args.camera_info = a + "-C.json"
        if not os.path.isfile(args.camera_info):
            print "not using",args.camera_info
            args.camera_info = None
        else:
            print "using",args.camera_info

    main(args.color,args.depth,args.camera_info,args.rate)