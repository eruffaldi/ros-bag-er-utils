#!/usr/bin/env python
#
from __future__ import division
import rosbag, rospy, numpy as np
import sys, os, cv2, glob
from itertools import izip, repeat
import argparse
import signal
import timesync
import json
import yaml
import message_filters
from sensor_msgs.msg import Image

   
def msg2json(msg):
   ''' Convert a ROS message to JSON format'''
   y = yaml.load(str(msg))
   return json.dumps(y,indent=4)

dostop=False
def signal_handler(signal, frame):
    global dostop
    print('You pressed Ctrl+C!')
    if dostop:
        sys.exit(0)
    dostop=True
signal.signal(signal.SIGINT, signal_handler)


# try to find cv_bridge:
try:
    from cv_bridge import CvBridge
except ImportError:
    # assume we are on an older ROS version, and try loading the dummy manifest
    # to see if that fixes the import error
    try:
        import roslib; roslib.load_manifest("bag2video")
        from cv_bridge import CvBridge
    except:
        print "Could not find ROS package: cv_bridge"
        print "If ROS version is pre-Groovy, try putting this package in ROS_PACKAGE_PATH"
        sys.exit(1)
def twincolor2depth(img):
    return np.bitwise_or(np.left_shift(img[:,:,0].astype(np.uint16),8),img[:,:,1].astype(np.uint16))


#NOT USED
#from https://github.com/OSUrobotics/bag2video/blob/master/bag2video.py
def get_info(bag, topic=None, start_time=rospy.Time(0), stop_time=rospy.Time(sys.maxint)):
    size = (0,0)
    times = []

    # read the first message to get the image size
    msg = bag.read_messages(topics=topic).next()[1]
    size = (msg.width, msg.height)

    # now read the rest of the messages for the rates
    iterator = bag.read_messages(topics=topic, start_time=start_time, end_time=stop_time)#, raw=True)
    for _, msg, _ in iterator:
        time = msg.header.stamp
        times.append(time.to_sec())
        size = (msg.width, msg.height)
    diffs = 1/np.diff(times)
    return np.median(diffs), min(diffs), max(diffs), size, times


#NOT USED
#from https://github.com/OSUrobotics/bag2video/blob/master/bag2video.py
def calc_n_frames(times, precision=10):
    # the smallest interval should be one frame, larger intervals more
    intervals = np.diff(times)
    return np.int64(np.round(precision*intervals/min(intervals)))

class SkeletonWriter:
    def __init__(self,topic,outfile):
        self.topic = topic
        self.outfile = outfile
        self.outfilef = open(outfile,"wb")
        self.timestampfile = open(outfile + ".timestamp","w")
    def message(self,msg,time):
        #first pass for precisely estimating timestamp
        self.outfilef.write(msg2json(msg)+"\n")
        self.timestampfile.write("%f\n" % msg.header.stamp.to_time())
    def message1(self,msg,time):
        #first pass for precisely estimating timestamp
        pass
    def close(self):
        self.outfilef.close()
        self.timestampfile.close()

class CameraInfoWriter:
    def __init__(self,topic,outfile):
        self.topic = topic
        self.outfile = outfile
        self.outfilef = open(outfile,"wb")
        self.done = False
    def message1(self,msg,time):
        #first pass for precisely estimating timestamp
        pass
    def message(self,msg,time):
        #first pass for precisely estimating timestamp
        if not self.done:
            self.outfilef.write(msg2json(msg))
            self.done = True
    def close(self):
        self.outfilef.close()

class ImageVideoWriter:
    def __init__(self,topic,outfile,encoding,fourcc,dis):
        self.topic = topic
        self.outfile = outfile
        self.display = dis
        self.fourcc = fourcc
        self.encoding = encoding
        self.encoder = None
        self.bridge = CvBridge()
        self.rate = 30.0
        self.timestampfile = open(outfile + ".timestamp","w")
        if self.display:
            cv2.namedWindow(self.topic)
    def message(self,msg,time):
        # ?? or use mes
        self.timestampfile.write("%f\n" % msg.header.stamp.to_time())
        size = (msg.width, msg.height) #for opencv
        img = np.asarray(self.bridge.imgmsg_to_cv2(msg, 'bgr8'))
        if self.encoder is None:
           self.encoder = cv2.VideoWriter(self.outfile, cv2.VideoWriter_fourcc(*self.fourcc), self.rate, size)
        # TODO reps
        self.encoder.write(img)
        if self.display:
            cv2.imshow(self.topic, img)

    def message1(self,msg,time):
        #first pass for precisely estimating timestamp
        pass
    def close(self):
        if self.encoder:
            self.encoder.release()

class DepthVideoWriter:
    def __init__(self,topic,outfile,en,fourcc,dis,outpublisher):
        self.topic = topic
        self.encoding = en
        self.fourcc = fourcc
        self.outfile = outfile
        self.display = dis
        self.encoder = None
        self.bridge = CvBridge()
        self.rate = 30.0
        self.timestampfile = open(outfile + ".timestamp","w")
        self.xrate = rospy.Rate(30) # 10hz        
        self.publisher = outpublisher
        if outpublisher is not None:
            self.bridge = CvBridge()
        if self.display:
            cv2.namedWindow(self.topic)
    def message(self,msg,time):
        self.timestampfile.write("%f\n" % msg.header.stamp.to_time())
        size = (msg.width, msg.height)
        dimg = np.asarray(self.bridge.imgmsg_to_cv2(msg, '16UC1'))
        img = np.zeros((msg.height,msg.width,3),dtype=np.uint8)
        #most significative on blue (first)
        img[:,:,1] = np.bitwise_and(dimg,0xFF).astype(np.uint8)
        img[:,:,0] = np.right_shift(dimg,8).astype(np.uint8)
        img[:,:,2] = img[:,:,0]
        if self.encoder is None:
           self.encoder = cv2.VideoWriter(self.outfile, cv2.VideoWriter_fourcc(*self.fourcc), self.rate, size)
        self.encoder.write(img)
        if self.publisher is not None:
            msg.header.stamp = rospy.get_rostime()
            self.publisher[0].publish(msg)
            x = self.bridge.cv2_to_imgmsg(twincolor2depth(img), encoding="16UC1")
            self.publisher[1].publish(x)
            x = self.bridge.cv2_to_imgmsg((twincolor2depth(img)-dimg)*10, encoding="16UC1")
            self.publisher[2].publish(x)
            self.xrate.sleep()
        if self.display:
            cv2.imshow(self.topic, img)
    def message1(self,msg,time):
        #first pass
        pass
    def close(self):
        if self.encoder:
            self.encoder.release()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Extract and encode video from bag files.')


    parser.add_argument('--outpath')
    parser.add_argument('--outfile', '-o', action='store', default=None,
                        help='Destination of the video file. Defaults to the location of the input file.')
    parser.add_argument('--doutfile', '-O', action='store', default=None,
                        help='Destination of the video file. Defaults to the location of the input file.')
    parser.add_argument('--coutfile', '-C', action='store', default=None,
                        help='Destination of the video file. Defaults to the location of the input file.')
    parser.add_argument('--soutfile', action='store', default=None,
                        help='Destination of the video file. Defaults to the location of the input file.')
    parser.add_argument('--precision', '-p', action='store', default=10, type=int,
                        help='Precision of variable framerate interpolation. Higher numbers\
                        match the actual framerater better, but result in larger files and slower conversion times.')
    parser.add_argument('--start', '-s', action='store', default=rospy.Time(0), type=rospy.Time,
                        help='Rostime representing where to start in the bag.')
    parser.add_argument('--stop', '-S', action='store', default=rospy.Time(sys.maxint), type=rospy.Time,
                        help='Rostime representing where to stop in the bag.')
    parser.add_argument('--encoding', choices=('rgb8', 'bgr8', 'mono8'), default='bgr8',
                        help='Encoding of the deserialized image.')
    parser.add_argument('--dtopic')
    parser.add_argument('--stopic')
    parser.add_argument('--ctopic')
    parser.add_argument('--republishdepth')
    parser.add_argument('--display',action="store_true")
    parser.add_argument('--ddisplay',action="store_true")
    parser.add_argument('--twopasses',action="store_true")
    parser.add_argument('--fourcc',default="H264")
    parser.add_argument('--dfourcc',default="FFV1",help="lossless is suggested, e.g. HFYU FFV1")

    parser.add_argument('--topic')
    parser.add_argument('--sync',default=True,type=bool)
    parser.add_argument('bagfile')
    
    args = parser.parse_args()

    if args.republishdepth != "":
        rospy.init_node('image_converter', anonymous=True)
        image_pubD = (rospy.Publisher(args.republishdepth,Image),rospy.Publisher(args.republishdepth+"re",Image),rospy.Publisher(args.republishdepth+"de",Image))
    else:
        image_pubD = None


    for bagfile in glob.glob(args.bagfile):
        print bagfile
        if dostop:
            break
        outfile = args.outfile
        #if not outfile:
        #    outfile = os.path.join(*os.path.split(bagfile)[-1].split('.')[:-1]) + '.avi'
        bag = rosbag.Bag(bagfile, 'r')

        if args.outfile is None:
            if args.outpath:
                outfile = os.path.join(args.outpath,os.path.splitext(os.path.split(bagfile)[-1])[0]+".avi")
            else:
                outfile = os.path.join(*os.path.split(bagfile)[-1].split('.')[:-1]) + '.avi'
        else:
            outfile = args.outfile

        if args.doutfile is None:
            if args.outpath:
                doutfile = os.path.join(args.outpath,os.path.splitext(os.path.split(bagfile)[-1])[0]+"-D.avi")
            else:
                doutfile = os.path.join(*os.path.split(bagfile)[-1].split('.')[:-1]) + '-D.avi'
        else:
            doutfile = args.doutfile

        if args.soutfile is None:
            if args.outpath:
                soutfile = os.path.join(args.outpath,os.path.splitext(os.path.split(bagfile)[-1])[0]+"-S.json")
            else:
                soutfile = os.path.join(*os.path.split(bagfile)[-1].split('.')[:-1]) + '-S.json'
        else:
            soutfile = args.soutfile

        if args.coutfile is None:
            if args.outpath:
                coutfile = os.path.join(args.outpath,os.path.splitext(os.path.split(bagfile)[-1])[0]+"-C.json")
            else:
                coutfile = os.path.join(*os.path.split(bagfile)[-1].split('.')[:-1]) + '-C.json'
        else:
            coutfile = args.coutfile

        aa = None
        ab = None
        ac = None
        ad = None
        if args.topic:
            aa = ImageVideoWriter(args.topic,outfile,args.encoding,args.fourcc,args.display)
            print "looking for color with",aa.topic
        if args.dtopic:
            ab = DepthVideoWriter(args.dtopic,doutfile,"",args.dfourcc,args.ddisplay,image_pubD)
            print "looking for depth with",ab.topic
        if args.stopic:
            ac = SkeletonWriter(args.stopic,soutfile)
            print "looking for depth with",ac.topic
        if args.ctopic:
            ad = CameraInfoWriter(args.ctopic,coutfile)
            print "looking for depth with",ad.topic
        targets = [x for x in (aa,ab,ac,ad) if x is not None]
        topics = [x.topic for x in targets]

        #        print 'Calculating video properties'
        #rate, minrate, maxrate, size, times = get_info(bag, args.topic, start_time=args.start, stop_time=args.end)
        #nframes = calc_n_frames(times, args.precision)
        # writer = cv2.VideoWriter(outfile, cv2.cv.CV_FOURCC(*'DIVX'), rate, size)
        #writer = cv2.VideoWriter(outfile, cv2.cv.CV_FOURCC(*'DIVX'), np.ceil(maxrate*args.precision), size)
        #print 'Writing video'
        #write_frames(bag, writer, len(times), topic=args.topic, nframes=nframes, start_time=args.start, stop_time=args.end, encoding=args.encoding)
        #writer.release()
        #print '\n'
        if args.twopasses:
            iterator = bag.read_messages(topics=topics, start_time=args.start, end_time=args.stop)
            for (topic, msg, time) in iterator:
                if dostop:
                    break
                if aa and aa.topic == topic:
                    aa.message1(msg,time)
                elif ab and ab.topic == topic:
                    ab.message1(msg.time)
        if not args.sync or len(targets) == 1:
            iterator = bag.read_messages(topics=topics, start_time=args.start, end_time=args.stop)
            ds = dict(zip(topics,targets))
            for (topic, msg, time) in iterator:
                if dostop:
                    break
                ds[topic].message(msg,time)
                if args.display or args.ddisplay:
                    cv2.waitKey(1)
        else:
            dsv = [timesync.SimpleFilter() for i in range(0,len(targets))]
            ds = dict(zip(topics,dsv))
            ts = timesync.ApproximateTimeSynchronizer(dsv,5,0.3)
            ts.registerCallback(lambda *args: [x.message(y,y.header.stamp) for (x,y) in zip(targets,args)])
            iterator = bag.read_messages(topics=topics, start_time=args.start, end_time=args.stop)
            for (topic, msg, time) in iterator:
                ds[topic].signalMessage(msg) # calls ts.add that MAYBE calls lambda that in turns calls message of the writer

        for x in targets:
            x.close()
