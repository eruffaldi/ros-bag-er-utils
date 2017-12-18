#!/usr/bin/env python
#from https://github.com/OSUrobotics/bag2video/blob/master/bag2video.py
from __future__ import division
import rosbag, rospy, numpy as np
import sys, os, cv2, glob
from itertools import izip, repeat
import argparse
import signal
import sys

dostop=False
def signal_handler(signal, frame):
        print('You pressed Ctrl+C!')
        dostop=True
        sys.exit(0)
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

def get_size(bag, topic=None):
    size = (0,0)
    times = []

    # read the first message to get the image size
    msg = bag.read_messages(topics=topic).next()[1]
    size = (msg.width, msg.height)
    return size

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

def calc_n_frames(times, precision=10):
    # the smallest interval should be one frame, larger intervals more
    intervals = np.diff(times)
    return np.int64(np.round(precision*intervals/min(intervals)))

def write_frames(bag, writer, total, topic=None, nframes=repeat(1), start_time=rospy.Time(0), stop_time=rospy.Time(sys.maxint), viz=False, encoding='bgr8',store_timings=None,incremental=False):
    global dostop
    bridge = CvBridge()
    if viz:
        cv2.namedWindow('win')
    count = 1
    if store_timings:
        store_timings = open(store_timings,"w")
    if not incremental:
        iterator = bag.read_messages(topics=topic, start_time=start_time, end_time=stop_time)
        for (topic, msg, time), reps in izip(iterator, nframes):
            print '\rWriting frame %s of %s at time %s repetitions %d' % (count, total, time,reps),
            if store_timings:
                time = msg.header.stamp
                store_timings.write("%f\n" % time.to_sec())
                pass
            img = np.asarray(bridge.imgmsg_to_cv2(msg, 'bgr8'))
            for rep in range(reps):
                writer.write(img)
            if viz:
                imshow('win', img)
            count += 1
            if dostop:
                break
    else:
        pass

def imshow(win, img):
    cv2.imshow(win, img)
    cv2.waitKey(1)

def noshow(win, img):
    pass

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Extract and encode video from bag files.')
    parser.add_argument('--outfile', '-o', action='store', default=None,
                        help='Destination of the video file. Defaults to the location of the input file.')
    parser.add_argument('--precision', '-p', action='store', default=10, type=int,
                        help='Precision of variable framerate interpolation. Higher numbers\
                        match the actual framerater better, but result in larger files and slower conversion times.')
    parser.add_argument('--viz', '-v', action='store_true', help='Display frames in a GUI window.')
    parser.add_argument('--start', '-s', action='store', default=rospy.Time(0), type=rospy.Time,
                        help='Rostime representing where to start in the bag.')
    parser.add_argument('--end', '-e', action='store', default=rospy.Time(sys.maxint), type=rospy.Time,
                        help='Rostime representing where to stop in the bag.')
    parser.add_argument('--encoding', choices=('rgb8', 'bgr8', 'mono8'), default='bgr8',
                        help='Encoding of the deserialized image.')
    parser.add_argument('--fourcc', default='DIVX',
                        help='Encoding of the video.')
    parser.add_argument('--rateless',action="store_true")

    parser.add_argument('topic')
    parser.add_argument('bagfile')

    args = parser.parse_args()

    if not args.viz:
        imshow = noshow

    for bagfile in glob.glob(args.bagfile):
        print bagfile
        outfile = args.outfile
        if not outfile:
            outfile = os.path.join(*os.path.split(bagfile)[-1].split('.')[:-1]) + '.avi'
        bag = rosbag.Bag(bagfile, 'r')
        print 'Calculating video properties'
        if not args.rateless:
            rate, minrate, maxrate, size, times = get_info(bag, args.topic, start_time=args.start, stop_time=args.end)
            nframes = calc_n_frames(times, args.precision)
            # writer = cv2.VideoWriter(outfile, cv2.cv.CV_FOURCC(*'DIVX'), rate, size)
        else:
            maxrate = 30/args.precision
            size = get_size(bag,args.topic)
            times = []
            nframes = repeat(1)
        writer = cv2.VideoWriter(outfile, cv2.VideoWriter_fourcc(*args.fourcc), np.ceil(maxrate*args.precision), size)
        print 'Writing video',"with framerate",np.ceil(maxrate*args.precision), "size",size,"to",outfile
        write_frames(bag, writer, len(times), topic=args.topic, nframes=nframes, start_time=args.start, stop_time=args.end, encoding=args.encoding, store_timings=outfile+".timestamp" if args.rateless else None)
        writer.release()
        print '\n'