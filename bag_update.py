#! /usr/bin/env python
import rosbag
import sys

assert len(sys.argv) >= 3


def updatedETH():
	inbag_uwe = rosbag.Bag(sys.argv[1], 'r')
	inbag_eth = rosbag.Bag(sys.argv[2], 'r')
	outbag = rosbag.Bag(sys.argv[3], 'w')

	for topic, msg, t in inbag_uwe.read_messages(topics=['/camera/depth_registered/camera_info', '/camera/depth_registered/image_raw']):
		outbag.write(topic, msg, t)

	for topic, msg, t in inbag_eth.read_messages():
		outbag.write(topic, msg, t)


def updeteOther():
	inbag = rosbag.Bag(sys.argv[1], 'r')
	outbag = rosbag.Bag(sys.argv[2], 'w')

	for topic, msg, t in inbag.read_messages(topics=['/camera/depth_registered/camera_info']):
		camera_info = msg
		break

	for topic, msg, t in inbag.read_messages(topics=['/ArmPoseEff', '/ArmPoseTCP', '/camera/depth_registered/camera_info', '/camera/depth_registered/image_raw', '/camera/rgb/image_color', '/palpation_effector/force', '/palpation_effector/humidity', '/palpation_effector/temperature', '/tf']):
		outbag.write(topic, msg, t)
		if topic == '/camera/rgb/image_color':
			camera_info.header.stamp = msg.header.stamp
			outbag.write('/camera/rgb/camera_info', camera_info, t)

	inbag.close()
	outbag.close()

def updateRamcip():
	inbag = rosbag.Bag(sys.argv[1], 'r')
	outbag = rosbag.Bag(sys.argv[2], 'w')
	for topic, msg, t in inbag.read_messages(topics=['/kinect1/depth/camera_info', '/kinect1/rgb/image/compressed', '/kinect2/qhd/camera_info', '/kinect2/qhd/image_color_rect', '/trigger', '/tf']):
		if topic == '/kinect1/rgb/image/compressed':
			msg.header.frame_id = 'kinect1_rgb_optical_frame'
		outbag.write(topic, msg, t)

	inbag.close()
	outbag.close()
	
if __name__ == '__main__':
	#updateRamcip()
	#updateOther()
	updatedETH()