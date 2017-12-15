import sys
import os
import subprocess

def scan(fp):
	global badurdftime
	if os.path.isfile(fp):
		if not fp.endswith(".bag"):
			return
		print "scanning",fp
		result = subprocess.check_output("rosbag info \"%s\"" % fp, shell=True)
		resultl = result.split("\n")
		goodl = set(["start","end","size"])
		resultg = []
		di = dict()
		for y in resultl:
			a = y.split(":",1)
			if len(a) > 1:
				di[a[0]] = a[1]
			else:
				continue
			if a[0] in goodl:
				resultg.append("\t"+y)
		ts= float(di["start"].split("(")[1].rstrip(") "))
		if ts < badurdftime:
			resultg.append("\tBAD URDF")
		if result.find("/kinect1/depth/image/compressed") > 0 and result.find("/kinect1/depth/image/compressedDepth") == -1:
			resultg.append("\tBAD compression")
		if result.find("/kinect1/depth/image") < 0:
			resultg.append("\tMISSING depth")
		if result.find("/joint_states") < 0 or result.find("/tf") < 0:
			resultg.append("\tBAD robot info")
		if result.find("/board") > 0:
			resultg.append("\twith Aruco Board Detector")
		if result.find("/plane") > 0:
			resultg.append("\twith Plane Segmentator")
		if result.find("/trigger") > 0:
			resultg.append("\twith Trigger")
		print "\n".join(resultg)
	else:
		for y in os.listdir(fp):
			if y[0] != ".":
				ffp = os.path.join(fp,y)
				scan(ffp)


if __name__ == '__main__':
	badurdftime = 1475748170.61
	for a in sys.argv[1:]:
		scan(os.path.abspath(a))
