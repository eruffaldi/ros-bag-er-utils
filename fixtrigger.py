import rosbag
from std_msgs.msg import Int32, String
import sys
from geometry_msgs.msg import TransformStamped
import numpy
import StringIO

def dupe_joint(j):
	r = Joint()
	r.name = j.name
	r.confidence =j.confidence
	r.position = j.position
	return j

def dupe_skeleton(s):
	r = Skeleton()
	r.user_id = s.user_id
	r.joints = [dupe_joint(j) for j in s.joints]
	return r

def main():
	if len(sys.argv) != 3:
		print "inputbag outputbag"
	else:
		outbag = rosbag.Bag(sys.argv[2], 'w')
		inbag = rosbag.Bag(sys.argv[1])
		#if len(sys.argv) == 3:
		z = TransformStamped()
		rep = z._full_text
		repmd5 = z._md5sum
		#else:
		#	rep = open(sys.argv[3],"rb").read()
		#	repmd5 = Skeleton.open(sys.argv[3]+".md5","rb").read()

		for topic, msg, t in inbag.read_messages():
			if topic == "/trigger":
				# _md5sum
				# _type
				if msg._md5sum != repmd5:
					print "fix due to diff MD5"
					q = TransformStamped()
					if False:
						c = StringIO.StringIO()
						msg.serialize(c)
						print len(str(c))
						try:
							q.deserialize(str(c))
						except:
							print "bug"
							break
						print "got",len(str(c))
					else:
						q.header.stamp = t
					#msg._full_text = rep
					#msg._md5sum = repmd5
					msg = q
				else:
					print "nofix"
				outbag.write(topic,msg,t)
			else:
				outbag.write(topic,msg,t)
		outbag.close()
if __name__ == '__main__':
		main()	


