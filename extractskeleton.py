import rosbag
from std_msgs.msg import Int32, String
import sys

if len(sys.argv) != 3:
	print "inputbag msgstore"
else:
	inbag = rosbag.Bag(sys.argv[1])


	for topic, msg, t in inbag.read_messages():
		if topic == "/skeleton/users":
			print "found MD5",msg._md5sum,len(msg._full_text)
			onf = open(sys.argv[2],"wb")
			onf.write(msg._full_text)
			onf.close()
			onf = open(sys.argv[2]+".md5","wb")
			onf.write(msg._md5sum)
			onf.close()
			break
	inbag.close()