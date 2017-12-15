#
# Given a specific message considered as sync, keep only messages related to a triggering message
#
# Policies:
# - exact to trigger
# - nearest in the past to trigger
# - nearest to trigger
#
# Special: tf has to be preserved
import rosbag
from std_msgs.msg import Int32, String
import sys
from geometry_msgs.msg import TransformStamped
import numpy
import StringIO
import argparse

class MsgQueue:
	def __init__(self,topic,maxn):
		self.queue = []
		self.topic = topic
		self.maxn = maxn
	def add(self,msg,t):
		if len(self.queue) == self.maxn:
			self.queue = self.queue[1:]
		self.queue.append((t,msg))

def do_list(args):
	inbag = rosbag.Bag(args.input)
	lasttime = None
	always = set(args.always.split(","))
	ignore = set(args.ignore.split(","))
	notified = set()

	for topic, msg, msgtime in inbag.read_messages():
		if hasattr(msg,"header") and hasattr(msg.header,"stamp"):
			t = msg.header.stamp
		else:
			if not topic in notified:
				print "not timestamp message",topic
				notified.add(topic)
			t = msgtime
		if lasttime is not None:
			dt = (t-lasttime)/1000000.0
		else:
			dt = 0
		if topic == args.trigger:
			print msgtime,t
			lasttime = msgtime
		elif topic in ignore:
			continue
		elif topic in always:
			print "\t","!%-39s" %topic,msgtime,t,dt
		else:
			print "\t","%-40s" % topic,msgtime,t,dt

def main():


	parser = argparse.ArgumentParser(description='Filters via trigger')
	parser.add_argument('input',help="input bag")
	parser.add_argument('output',help="output bag")
	parser.add_argument('--trigger',help="trigger",default="/trigger")
	parser.add_argument('--policy',help="trigger sync policy",choices=["exact","last","nearest","after","beforeafter"],default="last")
	parser.add_argument('--always',help="always kept space separated",default="/tf")
	parser.add_argument('--ignore',help="ignore",default="")
	parser.add_argument('--resync',action="store_true")
	parser.add_argument('--list',action="store_true")

	args = parser.parse_args()


	if args.list:
		do_list(args)
		return
	inbag = rosbag.Bag(args.input)
	outbag = rosbag.Bag(args.output, 'w')
	ignores = set(args.ignore.split(","))
	always = set(args.always.split(","))

	others = dict()

	MODE_EXACT=0
	MODE_LAST=1
	MODE_NEAREST=2
	MODE_AFTER=3
	MODE_BEFOREAFTER=4

	policycode = dict(exact=MODE_EXACT,last=MODE_LAST,nearest=MODE_NEAREST,after=MODE_AFTER,beforeafter=MODE_BEFOREAFTER)
	policy = policycode[args.policy]
	queues = dict()
	lasttrigger = None
	emissionqueue = []
	pendingtriggertime = None
	needemit = set()
	needafter = False
	for topic, msg, msgtime in inbag.read_messages():
		if topic in ignores:
			continue
		#if hasattr(msg,"header") and hasattr(msg.header,"stamp"):
		#	t = msg.header.stamp
		#else:
		t = msgtime
		if pendingtriggertime is not None and msgtime > pendingtriggertime:
			if policy == MODE_EXACT:
				# only precise timing, clear rest
				for m in queues.values():
					if len(m.queue) > 0 and m.queue[-1][0] == pendingtriggertime:
						outbag.write(m.topic,m.queue[-1][1],m.queue[-1][0])
						print "\texact",m.topic,m.queue[-1][0],(m.queue[-1][0]-pendingtriggertime)/1E6
						m.queue = []
			elif policy == MODE_LAST:				
				# only last
				for m in queues.values():
					if len(m.queue) > 0:
						outbag.write(m.topic,m.queue[-1][1],m.queue[-1][0])
						print "\tlast",m.topic,m.queue[-1][0],(m.queue[-1][0]-pendingtriggertime)/1E6
						m.queue = []
			elif policy == MODE_NEAREST:
				# emit last and append for others			
				for m in queues.values():
					if len(m.queue) > 0:
						outbag.write(m.topic,m.queue[-1][1],m.queue[-1][0])
						print "\tnearest",m.topic,m.queue[-1][0],(m.queue[-1][0]-pendingtriggertime)/1E6
						m.queue = []
					else:
						# will be scheduled at next
						needemit.add(m.topic)
				needafter = len(needemit) > 0
			elif policy == MODE_AFTER:		
				# same time or after
				for m in queues.values():
					if len(m.queue) > 0 and m.queue[-1][0] == pendingtriggertime:
						outbag.write(m.topic,m.queue[-1][1],m.queue[-1][0])
						print "\tafter",m.topic,m.queue[-1][0],(m.queue[-1][0]-pendingtriggertime)/1E6
						m.queue = []				
					else:
						# will be scheduled at next
						needemit.add(m.topic)
				needafter = len(needemit) > 0
			elif policy == MODE_BEFOREAFTER:		
				# same time or after
				for m in queues.values():
					if len(m.queue) > 0:
						outbag.write(m.topic,m.queue[-1][1],m.queue[-1][0])
						print "\tbefore",m.topic,m.queue[-1][0],(m.queue[-1][0]-pendingtriggertime)/1E6
						m.queue = []
					# will be scheduled at next
					needemit.add(m.topic)
				needafter = len(needemit) > 0
			pendingtriggertime = None
		if topic == args.trigger:
			# trigger was stored in two ways
			lasttrigger = t
			pendingtriggertime = t
			outbag.write(topic,msg,t)
			needemit = set()
			needafter = False
			print "emit trigger",t
		elif topic in always:
			outbag.write(topic,msg,t)
		else:
			w = queues.get(topic)
			if w is None:
				w = MsgQueue(topic,5)
				queues[topic] = w
				if needafter:
					outbag.write(topic,msg,t)
					print "\tafterunknown",topic,t
			else:
				if needafter:
					if topic in needemit:
						needemit.remove(topic)
						outbag.write(topic,msg,t)
						needafter = len(needemit) > 0
						print "\tafter",topic,t,(t-lasttrigger)/1E6
						continue
			w.add(msg,t)
	outbag.close()

	# finish emitting outstanding (only policy nearest)

	#outbag.write(topic,msg,t)
	#outbag.close()
if __name__ == '__main__':
		main()	


