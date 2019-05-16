#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String

import sys, select, termios, tty

msg = """
Welcome to -- ROBSONEMA BASE STATION TELEOP KEY --
---------------------------
===== general command =====
s = stop all robot
S = Start ball finder

===== positioning command =====
q = kick off
w = corner kick
e = goal kick

h = HELP

CTRL-C to quit
"""
#skill, position, info, goalkeeper command
moveBindings = {
		's':(0,0,"STOP",0),
		'S':(1,0,"START",0), 
		'q':(2,0,"KICK OFF",0),
		'Q':(2,0,"KICK OFF",1),
		'w':(2,1,"FREE KICK",0),
		'W':(2,1,"FREE KICK",1),
		'e':(2,2,"GOAL KICK",0),
		'E':(2,2,"GOAL KICK",1),
		'r':(2,3,"THROW IN A",0),
		'R':(2,3,"THROW IN D",1),
		't':(2,4,"CORNER KICK",0),
		'T':(2,4,"GOAL KICK",1),
		'h':(20,0,"HELP",1),
		}

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)

	rospy.init_node('base_station_teleop_node')
	pub = rospy.Publisher('base_station_command', Int32MultiArray, queue_size = 10)
	command = Int32MultiArray()
	note = String()
	command.data = [0,0,0]
	try:
		print(msg)
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				skill = moveBindings[key][0]
				code = moveBindings[key][1]
				sym = moveBindings[key][2]
				act = moveBindings[key][3]
			else:
				skill = 0
				code = 0
				sym = "STOP"
				if (key == '\x03'):
					break
			if(skill == 20):
				print(msg)
			else:
				command.data[0] = skill
				command.data[1] = code
				command.data[2] = act
				note.data = sym
			pub.publish(command)
			print(note)
		
	except Exception as e:
		print(e)

	finally:
		pub.publish(command)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


