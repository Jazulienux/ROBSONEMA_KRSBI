#!/usr/bin/env python

import rospy
import random as rd
from numpy import *
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray

class Server:
	def __init__(self):
		#================================= Intel NUC Robot ======================================
		self.destination1 = array([[5., 1.5 , 0., 3.5, 3., 0.], #kickoff 0 (x1, y1, z1, x2, y2, z2)
									[4.0, 3., 0., 4.0, 3., 0.], #freekick 1
									[5.5, 0.5, 0., 3.5, 3., 0.], #goalkick 2									
									[3.5, 4., 0., 3.5, 3., 0.], #throwin 3
									[5., 7., 0., 3.5, 2., 180.], #cornerkick 4
									[4., 2., 0., 6., 4.5, 180.], #penalty 5
									[6., 0., 0., 5.5, 1., 0.], #Repair 6
									[3.5, 3., 0., 4., 2., 0.], #goal 7
									[3.5, 3., 0., 3.5, 3., 0.], #Yellow 8
									[6., 0., 0., 6, 0., 0.] #red 9
								]) #robot 1

		self.target1 = array([[1.5, 4.5, 15.], [2.0, 4.5, 10.], [3.0, 4.5, 20.],	#target robot1 ketika memegang bola untuk menendang
							  [1.5, 4.0, 15.], [2.0, 4.0, 10.], [3.0, 4.0, 8.]])
		#1 Robot 2 Melihat Bola #2 bola dipegang Robot1 #3 bola dipegang kiper 
		self.cari_posisi1 = array([[0., 0., 0.], [4.5, 2.0, 0.], [5.0, 3.0, 0.], [5.0, 4.5, 0.]]) 
		#variable penampung jarak terdekat
		self.distance1 = array([0., 0., 0., 0., 0., 0., 0., 0., 0.])
		#Pemilihan tendang langsung atau meuju posisi
		self.kick_mode1 = array([0, 1, 1, 0, 1, 0, 0, 0, 1, 0])
		self.lock1 = 0
		self.index_lokalisasi1 = 0
		self.robot1_ball = 0 #status bola pada dribbler
		self.ball_radius1 = 0 #radius bola pada kamera

		#================================== Nvidia Jetson Robot ===================================
		self.destination2 = array([[2., 3. , 0., 1.3, 3., 0.], #kickoff 0 (x1, y1, z1, x2, y2, z2)
									[2.0, 3., 0., 2.0, 3., 0.], #freekick 1
									[0.5, 1., 0., 2.5, 3., 0.], #goalkick 2									
									[2.5, 4., 0., 2.5, 3., 0.], #throwin 3
									[1., 7., 0., 2.5, 2., 180.], #cornerkick 4
									[2., 2., 0., 0., 4.5, 180.], #penalty 5
									[0., 0., 0., 0.5, 1., 0.], #Repair 6
									[2.5, 3., 0., 2., 3., 0.], #goal 7
									[2.5, 3., 0., 2.5, 3., 0.], #Yellow 8
									[0., 0., 0., 0., 0., 0.] #red 9
								]) #robot 2

		self.target2 = array([[2.0, 4.5, 0.], [2.5, 3.5, 355.], [3.0, 4.5, 8.],
							  [2.0, 3.0, 0.], [2.5, 3.0, 0.], [3.0, 3.0, 8.],
							  [3.0, 6.0, 0.]])
		#1 Bola dipegang robot 1 #3 bola dipegang kiper 
		self.cari_posisi2 = array([[0., 0., 0.], [2., 3., 0.], [0., 0., 0.], [1., 4.5, 0.]])
		#variable penampung jarak terdekat
		self.distance2 = array([0., 0., 0., 0., 0., 0., 0., 0.])
		#Pemilihan tendang langsung atau meuju posisi
		self.kick_mode2 = array([0, 1, 1, 0, 1, 0, 0, 0, 1, 0])
		self.lock2 = 0
		self.index_lokalisasi2 = 0
		self.robot2_ball = 0
		self.ball_radius2 = 0

		#================================= Intel NUC Keeper Robot =============================
		self.robot3_ball=0

		#================================= Global Variable ====================================
		self.odom = array([[0., 0., 0.], [0., 0., 0.], [0., 0., 0.]])
		self.cmd = array([[0, 1]])
		self.positioning = 0
		self.index_bola = 0
		self.find_ball = 0

		print "node manager running"
		print self.positioning

	def odometry_fw(self, dat):#subscribe odometry robot 1
		self.odom[0,0] = dat.x
		self.odom[0,1] = dat.y
		self.odom[0,2] = dat.z
		#self.command_skill_robot1()

	def odometry_cb(self, dat):#subscribe odometry robot 2
		self.odom[1,0] = dat.x
		self.odom[1,1] = dat.y
		self.odom[1,2] = dat.z		
		#self.command_skill_robot2()

	def odometry_gk(self, dat):#subscribe odometry robot 3
		self.odom[2,0] = dat.x
		self.odom[2,1] = dat.y
		self.odom[2,2] = dat.z
		#self.command_skill_robot3()

	def get_camera_robot1(self, dat):	#subscribe data camera robot1
		self.ball_radius1 = -dat.z

	def get_camera_robot2(self, dat):	#subscribe data camera robot1
		self.ball_radius2 = -dat.z

	def get_ball_position1(self, dat):	#subscribe posisi bola robot1
		self.robot1_ball = dat.data
		self.command_skill_robot1()

	def get_ball_position2(self, dat):	#subscribe posisi bola robot2
		self.robot2_ball = dat.data
		self.command_skill_robot2()

	def get_ball_position3(self, dat):	#subscribe posisi bola robot3
		self.robot3_ball = dat.data
		self.command_skill_robot3()

	def decide_ball_position(self):
		if(self.robot1_ball != 0):
			self.index_bola = 1	
		elif(self.robot2_ball != 0):
			self.index_bola = 2	
		elif(self.robot3_ball != 0):
			self.index_bola = 3
		else:
			self.index_bola = 0
		

	def get_acc(self,dat):				#subscribe info perintah node telah selesai dijalankan (optional)
		self.clear = dat.data
		if(self.clear == 1):
			self.index_lokalisasi = self.index_lokalisasi + 1
			if(self.index_lokalisasi > 4):
				self.positioning = 0
			else :
				self.positioning = 5
		self.clear = 0

	def command_input(self, dat):		#Subscribe Data Perintah dari Ref_Box
		self.positioning = dat.data[0]	#mode gerakan, 0 = stop, 1 = Kejar Bola, 2 = Positioning
		self.cmd[0,0] = dat.data[1]		#kode gerakan yang akan dilakukan (Kick Off, Goal Kick, Corner Kick)
		self.cmd[0,1] = dat.data[2]		#kode posisi menyerang atau posisi bertahan
		self.command_skill_robot1()
		self.command_skill_robot2()
		self.command_skill_robot3()
		print self.positioning	
	
	def command_skill_robot1(self):		#Publish Perintah yang diberikan pada robot 1 (Intel NUC robot)
		dat1.data[0] = self.positioning

		#if(self.positioning == 0):		#Stop
			#print "ROBOT 1 STOP"

		if(self.positioning == 1): #game on / start
			if(self.robot1_ball == 1):#jika bola dipegang robot 1
				for i in range(0,4):
					self.distance1[i] = sqrt((self.odom[0,0] - self.target1[i,0])**2 + (self.odom[0,1] - self.target1[i,1])**2)
				index_target = argmin(self.distance1)
				print index_target
				if(index_target >= 0) and (index_target <= 4):
					index_target = index_target
				else:
					index_target = 1
				dat1.data[1] = self.target1[index_target, 0]
				dat1.data[2] = self.target1[index_target, 1]
				theta1 = self.target1[index_target, 2] * pi /180
				dat1.data[3] = theta1
				#print "ROBOT 1 GO TO POSITION\t", dat1.data[1], dat1.data[2], dat1.data[3]
		
		elif(self.positioning == 2): #positioning
			if(self.cmd[0,1] == 0):
				dat1.data[1] = self.destination1[self.cmd[0,0], 0]
				dat1.data[2] = self.destination1[self.cmd[0,0], 1]
				theta1 = self.destination1[self.cmd[0,0], 2] * pi /180
				dat1.data[3] = theta1
			elif(self.cmd[0,1] == 1):
				dat1.data[1] = self.destination1[self.cmd[0,0], 3]
				dat1.data[2] = self.destination1[self.cmd[0,0], 4]
				theta1 = self.destination1[self.cmd[0,0], 5] * pi /180
				dat1.data[3] = theta1
			#print "ROBOT1 destination : ", dat1

		elif(self.positioning == 3):
			print "penalty kick"
			dat1.data[1] = self.odom[1,0]
			dat1.data[2] = self.odom[1,1]
			dat1.data[3] = 0.*pi/180.

		robot1_pub.publish(dat1)
		robot1_action.publish(self.positioning)
	
	def command_skill_robot2(self):#publish skill
		dat2.data[0] = self.positioning
		#if(self.positioning == 0):
			#print "ROBOT 2 STOP"

		if(self.positioning == 1): #game on / start
			if(self.robot2_ball == 1):
				for i in range(0,6):
					self.distance2[i] = sqrt((self.odom[1,0] - self.target2[i,0])**2 + (self.odom[1,1] - self.target2[i,1])**2)
				index_target = argmin(self.distance2)
				dat2.data[1] = self.target2[index_target, 0]
				dat2.data[2] = self.target2[index_target, 1]
				theta2 = self.target2[index_target, 2] * pi /180
				dat2.data[3] = theta2
				print "ROBOT 2 GO TO POSITION\t", dat2.data[1], dat2.data[2], dat2.data[3]
		
			elif(self.robot1_ball == 1):
				dat2.data[1] = self.cari_posisi2[self.index_bola, 0]
				dat2.data[2] = self.cari_posisi2[self.index_bola, 1]
				theta2 = self.cari_posisi2[self.index_bola, 2] * pi /180
				dat2.data[3] = theta2
				print "ROBOT 2 LOOKING FOR POSITION\t", dat2.data[1], dat2.data[2], dat2.data[3]

		elif(self.positioning == 2): #positioning
			if(self.cmd[0,1]==0):
				dat2.data[1] = self.destination2[self.cmd[0,0], 0]
				dat2.data[2] = self.destination2[self.cmd[0,0], 1]
				theta2 = self.destination2[self.cmd[0,0], 2] * pi /180
				dat2.data[3] = theta2
			if(self.cmd[0,1]==1):
				dat2.data[1] = self.destination2[self.cmd[0,0], 3]
				dat2.data[2] = self.destination2[self.cmd[0,0], 4]
				theta2 = self.destination2[self.cmd[0,0], 5] * pi /180
				dat2.data[3] = theta2
			#print "ROBOT2 destination : ", dat2
		elif(self.positioning == 3):
			print "penalty kick"
			dat2.data[1] = self.odom[1,0]
			dat2.data[2] = self.odom[1,1]
			dat2.data[3] = 0.*pi/180.

		robot2_pub.publish(dat2)
		robot2_action.publish(self.positioning)
		
	def command_skill_robot3(self):#publish skill
		robot3_pub.publish(self.positioning)
		
if __name__ == "__main__":
	rospy.init_node('robot3_manager_node')

	robot1_pub = rospy.Publisher('robot1/skill', Float32MultiArray, queue_size = 10)
	robot1_action = rospy.Publisher('robot1/action', Int32, queue_size = 10)
	
	robot2_pub = rospy.Publisher('robot2/skill', Float32MultiArray, queue_size = 10)
	robot2_action = rospy.Publisher('robot2/action', Int32, queue_size = 10)
	
	robot3_pub = rospy.Publisher('robot3/action', Int32, queue_size = 10)
	dat1 = Float32MultiArray()
	dat2 = Float32MultiArray()
	server = Server()
	dat1.data = [0, 0., 0., 0., 0., 0]
	dat2.data = [0, 0., 0., 0., 0., 0]
	try:
		rospy.Subscriber('robot1/odometry', Vector3, server.odometry_fw)
		rospy.Subscriber('robot1/ball_position', Int32, server.get_ball_position1)
		rospy.Subscriber('robot1/command_clear', Int32, server.get_acc)
		rospy.Subscriber('robot1/camera', Vector3, server.get_camera_robot1)

		rospy.Subscriber('robot2/odometry', Vector3, server.odometry_cb)
		rospy.Subscriber('robot2/ball_position', Int32, server.get_ball_position2)
		rospy.Subscriber('robot2/command_clear', Int32, server.get_acc)
		rospy.Subscriber('robot2/camera', Vector3, server.get_camera_robot2)

		rospy.Subscriber('robot3/odometry', Vector3, server.odometry_gk)
		#rospy.Subscriber('robot3/ball_position', Int32, server.get_ball_position3)

		rospy.Subscriber('base_station_command', Int32MultiArray, server.command_input)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass