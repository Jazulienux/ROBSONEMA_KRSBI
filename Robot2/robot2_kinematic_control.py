#!/usr/bin/env python
#kinematic control robot2
import sympy as sp
import numpy as np
from numpy import *
import matplotlib.pyplot as plt
from numpy.linalg import inv
import rospy
import random as rd
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32


class Server:
	def __init__(self):
		self.alp = array([[0*pi/180, 120*pi/180, 240*pi/180]]).T #robot specification
		self.cam_des = sp.Matrix([[240., 90*pi*180, -60.]]).transpose()
		self.cam = sp.Matrix([[0., 0., 0.]]).transpose()
		self.omni_cam = sp.Matrix([[0., 0., 0.]]).transpose()
		self.cam_rot = sp.Matrix([[1,0,0], [0,0,1], [0,0,0]])
		self.l = 0.20		
		self.r = 0.05
		self.Jr = self.jacobianR()
		self.th = 0. * pi / 180.
		self.x_des = sp.Matrix([[0., 0., 0.]]).transpose()
		self.x = sp.Matrix([[0, 0, 0]]).transpose()#koordinat actual
		self.lamda = array([[20, 20]])
		self.kp1 = 0.85 #0.85
		self.kp2 = 8.0 # 1.75
		self.kp3 = 5.
		self.v_lambda = sp.Matrix([[self.kp1 , 0., 0.], [0., self.kp2, 0.], [0., 0., self.kp3]])
		self.iLs = self.camera_matrix()
		self.goal_pos = array([[320.*pi/180., 5.*pi/180, 15.*pi/180.]]).T
		self.skill = 0
		self.lastcm = 0
		self.diff = 0
		self.ball = 0
		self.ball1 = 0
		self.detik = 0
		self.kick = 0
		print "kinematic robot2 control running"
		print self.skill

	def jacobianR(self):
		Js=array([[0.,0.,0.],[0.,0.,0.],[0.,0.,0.]])
		Js[0,0]=sp.cos(self.alp[0,0])
		Js[0,1]=sp.cos(self.alp[1,0])
		Js[0,2]=sp.cos(self.alp[2,0])
		Js[1,0]=sp.sin(self.alp[0,0])
		Js[1,1]=sp.sin(self.alp[1,0])
		Js[1,2]=sp.sin(self.alp[2,0])
		Js[2,0]=1./self.l
		Js[2,1]=1./self.l
		Js[2,2]=1./self.l
		#print(Jr)
		return self.r*Js
	
	def jacobianW(self,th,Jr):
		rotZ = array([[sp.cos(th), -sp.sin(th), 0.], [sp.sin(th), sp.cos(th), 0.], [0., 0., 1]])
		#print(rotZ)
		Jw = rotZ.dot(Jr)
		return sp.Matrix(Jw)
		
	def camera_matrix(self):
		Ls = sp.Matrix([[0,0,0],[0,0,0],[0,0,0]])
		Ls[0,0] = -1. / 0.3
		Ls[0,1] = 0
		Ls[0,2] = 160. / 0.3
		Ls[1,0] = 0
		Ls[1,1] = -1./ 0.3
		Ls[1,2] = 240./0.3
		Ls[2,0] = 0
		Ls[2,1] = 0
		Ls[2,2] = 60. * (2. / 0.3)
		Lsinv = Ls.inv()
		#sp.pprint(Ls)
		#sp.pprint(Lsinv)
		return Lsinv

	def get_odom(self, dat):
		self.x[0, 0] = dat.x #present x coordinate
		self.x[1, 0] = dat.y #present y coordinate
		self.x[2, 0] = dat.z * pi / 180. #present heading in radian
		self.th = self.x[2, 0]
		#print self.th
		self.main()

	def get_skill(self, dat):
		self.skill = dat.data[0]
		self.x_des[0, 0] = dat.data[1]
		self.x_des[1, 0] = dat.data[2]
		self.x_des[2, 0] = dat.data[3]
		
	def get_camera(self, dat):
		self.cam[0,0] = dat.x
		self.cam[1,0] = dat.y
		self.cam[2,0] = -dat.z
		self.main()

	def get_ball_info(self, dat):
		self.ball = dat.data

	def get_ball_info1(self, dat):
		self.ball1 = dat.data

	def pwm_leveling(self, w):
		#====================== W1 ====================
		if(w[0,0]>0.4) and (w[0,0]<125):
			w[0,0] = 125
		elif (w[0,0]<-0.4) and (w[0,0]>-125):
			w[0,0] = -125
		elif(w[0,0]>400):
			w[0,0] = 400
		elif (w[0,0]<-400):
			w[0,0] = -400
		#====================== W2 ====================
		if(w[1,0]>3) and (w[1,0]<150):
			w[1,0] = 150
		elif (w[1,0]<-3) and (w[1,0]>-150):
			w[1,0] = -150
		elif(w[1,0]>400):
			w[1,0] = 400
		elif (w[1,0]<-400):
			w[1,0] = -400
		#====================== W3 ====================
		if(w[2,0]>3) and (w[2,0]<150):
			w[2,0] = 150
		elif (w[2,0]<-3) and (w[2,0]>-150):
			w[2,0] = -150
		elif(w[2,0]>400):
			w[2,0] = 400
		elif (w[2,0]<-400):
			w[2,0] = -400
		pwm.y = w[0,0]
		pwm.z = w[1,0]
		pwm.x = w[2,0]
		pub.publish(pwm)
		
	def pwm_leveling1(self, w):
		#====================== W1 ====================
		if(w[0,0]>0.4) and (w[0,0]<100):
			w[0,0] = 100
		elif (w[0,0]<-0.4) and (w[0,0]>-100):
			w[0,0] = -100
		if(w[0,0]>400):
			w[0,0] = 400
		elif (w[0,0]<-400):
			w[0,0] = -400
		#====================== W2 ====================
		if(w[1,0]>3) and (w[1,0]<100):
			w[1,0] = 100
		elif (w[1,0]<-3) and (w[1,0]>-100):
			w[1,0] = -100
		if(w[1,0]>400):
			w[1,0] = 400
		elif (w[1,0]<-400):
			w[1,0] = -400
		#====================== W3 ====================
		if(w[2,0]>3) and (w[2,0]<100):
			w[2,0] = 100
		elif (w[2,0]<-3) and (w[2,0]>-100):
			w[2,0] = -100
		if(w[2,0]>400):
			w[2,0] = 400
		elif (w[2,0]<-400):
			w[2,0] = -400
		pwm.y = w[0,0]
		pwm.z = w[1,0]
		pwm.x = w[2,0]
		pub.publish(pwm)
		
	def visual_servoing(self):
		J = self.jacobianW(self.th, self.Jr)
		vJinv = sp.Matrix(self.Jr).inv()
		self.Jinv = J.inv()
		er = self.cam_des - self.cam
		rr = self.cam_rot * er
		rr[2,0] = self.cam[1,0] - (90.*pi/180.) #heading ball
		if(self.cam[2,0] >= -20):
			if(self.cam[0,0]>30) and (self.cam[0,0]<280):
				self.v_lambda[0,0] = 2.8  #2.5
				self.v_lambda[1,1] = 2.75 #2.75
				self.v_lambda[2,2] = 5.0
			else:
				self.v_lambda[0,0] = 1.0#0.75
				self.v_lambda[1,1] = 1.5
				self.v_lambda[2,2] = 6
		elif(self.cam[2,0] >= -40):
			self.v_lambda[0,0] = 0.75
			self.v_lambda[1,1] = 1.5
			self.v_lambda[2,2] = 4
		else:
			self.v_lambda[0,0] = 0.6
			self.v_lambda[1,1] = 1.4
			self.v_lambda[2,2] = 7.
		if((self.cam[1,0] - self.lastcm) < 0):
			self.diff = 2
		elif((self.cam[1,0] - self.lastcm) > 0):
			self.diff = -2
		if(self.cam[1,0] > 2.35) and (self.cam[1,0] < 2.36):
			w = vJinv * (self.v_lambda * self.iLs * rr)
			if(self.diff<0):
				w[0,0] = -30
				w[1,0] = -30
				w[2,0] = -30
			else:
				w[0,0] = 30
				w[1,0] = 30
				w[2,0] = 30
			w = w * self.v_lambda[2,2]
			print "LOST THE BALL"
		elif(rr.norm()<15):
			rr[0, 0] = rr[0,0] * 0
			rr[1, 0] = rr[1,0] * 0
			rr[2, 0] = rr[2,0] * 0
			w = vJinv * (self.v_lambda * self.iLs * rr)
		else:
			w = vJinv * (self.v_lambda * self.iLs * rr)
			print "VISUAL SERVOING"
		self.lastcm = self.cam[1,0]
		self.pwm_leveling1(w)	

	def ball_chaser(self):
		w = sp.Matrix([[0,0,0]]).transpose()
		if(self.cam[0,0] > 150) and (self.cam[0,0] < 320):
			w[0,0] = 0#w1 belakang
			w[1,0] = 300#w2 kanan
			w[2,0] = -300#w3 kiri
		elif(self.cam[0,0] > 0) and (self.cam[0,0] <= 150):
			w[0,0] = 0#w1 belakang
			w[1,0] = 120#w2 kanan
			w[2,0] = 120#w3 kiri
		elif(self.cam[0,0] > 320):
			w[0,0] = 0#w1 belakang
			w[1,0] = -120#w2 kanan
			w[2,0] = -120#w3 kiri
		self.pwm_leveling(w)	
		
	def control(self):
		J = self.jacobianW(self.th, self.Jr)
		self.Jinv = J.inv()
		#==================== compute error ==================================
		#desired 0 - 180
		if(self.x_des[2, 0] >= 0) and (self.x_des[2, 0] <= pi):
			self.Er = self.x_des - self.x	
			if(self.x[2,0] > pi) and (self.x[2,0] < 2*pi):
				self.Er[2,0] = self.Er[2,0] % (2*pi)	
		#desired 181 - 359
		elif(self.x_des[2, 0] > pi) and (self.x_des[2, 0] < 2*pi):
			self.Er = self.x_des - self.x	
			if(self.x[2,0] >= 0) and (self.x[2,0] < pi):
				self.Er[2,0] = self.Er[2,0] - (2*pi)
		if(self.Er.norm()<0.02):
			self.Er = self.Er * 0.
			self.detik = 0
		else:
			self.detik = self.detik + 1
		#print self.Er
		#rospy.loginfo(er)
		#===================== compute w for motor ============================
		w = self.kp*self.Jinv*self.Er
		print self.detik
		self.pwm_leveling(w)	

	def stop_motion(self):
		w = sp.Matrix([[0., 0., 0.]]).transpose()
		self.pwm_leveling(w)

	def go_to_target(self):
		J = self.jacobianW(self.th, self.Jr)
		self.Jinv = J.inv()
		#==================== compute error ==================================
		#desired 0 - 180
		if(self.x_des[2, 0] >= 0) and (self.x_des[2, 0] <= pi):
			self.Er = self.x_des - self.x	
			if(self.x[2,0] > pi) and (self.x[2,0] < 2*pi):
				self.Er[2,0] = self.Er[2,0] % (2*pi)	
		#desired 181 - 359
		elif(self.x_des[2, 0] > pi) and (self.x_des[2, 0] < 2*pi):
			self.Er = self.x_des - self.x	
			if(self.x[2,0] >= 0) and (self.x[2,0] < pi):
				self.Er[2,0] = self.Er[2,0] - (2*pi)
		if(self.Er.norm()<0.1):
			self.Er = self.Er * 0.
			self.detik = 0
			tendang.data = 1
			pub1.publish(tendang)
		else:
			self.detik = self.detik + 1
		#===================== compute w for motor ============================
		w = self.kp*self.Jinv*self.Er
		self.pwm_leveling(w)		

	def main(self):
		#print self.skill
		if(self.skill == 0):#command stop
			self.kick = 2
			pwm.y = 0
			pwm.z = 0
			pwm.x = 0
			pub.publish(pwm)
			self.kick = 0
			#print "ROBOT STOP"

		elif(self.skill == 1):#command ball_finder
			if(self.detik < 10):
				self.kp = self.lamda[0,0]*0.25
			elif(self.detik < 30):
				self.kp = self.lamda[0,0]*0.5
			elif(self.detik < 50):
				self.kp = self.lamda[0,0]*0.75
			else:
				self.kp = self.lamda[0,0]
			if(self.ball1 == 1):
				if(self.kick == 0):
					tendang.data = 2
					pub1.publish(tendang)
					self.kick =1
				else:
					self.go_to_target()
					print "GO TO TARGET"
			elif(self.ball == 1):
				self.kick = 1
				self.control()
			elif(self.cam[2,0] > -45):
				self.visual_servoing()
			elif(self.cam[2,0] <=-45):
				self.ball_chaser()
				print "BALL CHASER"
			self.index_lock = 0
	
		elif(self.skill == 2):#command position
			if(self.detik < 10):
				self.kp = self.lamda[0,0]*0.25
			elif(self.detik < 30):
				self.kp = self.lamda[0,0]*0.5
			elif(self.detik < 50):
				self.kp = self.lamda[0,0]*0.75
			else:
				self.kp = self.lamda[0,0]
			self.index_lock = 0
			self.control()
			#print "ROBOT POSITIONING"

		elif(self.skill == 3):#command ball_finder
			if(self.detik < 10):
				self.kp = self.lamda[0,0]*0.25
			elif(self.detik < 30):
				self.kp = self.lamda[0,0]*0.5
			elif(self.detik < 50):
				self.kp = self.lamda[0,0]*0.75
			else:
				self.kp = self.lamda[0,0]
			if(self.ball1 == 1):
				self.go_to_target()
			elif(self.cam[2,0] > -45):
				self.visual_servoing()
			elif(self.cam[2,0] <=-45):
				self.ball_chaser()
			print "PENALTY"
	
if __name__ == "__main__":
	rospy.init_node("robot2_kinematic_node")
	pub = rospy.Publisher("robot2/pwm_val", Vector3, queue_size=2)
	pub1 = rospy.Publisher("robot2/kick_ball", Int32, queue_size=2)
	pub2 = rospy.Publisher("robot2/command_clear", Int32, queue_size=2)
	pwm = Vector3()
	er = Float32()
	odom = Vector3()
	tendang = Int32()
	gar = Int32()
	server = Server()
	try:
		rospy.Subscriber('robot2/skill', Float32MultiArray, server.get_skill)
		rospy.Subscriber('robot2/odometry', Vector3, server.get_odom)
		rospy.Subscriber('robot2/camera', Vector3, server.get_camera)
		rospy.Subscriber('robot2/ball_position', Int32, server.get_ball_info1)
		rospy.Subscriber('robot1/ball_position', Int32, server.get_ball_info)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass