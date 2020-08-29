#! /usr/bin/python
import lcm
from time import sleep
import sys
sys.path.append("lcmtypes")

from lcmtypes import mbot_encoder_t
from lcmtypes import mbot_imu_t
from lcmtypes import mbot_motor_command_t
from lcmtypes import odometry_t
from lcmtypes import pose_xyt_t
from lcmtypes import robot_path_t
from lcmtypes import timestamp_t
import math

#import numpy as np

class WaypointFollower():
    def __init__(self):
        """ Setup LCM and subscribe """
        self.lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=2")
		self.wc = lcm.LCM("udpm://239.255.76.67:7667?ttl=2")
       	self.lcm_sub = self.lc.subscribe("ODOMETRY", self.odometry_handler)
		self.gui_sub = self.wc.subscribe("CONTROLLER_PATH", self.waypt_handler)
		self.waypoints = [ [1.0,0.0],[1.0,1.0],[0.0,1.0],[0.0,0.0]]
        self.wpt_num = 1
        self.wpt_thresh = 0.1
        self.x = 0
        self.y = 0
		self.path_l = 0
		self.path = []
        self.theta = 0
		#self.wp_theta = 0
		self.pi = 3.14159261
		self.D2R = self.pi / 180
           
    # Called to update x,y,theta
    def odometry_handler(self, channel, data):
		msg = odometry_t().decode(data)
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

    def waypt_handler(self, channel, data):
        print("Handling Waypoints")
		msg = robot_path_t().decode(data)
        self.path_l = msg.path_length
        self.path = msg.path        

    def motor_cmd_publish(self, fwd_vel, turn_vel):
        msg = mbot_motor_command_t()
        # msg.utime = 0.0 # Not sure if we need to change this
        msg.trans_v = fwd_vel
        msg.angular_v = turn_vel
        self.lc.publish("MBOT_MOTOR_COMMAND",msg.encode())


    def go_to(self, waypoint):
		turn_vel = 1
		fwd_vel = .2
		wpt_thresh = 0.05
		self.lc.handle()
		x_diff = waypoint[0]-self.x
		y_diff = waypoint[1]-self.y
		print(x_diff,y_diff)
		theta_goal = math.atan2(y_diff, x_diff)
		# Rotate until proper angle
		theta_err = 999999999
		print("starting to turn")
		while theta_err > 6*self.D2R:
			theta_err = abs(self.theta - theta_goal)
			print("theta goal,theta err: " + str(theta_goal)+" " +str(theta_err))
			self.lc.handle()
			if self.theta - theta_goal > 0:
				dir = -1
			else:
				dir = 1
			if abs(self.theta - theta_goal) > self.pi:
				dir = dir * -1
			self.motor_cmd_publish(0,turn_vel*dir)

		# Facing the right direction, now drive straight until proper position
		pos_err = 99999999
		turn_kp = -2
		print("Starting to go straight")
		while pos_err > wpt_thresh:
			print(self.x - waypoint[0],self.y - waypoint[1])
				pos_err = ( (self.x - waypoint[0])**2 + (self.y - waypoint[1])**2)
				print("Pos err: " + str(pos_err))
				self.lc.handle()
				theta_err = self.theta - theta_goal
			turn_vel = turn_kp * theta_err
			if abs(theta_err) > self.pi:
			turn_vel = turn_vel * -1/10
			self.motor_cmd_publish(fwd_vel, turn_vel)

   
    def follow_wp(self):
	turn_vel = 1
	fwd_vel = .2
	wpt_thresh = 0.05
	self.lc.handle()
	print("SP",self.x, self.y)
	print("WP",self.wp_x, self.wp_y)
	x_diff = self.wp_x-self.x
	y_diff = self.wp_y-self.y
	theta_goal = math.atan2(y_diff, x_diff)
	# Rotate until proper angle
	theta_err = 999999999
        print("starting to turn")
        while theta_err > 6*self.D2R:
            theta_err = abs(self.theta - theta_goal)
            print("theta goal,theta err: " + str(theta_goal)+" " +str(theta_err))
            self.lc.handle()
 	    if self.theta - theta_goal > 0:
	       dir = -1
	    else:
	       dir = 1
	    if abs(self.theta - theta_goal) > self.pi:
		dir = dir * -1
            self.motor_cmd_publish(0,turn_vel*dir)

	 # Facing the right direction, now drive straight until proper position
	pos_err = 99999999
        print("Starting to go straight")
        while pos_err > wpt_thresh**2:
             pos_err = ( (self.x - self.wp_x)**2 + (self.y - self.wp_y)**2)
             print("Pos err: " + str(pos_err))
             self.lc.handle()
	     theta_err = self.theta - theta_goal
	     turn_vel = turn_kp * theta_err
	     if abs(theta_err) > self.pi:
		turn_vel = turn_vel * -1/10
             self.motor_cmd_publish(fwd_vel,0)

    '''	
    def drive_square(self):
        # RTR for each waypoint
	self.lc.handle()
        for waypoint in self.waypoints:
            # Go straight first with while loop
	    pos_err = 99999999
            #while abs(self.x - waypoint[0]) > self.wpt_thresh and abs(self.y - waypoint[1]) > self.wpt_thresh :
            print("Starting to go straight")
	    while pos_err > self.wpt_thresh**2:
		pos_err = ( (self.x - waypoint[0])**2 + (self.y - waypoint[1])**2)
		print("Pos err: " + str(pos_err))
		self.lc.handle()
		self.motor_cmd_publish(.2,0)
		#print("X: " + str(self.x))
		#print("Y: " + str(self.y))
                #print("Theta: " + str(self.theta))


            # Turn to desired heading with while loop
	    theta_err = 999999999
	    print("starting to turn")
            #while abs (self.theta - self.wpt_num * self.pi/2 ) > 10*self.D2R :
	    while theta_err > 10*self.D2R:
                theta_err = abs(self.theta - self.wpt_num * self.pi/2)
		print("theta err: " + str(theta_err))
		self.lc.handle()
		self.motor_cmd_publish(0,-2)
            self.wpt_num += 1
                #print("Made it. Turning")
    '''

    def stop(self):
	self.motor_cmd_publish(0,0)

if __name__ == "__main__":
    WPF = WaypointFollower()
    WPF.wc.handle()
    for i in range(WPF.path_l):
	if not i==len(WPF.path)-1:
	    WPF.wp_x = WPF.path[i+1].x
	    WPF.wp_y = WPF.path[i+1].y
	    WPF.go_to([WPF.wp_x,WPF.wp_y])
#    for waypoint in WPF.waypoints:
#	WPF.go_to(waypoint)
            WPF.stop()
#    while(True):
#	WPF.follow_wp()
#	WPF.stop()
	
