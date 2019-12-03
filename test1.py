#!/usr/bin/env python

from math import pi, sqrt, atan2, cos, sin
import matplotlib.pyplot as plt

import rospy
import tf
import numpy as mp
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

GPIO.setup("#PIN_NUM#", GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

self.lastpoint = 0

class Arm():
    def __init__(self):
        rospy.init_node("arm_move")
        rospy.loginfo("Press CTRL + C to terminate")
        rospy.on_shutdown(self.stop)

	
        self.rate = rospy.Rate(10)
        

    def run(self):
        waypoints = self.get_path_from_A_star()
        for i in range(len(waypoints)):
            if i < (len(waypoints) - 1):
                self.move_to_point(waypoint[i], waypoints[i+1])
            else:
                self.lastpoint = 1
                self.move_to_point(waypoints[i], "MULTIPLICATIVE INVERSE OF PREV. VECTOR")
        self.stop
        self.loginfo("Arm in position")
        #self.visualization

    def move_to_point(self, point, next_point):
        # Polynomial method of waypoint navigation will be utilized.
        # Polynomial degree will be 5.
        # 5th order polynomial will allow us to set conditions on position, velocity, and
        # acceleration of arm.

    def get_path_from_A_star(self):
        # By defining a start point and desired end point A* will find the optimal path to
        # destination.
        # This path, returned as a list of points, is passed as an argument into the
        # move_to_point function so the arm can follow this path.
        # This will have to be adjusted so the arm travels a shorter distance depth wise.
        # Estimated end point will only be accuarte if IR sensor is within acceptable distance.
        # Algorithm will have to be run multiple times to ensure arm is on correct path as it
        # closer to fruit.

    def Total_cost(self, point, past_cost):
        heuristic_cost = "TAXICAB(L1), Manhattan distance from current point to estimated end point"
        cost = heuristic_cost + past_cost + 1 # Assuming grid size of movement is unit length, cost of travel is 1.
        
        return cost

    def stop(self):
        # Stops all motor movement once destination is reached.

if __name__ == '__main__'
    try:
        Arm()
    except rospy.ROSInterruptException:
        pass
