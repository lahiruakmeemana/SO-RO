import numpy as np
#import RPi.GPIO as GPIO
import datetime
import time
import json
import numpy as np
from threading import Thread

#from Sensor import MultiSensor
#from wheels import Wheels
from grid_map import Grid

def load_json(input_file):
    with open(input_file) as f:
    
        all_points = json.load(f)
    return all_points

class Robot:
    def __init__(self):
        #self.multisensor = MultiSensor(1, 90, [11],[13],5,15)   #num_of_sensors,sweep_angle,TRIG,ECHO,servo,step
        #self.wheels = Wheels()
        self.location = (0,0)
        self.direction = 0 #[0,90,180,270]-->[x+ dir, y+ dir, x- dir, y- dir]
        self.map = Grid(2,0.75) #resolution,ratio for weighted avg
        
    def inputdata(self,data):
        self.map.input_rawdata(data)
    def show(self):
        self.map.show()
    
    
    def frontier(self):
        return #return a point to robot to go
        
    def pathplan(self,start,end):
        return #return a distances list and directions
    
    def movetopoint(self,distances,directions)
        #control wheels
        #update location and direction
        return
    
robot = Robot()

# while True:
    # i = input()
    # if i == "w":robot.wheels.goForward(10)
    # elif i == "s":robot.wheels.goBackward(10)
    # elif i == "a":robot.wheels.turnLeft()
    # elif i == "d":robot.wheels.turnRight()
robot.inputdata(load_json('testdata.json'))
robot.show()   