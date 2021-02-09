import numpy as np
import RPi.GPIO as GPIO
import datetime
import time
import json
import numpy as np
from threading import Thread

#from Sensor import MultiSensor
from wheels import Wheels
#from grid_map import Grid

class Robot:
    def __init__():
        #self.multisensor = MultiSensor(1, 90, [11],[13],5,15)   #num_of_sensors,sweep_angle,TRIG,ECHO,servo,step
        self.wheels = Wheels()
        self.location = (0,0)
        #self.map = Grid(2,0.75) #resolution,ratio for weighted avg
        
        
robot = Robot()
i = input()
while True:
    if i == "w":robot.wheels.goForward(10)
    elif i == "s":robot.wheels.goBackward(10)
    elif i == "a":robot.wheels.turnLeft()
    elif i == "d":robot.wheels.turnRight()
    