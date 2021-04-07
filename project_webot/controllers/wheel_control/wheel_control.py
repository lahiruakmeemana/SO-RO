"""wheel_control controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import cv2
from Robot import Soro
from controller import Robot,Motor,Keyboard
from wheels import Wheels
from Sensor import MultiSensor
from point_grid import Point_grid
import matplotlib.pyplot as plt
import time

robot = Robot()

robot = Soro(robot)    

timestep = 32 



robot.run()
