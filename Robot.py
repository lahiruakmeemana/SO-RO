import numpy as np
import sys
import RPi.GPIO as GPIO
import datetime
import time
import json
import numpy as np
from threading import Thread
import sys

from Sensor import MultiSensor
from wheels import Wheels
from grid_map import Grid
from point_grid import Point_grid
from pathplan import pathplanning

GPIO.setmode(GPIO.BOARD)

def write_json(readings):      
    with open("map.json", 'w') as outfile:
        json.dump(readings, outfile)

def load_json(input_file):
    with open(input_file) as f:
    
        all_points = json.load(f)
    return all_points

class Robot:
    def __init__(self):
        self.multisensor = MultiSensor(4, 90, [7,13,15,19],[37,38,35,40],5,5)   #num_of_sensors,sweep_angle,TRIG,ECHO,servo,step
        self.wheels = Wheels()
        self.location = (0,0)
        self.direction = 90 #[0,90,180,270]-->[x+ dir, y+ dir, x- dir, y- dir]
        
        self.map_resolution = 1
        self.map = Grid(self.map_resolution,0.75) #resolution,ratio for weighted avg
        self.map_completed = False
        self.visited = [self.location]
    def inputdata(self,data):
        point_map = Point_grid(data,self.map_resolution)
        point,pointgrid = point_map.get_grid(flip=False)
        self.map.update_pointgrid(point,pointgrid)
        self.show()
    def show(self):
        self.map.show()
    
    def smoothen(self): #ML part
        return
    
    def build_point_grid(self,point,direction):
        point_data = self.multisensor.turnAndGetDistance(point)
        point_map = Point_grid(point_data,self.direction,self.map_resolution)
        #do smoothing
        return point_map #smoothen map
    
    def frontier(self):
        temp_map = self.map.get_grid()
        
        loc = self.location.copy()
        neighbors = list((loc[0]+1,loc[1]),(loc[0],loc[1]+1),(loc[0]-1,loc[1]),(loc[0],loc[1]-1))
        i = 0
        while len(neighbors):
            curr = neighbors[i]
            if temp_map[curr[1],curr[0]] == 0.5 and curr not in self.visited:
                self.visited.append(curr)
                return curr
            
            curr_neighbors = list((curr[0]+1,curr[1]),(curr[0],curr[1]+1),(curr[0]-1,curr[1]),(curr[0],curr[1]-1))
            for p in curr_neighbors:
                if p not in neighbors:neighbors.append(p)
            i += 1
        
        return (0,0)#return a point to robot to go
        
    def pathplan(self,start,end,animation=False):
        start = (start[0]//self.map.resolution,start[1]//self.map.resolution)
        end = (end[0]//self.map.resolution,end[1]//self.map.resolution)
        dir,dis = pathplanning(self.map,start,end,animation)
        dis = np.array(dis)*self.map.resolution
        return dir,dis.tolist()#return a distances list and directions
    
    def movetopoint(self,next_loc):
        #control wheels
        #update location and direction
        directions,distances = self.pathplan(self.location,next_loc)
        if (directions,distances) == (0,0):
            print("No path found")
            return -1
        directions = directions  - self.direction
        directions[directions==[-90]] = 270
        directions[directions==[-180]] = 180
        directions[directions==[-270]] = 90
        for dir,dis in zip(directions,distances):
            if wheels.turn(dir):
                if not(wheels.goForward(dis,self.multisensor)):
                    return 0
                
            else:
                print("Wrong direction")
                return 0
        else:
            self.direction = dir
            self.location = next_loc
            print("moved to",next_loc)
            
        
        return 1
    
    def load_json(self,input_file):
        with open(input_file) as f:
        
            map = json.load(f)
        return map
    
    def load_map():
        try: 
            self.map = self.load_map("complete_map")
            print("Complete map loaded")
            self.map_completed = True
        except: 
            try:    
                self.map = self.load_map("partial_map")
                print("Partially completed map loaded")
            except:
                self.map = Grid(self.map_resolution,0.75)
    
    def save_map(self,string):
        with open(string+".json", 'w') as outfile:
            grid = self.map.get_grid()
            json.dump(grid.tolist(), outfile)
            
    def run():
        load_map()
        
        while not(self.map_completed):
            try:
                loc, pmap = build_point_grid(self.location)
                self.map.update_pointgrid(loc,pmap)
                
                next_loc = self.frontier()
                if next_loc == (0,0): 
                    self.map_completed = True
                    print("Mapping finished")
                    self.map.show()
                    self.save_map("complete_map")
                
                moved = self.movetopoint(next_loc)
                
                
            except:
                #self.save_map("partial_map")
                print('Error',sys.exc_info())
                print("Current map saved")
     
    def run_manually_moving():
        load_map()
        
        while not(self.map_completed):
            try:
                loc, pmap = build_point_grid(self.location)
                self.map.update_pointgrid(loc,pmap)
                
                next_loc = self.frontier()
                print(next_loc)
                if next_loc == (0,0): 
                    self.map_completed = True
                    print("Mapping finished")
                    self.map.show()
                    self.save_map("complete_map")
                
                else:
                    moved = input("Manually moved? 1/0")
                    
                    
            except:
                #self.save_map("partial_map")
                print('Error',sys.exc_info())
                print("Current map saved")
        
        
robot = Robot()
# directions = [90,180]
# distances = [10,15]
# for dir,dis in zip(directions,distances):
            # if robot.wheels.turn(dir):
                # if not(robot.wheels.goForward(dis,robot.multisensor)):
                    # break
                # print("robot loc:",robot.location," robot dir:",robot.direction)
                
            # else:
                # print("Wrong direction")
                # break
# while True:
    # try:
        # x,y = map(int,input().split())
        # temp = robot.multisensor.turnAndGetDistance((x,y))
        # print(temp)
    # except:
        # robot.multisensor.write_json()
        # print("Unexpected error:", sys.exc_info())
        # print("readings saved")
        # break
        
while True:
    i = input()
    if i == "w":robot.wheels.goForward(10,robot.multisensor)
    elif i == "s":robot.wheels.turn(180)
    elif i == "a":robot.wheels.turn(90)
    elif i == "d":robot.wheels.turn(270)
    else:break
# robot.inputdata([{"point": [1, 0], "angles": [0, 30, 60, 90, 120, 150, 180], "distances": [125.38, 160.64, 104.22, 94.52, 98.82, 222.12, 292.59], "dis_probs": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]}])
# robot.inputdata([{"point": [1, 0.5], "angles": [0, 30, 60, 90, 120, 150, 180], "distances": [120.34, 123.0, 77.57, 76.24, 75.88, 92.31, 292.5], "dis_probs": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]}])
# robot.inputdata([{"point": [2, 1.5], "angles": [0, 30, 60, 90, 120, 150, 180], "distances": [74.59, 79.09, 31.63, 29.85, 33.38, 118.92, 117.32], "dis_probs": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]}])
# robot.inputdata([{"point": [1.5, 1], "angles": [0, 30, 60, 90, 120, 150, 180], "distances": [95.98, 102.92, 56.4, 55.1, 59.66, 88.26, 292.5], "dis_probs": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]}])
# robot.inputdata([{"point": [-1, 0.5], "angles": [0, 30, 60, 90, 120, 150, 180], "distances": [217.83, 89.43, 66.88, 67.1, 69.18, 292.58, 292.62], "dis_probs": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]}])

robot.inputdata({"point": [0, 0], "angles": [45, 135, 225, 315, 50, 140, 230, 320, 55, 145, 235, 325, 60, 150, 240, 330, 65, 155, 
    245, 335, 70, 160, 250, 340, 75, 165, 255, 345, 80, 170, 260, 350, 85, 175, 265, 355, 90, 180, 270, 0, 95, 185, 275, 5, 100, 190, 280, 
    10, 105, 195, 285, 15, 110, 200, 290, 20, 115, 205, 295, 25, 120, 210, 300, 30, 125, 215, 305, 35, 130, 220, 310, 40, 135, 225, 315, 45], 
    "distances": [205, 121, 87, 69, 208, 72, 89, 87, 157, 104, 93, 86, 169, 187, 94, 85, 102, 119, 47, 85, 54, 118, 46, 87, 52, 118, 45, 87, 52,
     123, 44, 86, 51, 231, 45, 112, 51, 217, 44, 157, 51, 241, 44, 250, 51, 241, 44, 250, 52, 201, 44, 195, 51, 113, 44, 250, 52, 115, 44, 158, 
     53, 112, 44, 183, 54, 113, 45, 173, 57, 113, 46, 115, 201, 99, 117, 137], "dis_probs": [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
     1, 1, 1, 1, 1, 1, 1, 1]})
robot.inputdata({"point": [51,0], "angles": [45, 135, 225, 315, 50, 140, 230, 320, 55, 145, 235, 325, 60, 150, 240, 330, 65, 155, 245, 335, 70, 160, 250, 340, 75, 165,
     255, 345, 80, 170, 260, 350, 85, 175, 265, 355, 90, 180, 270, 0, 95, 185, 275, 5, 100, 190, 280, 10, 105, 195, 285, 15, 110, 200, 290, 20, 115, 205, 295,
     25, 120, 210, 300, 30, 125, 215, 305, 35, 130, 220, 310, 40, 135, 225, 315, 45], "distances": [171, 86, 62, 92, 169, 229, 53, 90, 166, 221, 51, 89, 56, 
     219, 50, 91, 53, 156, 50, 90, 53, 167, 50, 91, 53, 220, 50, 91, 52, 204, 50, 92, 52, 237, 51, 93, 52, 182, 52, 92, 52, 182, 55, 126, 52, 181, 55, 164,
     52, 181, 55, 160, 57, 83, 56, 119, 149, 80, 56, 115, 152, 79, 57, 105, 196, 71, 78, 118, 251, 65, 62, 107, 235, 64, 82, 92], "dis_probs": [1, 1, 1, 1, 
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]})
# #robot.show()
# robot.pathplan((0,0),(-75*2,-15*2),animation=True)

#

# write_json(robot.map.get_grid().tolist())
# print('json saved')


