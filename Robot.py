import numpy as np
import sys
import datetime
import time
import json
import numpy as np
import matplotlib.pyplot as plt
from threading import Thread
import sys
import os

#import RPi.GPIO as GPIO
#from Sensor import MultiSensor
#from wheels import Wheels
from grid_map import Grid
from point_grid import Point_grid
from pathplan import pathplanning

# GPIO.setmode(GPIO.BOARD)
# GPIO.setwarnings(False)

class Robot:
    def __init__(self):
        #self.multisensor = MultiSensor(4, 90, [7,13,15,19],[37,38,35,40],5,5)   #num_of_sensors,sweep_angle,TRIG,ECHO,servo,step
        #self.wheels = Wheels()
        self.location = (0,0)
        self.direction = 0 #[0,90,180,270]-->[x+ dir, y+ dir, x- dir, y- dir]
        
        self.map_resolution = 4
        #self.map = Grid(self.map_resolution,0.75) #resolution,ratio for weighted avg
        self.map_completed = False
        self.visited = [self.location]
    def inputdata(self,data):
        point_map = Point_grid(data,self.direction,self.map_resolution)
        point,pointgrid = point_map.get_grid(flip=False)
        self.map.update_pointgrid(point,pointgrid)
        #self.show()
    def show(self):
        self.map.show()
    
    def smoothen(self): #ML part
        return
    
    def build_point_grid(self,point):
        point_data = self.multisensor.turnAndGetDistance(point,self.direction)
        point_map = Point_grid(point_data,self.direction,self.map_resolution)
        #do smoothing
        return point_map.get_grid(flip=False) #smoothen map
    
    def frontier(self,show=False):
        temp_map = self.map.get_grid(flip=False)
        
        loc = np.array(self.location)
        cen = np.array(self.map.center)
        s = 1
        neighbors = (np.array([(s,0),(0,s),(-s,0),(0,-s)]) + loc).tolist()
        
        i = 0
        while len(neighbors):
            curr = neighbors[i]
            
            if (temp_map[curr[1]+cen[1],curr[0]+cen[0]]<=0.5):
                curr_neighbors = [[curr[0]+1,curr[1]],[curr[0],curr[1]+1],[curr[0]-1,curr[1]],[curr[0],curr[1]-1]]
                for p in curr_neighbors:
                    if (temp_map[p[1]+cen[1],p[0]+cen[0]] == 0.5) and (curr not in self.visited):
                        self.visited.append(curr)
                        # if show:
                            
                            # fig = plt.figure()
                            # #temp_map[temp_map>0.5] += 1
                            # #temp_map[temp_map<0.5] -= 0.5
                            # #temp_map[p[1]+cen[1]:p[1]+cen[1]+5,p[0]+cen[0]:p[0]+cen[0]+5] = -1
                            # plt.imshow(temp_map, cmap = "PiYG_r")
                            # plt.clim(0,1)
                            
                            # plt.gca().set_xticks(np.arange(-.5, temp_map.shape[1], 2), minor = True)
                            # plt.gca().set_yticks(np.arange(-.5, temp_map.shape[0], 2), minor = True)
                            # plt.grid(True, which="minor", color="w", linewidth = .6, alpha = 0.5)
                            # plt.colorbar()
                            # plt.show()
                        return [curr[0]*self.map.resolution,curr[1]*self.map.resolution]
                
                    if (p not in neighbors):
                        neighbors.append(p)
            
            i += 1
        
        return (0,0)
        
    def pathplan(self,start,end,animation=False):
        start = (start[0]//self.map.resolution,start[1]//self.map.resolution)
        end = (end[0]//self.map.resolution,end[1]//self.map.resolution)
        dir,dis = pathplanning(self.map,start,end,self.map.resolution,animation)
        dis = np.array(dis)*self.map.resolution
        print(dir,dis)
        return dir,dis.tolist()#return a distances list and directions
    
    def movetopoint(self,next_loc):
        #control wheels
        #update location and direction
        directions,distances = self.pathplan(self.location,next_loc)
        #directions,distances = np.array([180,90,0,90]),np.array([20,15,25,30])
        if (directions[0],distances[0]) == (0,0):
            print("No path found")
            return -1
        directions = directions  - self.direction
        directions[directions==[-90]] = 270
        directions[directions==[-180]] = 180
        directions[directions==[-270]] = 90
        for dir,dis in zip(directions,distances):
            input()
            if self.wheels.turn(dir):
                if not(self.wheels.goForward(dis,self.multisensor)):
                    return 0
                
            else:
                print("Wrong direction")
                return 0
        else:
            self.direction = dir
            self.location = next_loc
            print("moved to",next_loc)
            
        print("moved")
        return 1
    
    # def load_json(self,input_file):
        # with open(input_file) as f:
        
            # save = json.load(f)
            
        # return map
    
    def load_map(self):
        self.map = Grid(self.map_resolution,0.75)
        try:
            self.map = self.map.load_saved("complete_map.json")
            print("Complete map loaded")
            self.map_completed = True
        except: 
            try:    
                self.map = self.load_saved("partial_map.json")
                print("Partially completed map loaded")
            except:
                pass
        return
        
    def save_map(self,string):
        with open(string+".json", 'w') as outfile:
            
            save = {"min_x": self.map.min_x,
                    "min_y": self.map.min_y,
                    "max_x": self.map.max_x,
                    "max_y": self.map.max_y,
                    "resolution": self.map.resolution,
                    "center":self.map.center,
                    "ratio": self.map.ratio,
                    "map": self.map.get_grid(flip=False).tolist()}
            json.dump(save, outfile)
            print("Map saved")
        return
        
    def run(self):
        self.load_map()
        
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
     
    def run_manually_moving(self):
        self.load_map()
        
        while not(self.map_completed):
                #try:
                loc, pmap = self.build_point_grid(self.location)
                self.map.update_pointgrid(loc,pmap,show=True)
                
                next_loc = self.frontier()
                print(next_loc)
                if next_loc == (0,0): 
                    self.map_completed = True
                    print("Mapping finished")
                    self.map.show()
                    self.save_map("complete_map")
                
                else:
                    moved = int(input("Manually moved? (1/0) "))
                    if moved == 0:
                        sys.exit()
                        break
                    elif moved == 1:
                        self.location = next_loc
                #except:
                #self.save_map("partial_map")
                #exc_type, exc_obj, exc_tb = sys.exc_info()
                #fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
                #print(exc_type, fname, exc_tb.tb_lineno)
                #print('Error',sys.exc_info(),sys.exc_info()[2].tb_lineno)
                #print("Current map saved")
                #sys.exit()
    def drive(self):
        try:       
            while True:
                print(self.location,self.direction)
                i,dis = input().split()
                
                if i == "w":
                    dis = int(dis)
                    if (self.wheels.goForward(dis,self.multisensor)):
                        self.location = (self.location[0]+round((dis * np.cos(np.deg2rad(self.direction)))),\
                                        self.location[1]+round((dis * np.sin(np.deg2rad(self.direction)))))
                        input()
                        loc, pmap = self.build_point_grid(self.location)
                        self.map.update_pointgrid(loc,pmap)
                elif i == "s":
                    self.wheels.turn(180)
                    self.direction = (self.direction + 180)%360
                elif i == "a":
                    self.wheels.turn(90)
                    self.direction = (self.direction + 90)%360
                elif i == "d":
                    self.wheels.turn(270)
                    self.direction = (self.direction + 270)%360
        except:
            self.multisensor.cleanup() 
            self.show()
            self.save_map("complete_map")
              
        
      
    def manual(self): #without frontier. without wheels
        self.load_map()
        while True:
            try:
                in_ = input()
                #print(in_)
                loc_x,loc_y, dir = map(int, in_.split())
                
                self.location = (loc_x,loc_y)
                self.direction = dir
                loc, pmap = self.build_point_grid(self.location)
                self.map.update_pointgrid(loc,pmap)
                self.show()
            except:
                if in_=="1":
                    self.map_completed = True
                    print("Mapping finished")
                    self.show()
                    self.save_map("complete_map")
                elif in_ == "0":
                    print("Mapping paused. Partial map saved")
                    self.show()
                    #self.save_map("partial_map")
                self.multisensor.cleanup()
                exc_type, exc_obj, exc_tb = sys.exc_info()
                fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
                print(exc_type, fname, exc_tb.tb_lineno)
                print('Error',sys.exc_info(),sys.exc_info()[2].tb_lineno)
                sys.exit()
'''
robot = Robot()
robot.load_map()
robot.drive()
          
'''    
robot = Robot()
robot.load_map()
robot.inputdata({"point": [0, 0], "direction":90, "angles": [45, 135, 225, 315, 50, 140, 230, 320, 55, 145, 235, 325, 60, 150, 240, 330, 65, 155, 245, 335, 70, 160, 250, 340, 75, 165, 255, 345, 80, 170, 260, 350, 85, 175, 265, 355, 90, 180, 270, 0, 95, 185, 275, 5, 100, 190, 280, 10, 105, 195, 285, 15, 110, 200, 290, 20, 115, 205, 295, 25, 120, 210, 300, 30, 125, 215, 305, 35, 130, 220, 310, 40, 135, 225, 315, 45], "distances": [205, 121, 87, 69, 208, 72, 89, 87, 157, 104, 93, 86, 169, 187, 94, 85, 102, 119, 47, 85, 54, 118, 46, 87, 52, 118, 45, 87, 52, 123, 44, 86, 51, 231, 45, 112, 51, 217, 44, 157, 51, 241, 44, 250, 51, 241, 44, 250, 52, 201, 44, 195, 51, 113, 44, 250, 52, 115, 44, 158, 53, 112, 44, 183, 54, 113, 45, 173, 57, 113, 46, 115, 201, 99, 117, 137], "dis_probs": [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]}) 
robot.inputdata({"point": [50, 0], "direction":90, "angles": [45, 135, 225, 315, 50, 140, 230, 320, 55, 145, 235, 325, 60, 150, 240, 330, 65, 155, 245, 335, 70, 160, 250, 340, 75, 165, 255, 345, 80, 170, 260, 350, 85, 175, 265, 355, 90, 180, 270, 0, 95, 185, 275, 5, 100, 190, 280, 10, 105, 195, 285, 15, 110, 200, 290, 20, 115, 205, 295, 25, 120, 210, 300, 30, 125, 215, 305, 35, 130, 220, 310, 40, 135, 225, 315, 45], "distances": [171, 86, 62, 92, 169, 229, 53, 90, 166, 221, 51, 89, 56, 219, 50, 91, 53, 156, 50, 90, 53, 167, 50, 91, 53, 220, 50, 91, 52, 204, 50, 92, 52, 237, 51, 93, 52, 182, 52, 92, 52, 182, 55, 126, 52, 181, 55, 164, 52, 181, 55, 160, 57, 83, 56, 119, 149, 80, 56, 115, 152, 79, 57, 105, 196, 71, 78, 118, 251, 65, 62, 107, 235, 64, 82, 92], "dis_probs": [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]})
robot.inputdata({"point": [100, 0], "direction":90, "angles": [45, 135, 225, 315, 50, 140, 230, 320, 55, 145, 235, 325, 60, 150, 240, 330, 65, 155, 245, 335, 70, 160, 250, 340, 75, 165, 255, 345, 80, 170, 260, 350, 85, 175, 265, 355, 90, 180, 270, 0, 95, 185, 275, 5, 100, 190, 280, 10, 105, 195, 285, 15, 110, 200, 290, 20, 115, 205, 295, 25, 120, 210, 300, 30, 125, 215, 305, 35, 130, 220, 310, 40, 135, 225, 315, 45], "distances": [162, 110, 82, 58, 165, 166, 82, 59, 184, 106, 83, 59, 163, 136, 84, 60, 106, 140, 67, 72, 55, 137, 66, 80, 53, 271, 67, 93, 52, 270, 73, 90, 52, 265, 58, 122, 51, 283, 56, 114, 51, 283, 55, 151, 52, 284, 55, 151, 52, 266, 55, 151, 52, 179, 56, 151, 53, 124, 56, 152, 53, 84, 54, 117, 54, 87, 55, 79, 115, 84, 56, 86, 169, 83, 54, 78], "dis_probs": [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]}) 
robot.inputdata({"point": [150, 0], "direction":90, "angles": [45, 135, 225, 315, 50, 140, 230, 320, 55, 145, 235, 325, 60, 150, 240, 330, 65, 155, 245, 335, 70, 160, 250, 340, 75, 165, 255, 345, 80, 170, 260, 350, 85, 175, 265, 355, 90, 180, 270, 0, 95, 185, 275, 5, 100, 190, 280, 10, 105, 195, 285, 15, 110, 200, 290, 20, 115, 205, 295, 25, 120, 210, 300, 30, 125, 215, 305, 35, 130, 220, 310, 40, 135, 225, 315, 45], "distances": [151, 56, 80, 122, 160, 229, 93, 119, 179, 198, 85, 109, 178, 102, 86, 105, 104, 98, 84, 105, 54, 98, 45, 103, 54, 99, 43, 103, 53, 110, 43, 103, 52, 113, 42, 103, 52, 112, 43, 103, 52, 127, 42, 102, 52, 136, 42, 102, 52, 126, 42, 103, 53, 125, 42, 103, 53, 125, 43, 103, 101, 124, 43, 47, 56, 124, 43, 52, 191, 123, 44, 47, 192, 107, 60, 47], "dis_probs": [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]}) 
robot.inputdata({"point": [200, 0], "direction":90, "angles": [45, 135, 225, 315, 50, 140, 230, 320, 55, 145, 235, 325, 60, 150, 240, 330, 65, 155, 245, 335, 70, 160, 250, 340, 75, 165, 255, 345, 80, 170, 260, 350, 85, 175, 265, 355, 90, 180, 270, 0, 95, 185, 275, 5, 100, 190, 280, 10, 105, 195, 285, 15, 110, 200, 290, 20, 115, 205, 295, 25, 120, 210, 300, 30, 125, 215, 305, 35, 130, 220, 310, 40, 135, 225, 315, 45], "distances": [76, 144, 118, 63, 112, 175, 126, 64, 80, 183, 90, 65, 70, 143, 45, 57, 58, 195, 44, 56, 54, 159, 42, 55, 54, 193, 42, 55, 53, 168, 41, 55, 53, 162, 41, 55, 53, 161, 41, 54, 53, 167, 41, 54, 53, 170, 44, 54, 53, 151, 41, 55, 53, 127, 41, 55, 53, 124, 41, 55, 54, 123, 42, 56, 228, 125, 43, 60, 212, 125, 43, 73, 227, 104, 65, 71], "dis_probs": [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]})
robot.show()
# robot.map.map[65,65:135] = 1
    
# robot.map.map[88,40:135] = 1

# robot.map.map[65:88,135] = 1

# robot.map.map[40:88,40] = 1
    
# robot.map.map[40:85,65] = 1
    
# robot.map.map[40,40:65] = 1
    
# robot.map.map[41:84,41:65] = 0.2
# robot.map.map[66:88,41:134] =0.2
robot.show()
robot.save_map("pathplan_eg")


#path plan example
'''
robot = Robot()
robot.load_map()
robot.map.load_saved("pathplan_eg.json")
robot.show()
robot.pathplan((200,0),(-80,-120),animation=True)
'''