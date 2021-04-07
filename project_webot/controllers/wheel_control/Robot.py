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

from Sensor import MultiSensor
from wheels import Wheels
from grid_map import Grid
from point_grid import Point_grid
from pathplan import pathplanning

from controller import Robot,Motor

class Soro():
    def __init__(self,robot):
        self.robot = robot
        self.multisensor = MultiSensor(self.robot,90,5)   #num_of_sensors,sweep_angle,TRIG,ECHO,servo,step
        self.wheels = Wheels(self.robot)
        self.location = (0,0)
        self.direction = 0 #[0,90,180,270]-->[x+ dir, y+ dir, x- dir, y- dir]
        
        self.map_resolution = 4
        #self.map = Grid(self.map_resolution,0.75) #resolution,ratio for weighted avg
        self.map_completed = False
        self.visited = [self.location]
        self.load_map()
        self.multisensor.setAngle(0)
    def inputdata(self,data):
        point_map = Point_grid(data,self.direction,self.map_resolution)
        point,pointgrid = point_map.get_grid(flip=False)
        self.map.update_pointgrid(point,pointgrid)
        self.show()
    
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
                        if show:
                            
                            fig = plt.figure()
                            #temp_map[temp_map>0.5] += 1
                            #temp_map[temp_map<0.5] -= 0.5
                            #temp_map[p[1]+cen[1]:p[1]+cen[1]+5,p[0]+cen[0]:p[0]+cen[0]+5] = -1
                            plt.imshow(temp_map, cmap = "PiYG_r")
                            plt.clim(0,1)
                            
                            plt.gca().set_xticks(np.arange(-.5, temp_map.shape[1], 2), minor = True)
                            plt.gca().set_yticks(np.arange(-.5, temp_map.shape[0], 2), minor = True)
                            plt.grid(True, which="minor", color="w", linewidth = .6, alpha = 0.5)
                            plt.colorbar()
                            plt.show()
                        return [curr[0]*self.map.resolution,curr[1]*self.map.resolution]
                
                    if (p not in neighbors) and (temp_map[p[1]+cen[1],p[0]+cen[0]] <= 0.5):
                        neighbors.append(p)
            
            i += 1
        
        return (0,0)
        
    def pathplan(self,start,end,animation=False):
        start = (start[0]//self.map.resolution,start[1]//self.map.resolution)
        end = (end[0]//self.map.resolution,end[1]//self.map.resolution)
        dir,dis = pathplanning(self.robot,self.map,start,end,self.map.resolution,animation)
        dis = np.array(dis)*self.map.resolution
        print(dir,dis)
        return dir,dis.tolist()#return a distances list and directions
    
    def movetopoint(self,next_loc):
        #control wheels
        #update location and direction
        directions,distances = self.pathplan(self.location,next_loc,animation=False)
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
                if not(self.wheels.goForward(dis//100,self.multisensor)):
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
        
        #try:
        while not(self.map_completed):
            loc, pmap = self.build_point_grid(self.location)
            self.map.update_pointgrid(loc,pmap)
            self.show()
            next_loc = self.frontier()
            print('Next loc: ',next_loc)
            print('curr loc: ',self.location,'curr dir: ',self.direction)
            if next_loc == (0,0): 
                self.map_completed = True
                print("Mapping finished")
                self.map.show()
                self.save_map("complete_map")
            
            moved = self.movetopoint(next_loc)
                
                
        # except:
            # #self.save_map("partial_map")
            # print('Error',sys.exc_info())
            # print("Current map saved")
     
    def run_manually_moving(self):
        
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
              
        
      
    def prepath(self): #without frontier. without wheels
        path = [(0,90),(1,0),(0,180),(2.25,0),(0,270),(2.25,0)]
        for dis,dir in path:
            if dis==0:
                self.direction = (self.direction+dir)%360
                self.wheels.turn(dir)
            else:
                i = int(dis/0.25)
                for _ in range(i):
                    self.wheels.goForward(0.25)
                    self.location = (self.location[0]+(0.25*np.cos(np.deg2rad(self.direction))),self.location[1]+(0.25*np.sin(np.deg2rad(self.direction))))
                    loc, pmap = self.build_point_grid(self.location)
                    self.map.update_pointgrid(loc,pmap)
                    self.show()
            print("current location: ",self.location,'direction: ',self.direction)
        
                
        
   


