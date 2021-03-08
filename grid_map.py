import numpy as np
import json
import matplotlib.pyplot as plt
import math
import cv2
from prob_functions import sigmoidProbability,expoProbability

class Grid:
    def __init__(self,resolution,ratio):
        
        infinity = np.inf
        self.min_x, self.max_x, self.min_y, self.max_y= 0,0,0,0
        self.resolution=resolution
        #self.size=(int(round((self.max_x - self.min_x)/self.resolution)),int(round((self.max_y - self.min_y)/self.resolution)))
        self.map=np.ones((1,1),dtype=np.float)*0.5
        #sefl.center= (-self.min_x, -self.min_y)
        self.beta = np.deg2rad(5) #cone angle
        self.ratio = ratio
        self.center= (0,0)
        #probabilities
        print('map initialized')
    def get_grid(self,flip=True):
        out = self.map.copy()
        # out[self.map>0.65] = 1
        # out[self.map<0.4] = 0
        out[(out>=0.45) & (out<=0.6)] = 0.5
        if flip==False: return out
        return np.flipud(out)
    
    
        
    def grid_size_update(self,min_x,max_x,min_y,max_y): 
        
        if min_x < self.min_x:
            extend_distance = self.min_x - min_x 
            extending_part = np.ones((self.map.shape[0], extend_distance))*0.5
            self.map = np.hstack((extending_part, self.map))
            self.min_x=min_x 
            self.center=(-self.min_x, -self.min_y)
            
        if min_y < self.min_y:
            extend_distance = self.min_y - min_y 
            extending_part = np.ones((extend_distance, self.map.shape[1]))*0.5
            self.map = np.vstack((extending_part, self.map))
            self.min_y=min_y
            self.center=(-self.min_x, -self.min_y)
            
        if max_x > self.max_x:
            extend_distance = max_x - self.max_x 
            extending_part = np.ones((self.map.shape[0],extend_distance))*0.5
            self.map = np.hstack((self.map, extending_part))    
            self.max_x=max_x
        
        if max_y > self.max_y:
            extend_distance = max_y - self.max_y 
            extending_part = np.ones((extend_distance, self.map.shape[1]))*0.5
            self.map = np.vstack((self.map, extending_part))
            self.max_y=max_y
        print("grid size changed to: (",self.map.shape[1],",",self.map.shape[0],")")
        print(self.center)
     
    def bresenham(self,start, end):
        '''
        Implementation of Bresenham's line drawing algorithm
        See en.wikipedia.org/wiki/Bresenham's_line_algorithm
        Bresenham's Line Algorithm
        Produces a np.array from start and end (original from roguebasin.com)
        '''
        #if possible make this run on numpy array
        # setup initial conditions
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1
        is_steep = abs(dy) > abs(dx)  # determine how steep the line is
        if is_steep:  # rotate line
            x1, y1 = y1, x1
            x2, y2 = y2, x2
        # swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
        dx = x2 - x1  # recalculate differentials
        dy = y2 - y1  # recalculate differentials
        error = int(dx / 2.0)  # calculate error
        y_step = 1 if y1 < y2 else -1
        # iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)  #made this a tuple [x,y] --> (x,y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += y_step
                error += dx
        if swapped:  # reverse the list if the coordinates were swapped
            points.reverse()
        #points = np.array(points)
        return points
    
    def update_pointgrid(self,point,grid):#both maps upside down
        
        min_x,min_y = point[0] - grid.shape[0]//self.resolution , point[1] - grid.shape[1]//self.resolution
        max_x,max_y = point[0] + grid.shape[0]//self.resolution , point[1] + grid.shape[1]//self.resolution
        
        self.grid_size_update(min_x,max_x,min_y,max_y)
        #change grid resolution
        res = 600//self.resolution
        grid = cv2.resize(grid, (res,res), interpolation=cv2.INTER_NEAREST)
        print(grid.shape)
        self.map[min_y+self.center[1]:max_y+1+self.center[1], min_x+self.center[0]:max_x+1+self.center[1]] = \
        self.map[min_y+self.center[1]:max_y+1+self.center[1], min_x+self.center[0]:max_x+1+self.center[1]] *(1-self.ratio) +\
        grid * self.ratio
        
        print("Map updated")
        
        
    
    def update(self,xy_points,distances,angles,dis_prob):#input [xy_points,distances,angles,dis_prob]
        
        assert len(xy_points) == len(distances) == len(dis_prob)
        x, y = np.hsplit(xy_points,2)
        x=x.flatten().astype(np.int) #+ self.center[0]
        y=y.flatten().astype(np.int) #+ self.center[1]
        #print(x,y)
        x1 = ((x + distances * np.cos(angles + self.beta))//self.resolution).astype(np.int)
        y1 = ((y + distances * np.sin(angles + self.beta))//self.resolution).astype(np.int)

        x2 = ((x + distances * np.cos(angles - self.beta))//self.resolution).astype(np.int)
        y2 = ((y + distances * np.sin(angles - self.beta))//self.resolution).astype(np.int)
        #print(np.round((x + distances * np.cos(np.deg2rad(angles))/self.resolution).astype(np.int)))
        
        obs_lines= []
        empty_space= set()
        probs=np.array([])
        min_x,min_y,max_x,max_y = np.inf, np.inf, -np.inf, -np.inf
        obs_line_x=np.array([],dtype=np.int)
        obs_line_y=np.array([],dtype=np.int)
        for i in range(len(x)):
            #for j in range(distances.shape[0]):
            if x1[i]==0:continue
            
            obs_line = self.bresenham((x1[i],y1[i]),(x2[i],y2[i]))
            obs_lines.append(obs_line)
            
            obs_line = np.array(obs_line)
            temp_x,temp_y = np.hsplit(np.array(obs_line),2)
            
            obs_line_x = np.concatenate((obs_line_x, temp_x.flatten()),0)
            obs_line_y = np.concatenate((obs_line_y, temp_y.flatten()),0)
            
            probs = np.concatenate((probs,[dis_prob[i]]*obs_line.shape[0]),0)
        
            temp_min_x, temp_min_y = np.min(obs_line,0)
            temp_max_x, temp_max_y = np.max(obs_line,0)
            
            min_x = min(min_x, temp_min_x)
            min_y = min(min_y, temp_min_y)
            max_x = max(max_x, temp_max_x)
            max_y = max(max_y, temp_max_y)
            
        if (min_x,max_x,min_y,max_y) != (self.min_x, self.max_x, self.min_y, self.max_y):
            self.grid_size_update(min_x,max_x,min_y,max_y)
        
        for i,obs_line in enumerate(obs_lines):
            for point in obs_line:
                empty_space.update(tuple(self.bresenham((x[i]//self.resolution,y[i]//self.resolution), tuple(point))))
        
        
        
        empty_space=np.array(list(empty_space))
        
        empty_space_x ,empty_space_y = np.hsplit(empty_space,2)
        empty_space_x = empty_space_x.flatten() + self.center[0]
        empty_space_y = empty_space_y.flatten() + self.center[1]
        self.map [empty_space_y ,empty_space_x] = self.map [empty_space_y ,empty_space_x] * (1-self.ratio) + 0 #0 prob for empty space
        
        
        obs_line_x = obs_line_x.flatten() + self.center[0]
        obs_line_y = obs_line_y.flatten() + self.center[1]
        self.map [obs_line_y ,obs_line_x] = self.map [obs_line_y ,obs_line_x] * (1-self.ratio) + probs * self.ratio
        return x,y,obs_line_x,obs_line_y
    
    def dis_probs(self,distances):#implement probability
        temp = np.array(distances.copy(),dtype=np.float64)
        probs = sigmoidProbability(temp)
        
        return probs.tolist()
   
    def input_rawdata(self,all_points):
        xy_points = []
        distances = []
        angles = []
        dis_prob = []
        for rec in all_points:
            xy_points.append(rec['point']*len(all_points[0]['angles']))
            distances.append(rec['distances'])
            angles.append(rec['angles'])
            dis_prob.append( self.dis_probs(rec['distances'])) #rec['dis_probs'])
        
        xy_points = np.array(xy_points).reshape(-1,2)
        distances = np.array(distances).flatten()
        angles = np.deg2rad(np.array(angles).flatten())
        dis_prob = np.array(dis_prob).flatten()
        #print(dis_prob)
        return self.update(xy_points,distances,angles,dis_prob)
    
    def show(self):
        out = self.get_grid()
        res = out.shape
        fig=plt.figure(figsize=(10,6))
        plt.imshow(out, cmap = "PiYG_r")
        plt.clim(0.0, 1.0)
        plt.gca().set_xticks(np.arange(-.5, res[1], 2), minor = True)
        plt.gca().set_yticks(np.arange(-.5, res[0], 2), minor = True)
        plt.grid(True, which="minor", color="w", linewidth = .6, alpha = 0.5)
        plt.colorbar()
        plt.show()
    
if __name__ == '__main__':    
    map = Grid(2,0.75)
    map.input_rawdata([{"point": [0, 0], "angles": [45, 135, 225, 315, 50, 140, 230, 320, 55, 145, 235, 325, 60, 150, 240, 330, 65, 155, 
    245, 335, 70, 160, 250, 340, 75, 165, 255, 345, 80, 170, 260, 350, 85, 175, 265, 355, 90, 180, 270, 0, 95, 185, 275, 5, 100, 190, 280, 
    10, 105, 195, 285, 15, 110, 200, 290, 20, 115, 205, 295, 25, 120, 210, 300, 30, 125, 215, 305, 35, 130, 220, 310, 40, 135, 225, 315, 45], 
    "distances": [205, 121, 87, 69, 208, 72, 89, 87, 157, 104, 93, 86, 169, 187, 94, 85, 102, 119, 47, 85, 54, 118, 46, 87, 52, 118, 45, 87, 52,
     123, 44, 86, 51, 231, 45, 112, 51, 217, 44, 157, 51, 241, 44, 250, 51, 241, 44, 250, 52, 201, 44, 195, 51, 113, 44, 250, 52, 115, 44, 158, 
     53, 112, 44, 183, 54, 113, 45, 173, 57, 113, 46, 115, 201, 99, 117, 137], "dis_probs": [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
     1, 1, 1, 1, 1, 1, 1, 1]}, 
    {"point": [51,0], "angles": [45, 135, 225, 315, 50, 140, 230, 320, 55, 145, 235, 325, 60, 150, 240, 330, 65, 155, 245, 335, 70, 160, 250, 340, 75, 165,
     255, 345, 80, 170, 260, 350, 85, 175, 265, 355, 90, 180, 270, 0, 95, 185, 275, 5, 100, 190, 280, 10, 105, 195, 285, 15, 110, 200, 290, 20, 115, 205, 295,
     25, 120, 210, 300, 30, 125, 215, 305, 35, 130, 220, 310, 40, 135, 225, 315, 45], "distances": [171, 86, 62, 92, 169, 229, 53, 90, 166, 221, 51, 89, 56, 
     219, 50, 91, 53, 156, 50, 90, 53, 167, 50, 91, 53, 220, 50, 91, 52, 204, 50, 92, 52, 237, 51, 93, 52, 182, 52, 92, 52, 182, 55, 126, 52, 181, 55, 164,
     52, 181, 55, 160, 57, 83, 56, 119, 149, 80, 56, 115, 152, 79, 57, 105, 196, 71, 78, 118, 251, 65, 62, 107, 235, 64, 82, 92], "dis_probs": [1, 1, 1, 1, 
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]}, 
    {"point": [102,0], "angles": [45, 135, 225, 315, 50, 140, 230, 320, 55, 145, 235, 325, 60, 150, 240, 330, 65, 155, 245, 335, 70, 160, 250, 340, 75, 
    165, 255, 345, 80, 170, 260, 350, 85, 175, 265, 355, 90, 180, 270, 0, 95, 185, 275, 5, 100, 190, 280, 10, 105, 195, 285, 15, 110, 200, 290, 20, 115,
     205, 295, 25, 120, 210, 300, 30, 125, 215, 305, 35, 130, 220, 310, 40, 135, 225, 315, 45], "distances": [162, 110, 82, 58, 165, 166, 82, 59, 184, 106, 
     83, 59, 163, 136, 84, 60, 106, 140, 67, 72, 55, 137, 66, 80, 53, 271, 67, 93, 52, 270, 73, 90, 52, 265, 58, 122, 51, 283, 56, 114, 51, 283, 55, 151, 
     52, 284, 55, 151, 52, 266, 55, 151, 52, 179, 56, 151, 53, 124, 56, 152, 53, 84, 54, 117, 54, 87, 55, 79, 115, 84, 56, 86, 169, 83, 54, 78], "dis_probs": 
     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]}, 
    {"point": [153,0], "angles": [45, 135, 225, 315, 50, 140, 230, 320, 55, 145, 235, 325, 60, 150, 240, 330, 65, 155, 245, 335, 70, 160, 250, 340, 75,
     165, 255, 345, 80, 170, 260, 350, 85, 175, 265, 355, 90, 180, 270, 0, 95, 185, 275, 5, 100, 190, 280, 10, 105, 195, 285, 15, 110, 200, 290, 20, 
     115, 205, 295, 25, 120, 210, 300, 30, 125, 215, 305, 35, 130, 220, 310, 40, 135, 225, 315, 45], "distances": [151, 56, 80, 122, 160, 229, 93, 119, 
     179, 198, 85, 109, 178, 102, 86, 105, 104, 98, 84, 105, 54, 98, 45, 103, 54, 99, 43, 103, 53, 110, 43, 103, 52, 113, 42, 103, 52, 112, 43, 103, 52, 
     127, 42, 102, 52, 136, 42, 102, 52, 126, 42, 103, 53, 125, 42, 103, 53, 125, 43, 103, 101, 124, 43, 47, 56, 124, 43, 52, 191, 123, 44, 47, 192, 107, 
     60, 47], "dis_probs": [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]}, 
    {"point": [204,0], "angles": [45, 135, 225, 315, 50, 140, 230, 320, 55, 145, 235, 325, 60, 150, 240, 330, 65, 155, 245, 335, 70, 160, 250, 340, 75,
     165, 255, 345, 80, 170, 260, 350, 85, 175, 265, 355, 90, 180, 270, 0, 95, 185, 275, 5, 100, 190, 280, 10, 105, 195, 285, 15, 110, 200, 290, 20, 115,
     205, 295, 25, 120, 210, 300, 30, 125, 215, 305, 35, 130, 220, 310, 40, 135, 225, 315, 45], "distances": [76, 144, 118, 63, 112, 175, 126, 64, 80, 183, 
     90, 65, 70, 143, 45, 57, 58, 195, 44, 56, 54, 159, 42, 55, 54, 193, 42, 55, 53, 168, 41, 55, 53, 162, 41, 55, 53, 161, 41, 54, 53, 167, 41, 54, 53, 170, 
     44, 54, 53, 151, 41, 55, 53, 127, 41, 55, 53, 124, 41, 55, 54, 123, 42, 56, 228, 125, 43, 60, 212, 125, 43, 73, 227, 104, 65, 71], "dis_probs": [1, 1, 1, 
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
     , 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]}])           

    map.show()











