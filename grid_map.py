import numpy as np
import json
import matplotlib.pyplot as plt

class Grid:
    def __init__(self,resolution,ratio):
        
        infinity = np.inf
        self.min_x, self.max_x, self.min_y, self.max_y= 0,0,0,0
        self.resolution=resolution
        #self.size=(int(round((self.max_x - self.min_x)/self.resolution)),int(round((self.max_y - self.min_y)/self.resolution)))
        self.map=np.ones((1,1),dtype=np.float)*0.5
        #sefl.center= (-self.min_x, -self.min_y)
        self.beta = np.deg2rad(10) #cone angle
        self.ratio = ratio
        self.center= (0,0)
        #probabilities
        print('map initialized')
    def get_grid(self):
        return np.flipud(self.map)
    
    
        
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
        print("grid size changed to: ",self.map.shape)
        
     
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
    
    def update(self,xy_points,distances,angles,dis_prob):#input [xy_points,distances,angles,dis_prob]
        
        assert len(xy_points) == len(distances) == len(dis_prob)
        x, y = np.hsplit(xy_points,2)
        x=x.flatten().astype(np.int) #+ self.center[0]
        y=y.flatten().astype(np.int) #+ self.center[1]
        #print(x,y)
        x1 = np.round((x + distances * np.cos(np.deg2rad(angles) + self.beta))/self.resolution).astype(np.int)
        y1 = np.round((y + distances * np.sin(np.deg2rad(angles) + self.beta))/self.resolution).astype(np.int)

        x2 = np.round((x + distances * np.cos(np.deg2rad(angles) - self.beta))/self.resolution).astype(np.int)
        y2 = np.round((y + distances * np.sin(np.deg2rad(angles) - self.beta))/self.resolution).astype(np.int)
        #print(np.round((x + distances * np.cos(np.deg2rad(angles))/self.resolution).astype(np.int)))
        #obstacle_line = self.bresenham((x1,y1),(x2,y2)) #these inputs are np arrays won't work
        obs_lines= []
        empty_space= set()
        probs=np.array([])
        min_x,min_y,max_x,max_y = np.inf, np.inf, -np.inf, -np.inf
        obs_line_x=np.array([],dtype=np.int)
        obs_line_y=np.array([],dtype=np.int)
        for i in range(len(x)):
            #for j in range(distances.shape[0]):
            
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
        
    
   
    def input_rawdata(self,all_points):
        xy_points = []
        distances = []
        angles = []
        dis_prob = []
        for rec in all_points:
            xy_points.append(rec['point']*len(all_points[0]['angles']))
            distances.append(rec['distances'])
            angles.append(rec['angles'])
            dis_prob.append(rec['dis_probs'])

        xy_points = np.array(xy_points).reshape(-1,2) *51   #remove this 51
        distances = np.array(distances).flatten()
        angles = np.array(angles).flatten()
        dis_prob = np.array(dis_prob).flatten()
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
    
           













