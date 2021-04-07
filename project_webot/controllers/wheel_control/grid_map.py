import numpy as np
import json
import matplotlib.pyplot as plt
import math
import cv2
from prob_functions import sigmoidProbability,expoProbability,lineprob

class Grid:
    def __init__(self,resolution,ratio):
        
        self.min_x, self.max_x, self.min_y, self.max_y= 0,0,0,0
        self.resolution=resolution
        #self.size=(int(round((self.max_x - self.min_x)/self.resolution)),int(round((self.max_y - self.min_y)/self.resolution)))
        self.map=np.ones((1,1),dtype=np.float64)*0.5
        #sefl.center= (-self.min_x, -self.min_y)
        self.beta = np.deg2rad(2.5) #cone angle
        self.ratio = ratio
        self.center= (0,0)
        #probabilities
        print('map initialized')
    def get_grid(self,flip=True):
        out = self.map.copy()
        #out[self.map>0.65] = 1
        # out[self.map<0.4] = 0
        #out[(out>=0.45) & (out<=0.6)] = 0.5
        if flip==False: return out
        #print("flipped global")
        return np.flipud(out)
    
    def load_saved(self,string):
        with open(string) as f:
            save = json.load(f)
        self.min_x,self.max_x,self.min_y,self.max_y = save["min_x"],save["max_x"],save["min_y"],save["max_y"]
        self.resolution = save["resolution"]
        self.center = save["center"]
        self.ratio = save["ratio"]
        self.map = np.array(save["map"])
        
        
        
    def grid_size_update(self,min_x,max_x,min_y,max_y): 
        
        #min_x,max_x,min_y,max_y = min_x.item(),max_x.item(),min_y.item(),max_y.item()
        
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
        print("Center: ",self.center)

    def bresenham(self,start, end):
        '''
        Implementation of Bresenham's line drawing algorithm
        See en.wikipedia.org/wiki/Bresenham's_line_algorithm
        '''
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
        dy = y2 - y1  
        error = int(dx / 2.0)  # calculate error
        y_step = 1 if y1 < y2 else -1
        # iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)  
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += y_step
                error += dx
        if swapped:  # reverse the list if the coordinates were swapped
            points.reverse()
        #points = np.array(points)
        return points
    
    def update_pointgrid(self,point,grid,show=False):#both maps upside down
        point = np.array(point)*100
        res = int(150*(4/self.resolution))
        grid = cv2.resize(grid, (res,res), interpolation=cv2.INTER_NEAREST)
        #res = 4/self.resolution
        min_x,min_y = int(round(point[0]/self.resolution - grid.shape[0]/2)) , int(round(point[1]/self.resolution - grid.shape[1]/2))
        max_x,max_y = int(round(point[0]/self.resolution + grid.shape[0]/2)) , int(round(point[1]/self.resolution + grid.shape[1]/2))
        
        
        self.grid_size_update(min_x,max_x,min_y,max_y)
        #change grid resolution
        
        indices = np.where(grid!=[0.5])
        indexx,indexy = indices[1],indices[0]

        indexx=indexx.flatten().astype(np.int) + min_x + self.center[0]
        indexy=indexy.flatten().astype(np.int) +min_y + self.center[1]
        
        
        self.map[indexy,indexx] = self.map[indexy,indexx] * (1-self.ratio) + grid[indices[0],indices[1]] * self.ratio
        #self.map[0,10] = 1
        # self.map[min_y+self.center[1]:max_y+1+self.center[1], min_x+self.center[0]:max_x+1+self.center[1]] = \
        # self.map[min_y+self.center[1]:max_y+1+self.center[1], min_x+self.center[0]:max_x+1+self.center[1]] *(1-self.ratio) +\
        # grid * self.ratio
        if show: self.show()
        print("Map updated")
        
        
    
    def update(self,xy_points,distances,angles,dis_prob):#input [xy_points,distances,angles,dis_prob]
        
        assert len(xy_points) == len(distances) == len(dis_prob)
        x, y = np.hsplit(xy_points,2)
        x=x.flatten().astype(np.int32) #+ self.center[0]
        y=y.flatten().astype(np.int32) #+ self.center[1]
        #print(x,y)
        x1 = ((x + distances * np.cos(angles + self.beta))//self.resolution).astype(np.int32)
        y1 = ((y + distances * np.sin(angles + self.beta))//self.resolution).astype(np.int32)

        x2 = ((x + distances * np.cos(angles - self.beta))//self.resolution).astype(np.int32)
        y2 = ((y + distances * np.sin(angles - self.beta))//self.resolution).astype(np.int32)
        #print(np.round((x + distances * np.cos(np.deg2rad(angles))/self.resolution).astype(np.int)))
        
        obs_lines= []
        empty_space= set()
        probs=np.array([])
        min_x,min_y,max_x,max_y = np.inf, np.inf, -np.inf, -np.inf
        obs_line_x=np.array([],dtype=np.int32)
        obs_line_y=np.array([],dtype=np.int32)
        for i in range(len(x)):
            #for j in range(distances.shape[0]):
            if x1[i]==0:continue
            
            obs_line = self.bresenham((x1[i],y1[i]),(x2[i],y2[i]))
            obs_lines.append(obs_line)
            
            obs_line = np.array(obs_line)
            temp_x,temp_y = np.hsplit(np.array(obs_line),2)
            
            obs_line_x = np.concatenate((obs_line_x, temp_x.flatten()),0)
            obs_line_y = np.concatenate((obs_line_y, temp_y.flatten()),0)
            
            probs = np.concatenate((probs,[dis_prob[i]]*obs_line.shape[0]),0)#lineprob(obs_line.shape[0],dis,prob)
        
            temp_min_x, temp_min_y = np.min(obs_line,0)
            temp_max_x, temp_max_y = np.max(obs_line,0)
            
            min_x = min(min_x, temp_min_x)
            min_y = min(min_y, temp_min_y)
            max_x = max(max_x, temp_max_x)
            max_y = max(max_y, temp_max_y)
            
        if (min_x,max_x,min_y,max_y) != (self.min_x, self.max_x, self.min_y, self.max_y):
            self.grid_size_update(min_x.item(),max_x.item(),min_y.item(),max_y.item())
        
        for i,obs_line in enumerate(obs_lines):
            for point in obs_line:
                temp = tuple(self.bresenham((x[i]//self.resolution,y[i]//self.resolution), tuple(point)))
                
                empty_x,empty_y = np.hsplit(np.array(temp),2)
                empty_x = empty_x.flatten() + self.center[0]
                empty_y = empty_y.flatten() + self.center[1]
                
                empty_x = empty_x[:-1]
                empty_y = empty_y[:-1]
                newprobs = lineprob(len(empty_x),distances[i]/2,dis_prob[i])
                
                #empty_space.update(tuple(self.bresenham()))
                self.map [empty_y ,empty_x] = self.map [empty_y ,empty_x] * (1-self.ratio) + newprobs*self.ratio
        
        
        # empty_space=np.array(list(empty_space))
        
        # empty_space_x ,empty_space_y = np.hsplit(empty_space,2)
        # empty_space_x = empty_space_x.flatten() + self.center[0]
        # empty_space_y = empty_space_y.flatten() + self.center[1]
        # self.map [empty_space_y ,empty_space_x] = self.map [empty_space_y ,empty_space_x] * (1-self.ratio) + 0 #0 prob for empty space
        
        
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
            angles.append(np.array(rec['angles']) + rec['direction'] - 90)
            dis_prob.append( self.dis_probs(rec['distances'])) #rec['dis_probs'])
        
        xy_points = np.array(xy_points).reshape(-1,2)
        distances = np.array(distances).flatten()
        angles = np.deg2rad(np.array(angles).flatten())
        dis_prob = np.array(dis_prob).flatten()
        #print(dis_prob)
              
        temp = np.dstack((distances,angles,dis_prob))
        #temp.reshape((-1,3))
        np.sort(temp,axis=1)
        temp = np.dsplit(temp,3)
        distances = temp[0].flatten()
        angles = temp[1].flatten()
        dis_prob = temp[2].flatten()
        
        return self.update(xy_points,distances,angles,dis_prob)
     
    def from_json(self):
        with open("readings.json") as f:
            readings = json.load(f)
        self.input_rawdata(readings)
        self.show()
    
    def show(self):
        # out = self.get_grid(flip=True)
        # res = out.shape
        # fig = plt.figure()
        
        # #out[out>0.7] = 1
        # #out[(out<0.6) & (out!=0.5)] -= 0.4
        # #out[out==0.5] = 0
        # #ax = fig.add_subplot(1, 1, 1)
        
        # # ax.spines['top'].set_visible(False)
        # # ax.spines['right'].set_visible(False)
        # plt.imshow(out, cmap = "PiYG_r")
        # plt.clim(0, 1.0)
        
        
        # plt.gca().set_xticks(np.arange(-.5, res[1], 2), minor = True)
        # plt.gca().set_yticks(np.arange(-.5, res[0], 2), minor = True)
        # plt.grid(True, which="minor", color="w", linewidth = .6, alpha = 0.5)
        # plt.axis("equal")
        # plt.colorbar()
        # plt.show()
        cv2.imshow('grid',self.get_grid(flip=True))
        cv2.waitKey()

if __name__ == '__main__':
    grid = Grid(4,0.75)
    grid.from_json()
    grid.show()