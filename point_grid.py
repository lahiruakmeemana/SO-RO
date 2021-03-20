import numpy as np
import json
import matplotlib.pyplot as plt
import math
from prob_functions import sigmoidProbability,expoProbability,lineprob

class Point_grid:
    def __init__(self,data,direction,resolution=1,ratio=0.75):
        self.resolution = 4
        self.point = data["point"]
        self.direction = direction -90
        #print(self.direction)
        self.size = np.array([601,601],dtype=np.int16) //self.resolution
        #print(self.size)
        self.grid = np.ones((self.size)) * 0.5
        self.center = np.array([300,300])//self.resolution #0,0 point in the grid
        self.beta = np.deg2rad(5) #cone angle
        self.ratio = ratio
        self.generate(data)
        #self.save_json()
        self.show()
    def generate(self,data):
        #x,y = data['point']
        '''
        90--> +0
        180--> +90
        270--> +180
        0--> -90
        '''
        dis = np.array(data['distances'])
        angles = np.deg2rad(np.array(data['angles'])+self.direction) #check this
        x1 = np.round((dis * np.cos(angles + self.beta))//self.resolution).astype(np.int16)
        y1 = np.round((dis * np.sin(angles + self.beta))//self.resolution).astype(np.int16)
        
        x2 = np.round((dis * np.cos(angles - self.beta))//self.resolution).astype(np.int16)
        y2 = np.round((dis * np.sin(angles - self.beta))//self.resolution).astype(np.int16)
        
        dis_probs = self.dis_probs(np.array(data['distances']))
        
        obs_lines = []
        probs=np.array([])
        empty_space = set()
        for i in range(len(x1)):
            if x1[i] == 0:continue
            
            obs_line = self.bresenham((x1[i],y1[i]),(x2[i],y2[i]))
            obs_lines.append(obs_line)
            distance = dis[i]//self.resolution
            for point in obs_line:
                temp = tuple(self.bresenham((0,0), tuple(point)))
                
                empty_x,empty_y = np.hsplit(np.array(temp),2)
                empty_x = empty_x.flatten() + self.center[0]
                empty_y = empty_y.flatten() + self.center[1]
                
                empty_x = empty_x[:-1]
                empty_y = empty_y[:-1]
                newprobs = lineprob(len(empty_x),distance,dis_probs[i])
                #print(len(empty_x),newprobs.shape)
                self.grid [empty_y ,empty_x] = self.grid [empty_y ,empty_x] * (1-self.ratio) + newprobs*self.ratio
                #probs = np.concatenate((probs,[dis_probs[i]]*len(temp)),0)
                #empty_space.update(temp)
        
        # empty_space = np.array(list(empty_space))
        # empty_space += self.center
        # empty_x,empty_y = np.hsplit(empty_space,2)
        # empty_x = empty_x.flatten()
        # empty_y = empty_y.flatten()
        # print(empty_x.shape,probs.shape)
        # print(self.grid [empty_y ,empty_x] * (1-self.ratio) + (1-probs)*(self.ratio))
        # self.grid [empty_y ,empty_x] = self.grid [empty_y ,empty_x] * (1-self.ratio) + (1-probs)*(self.ratio)
        
        for i in range(len(obs_lines)):
            obs_x,obs_y = np.hsplit(np.array(obs_lines[i]),2)
            obs_x = obs_x.flatten() + self.center[0]
            obs_y = obs_y.flatten() + self.center[1]
            
            self.grid [obs_y ,obs_x] = self.grid [obs_y ,obs_x] * (1-self.ratio) + dis_probs[i] * self.ratio
    
    def dis_probs(self,distances):#implement probability
        temp = np.array(distances.copy(),dtype=np.float64)
        probs = sigmoidProbability(temp)
        return probs
        
    def get_grid(self,flip=True):
        out = self.grid.copy()
        # out[self.grid>0.8] = 1
        # out[self.grid<0.35] = 0
        #out[(out>=0.45) & (out<=0.55)] = 0.5
        if not(flip): return self.point,out
        #print("flipped")
        return self.point,np.flipud(out)
    
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
        
    def show(self):
        _,out = self.get_grid(flip=True)
        res = out.shape
        #fig=plt.figure(figsize=(10,6))
        plt.imshow(out, cmap = "PiYG_r")
        plt.clim(0.0, 1.0)
        plt.gca().set_xticks(np.arange(-.5, res[1], 2), minor = True)
        plt.gca().set_yticks(np.arange(-.5, res[0], 2), minor = True)
        plt.grid(True, which="minor", color="w", linewidth = .6, alpha = 0.5)
        plt.colorbar()
        plt.show()
    
    
    
    def save_json(self):
        with open("point_grid_"+str(self.point[0])+" "+str(self.point[1])+".json", 'w') as outfile:
            _,grid = self.get_grid()
            json.dump(grid.tolist(), outfile)
if __name__ == '__main__':

    grid = Point_grid({"point": [125, 50], "angles": [45, 135, 225, 315, 50, 140, 230, 320, 55, 145, 235, 325, 60, 150, 240, 330, 65, 155, 245, 335, 70, 160, 250, 340, 75, 165, 255, 345, 80, 170, 260, 350, 85, 175, 265, 355, 90, 180, 270, 0, 95, 185, 275, 5, 100, 190, 280, 10, 105, 195, 285, 15, 110, 200, 290, 20, 115, 205, 295, 25, 120, 210, 300, 30, 125, 215, 305, 35, 130, 220, 310, 40, 135, 225, 315, 45], "distances": [269.0, 191.0, 77.0, 268.0, 0.0, 63.0, 76.0, 117.0, 0.0, 46.0, 78.0, 74.0, 249.0, 44.0, 76.0, 55.0, 276.0, 43.0, 80.0, 54.0, 295.0, 42.0, 164.0, 54.0, 242.0, 41.0, 163.0, 54.0, 241.0, 41.0, 157.0, 54.0, 231.0, 41.0, 155.0, 215.0, 268.0, 41.0, 84.0, 215.0, 268.0, 41.0, 199.0, 214.0, 295.0, 41.0, 176.0, 215.0, 290.0, 41.0, 218.0, 216.0, 272.0, 42.0, 211.0, 216.0, 232.0, 42.0, 226.0, 216.0, 269.0, 43.0, 268.0, 216.0, 249.0, 43.0, 299.0, 269.0, 0.0, 54.0, 0.0, 218.0, 244.0, 139.0, 298.0, 217.0], "dis_probs": [0.735, 0.9296, 0.9923, 0.7389, 0.0, 0.9942, 0.9925, 0.983, 0.0, 0.9958, 0.9922, 0.9928, 0.8053, 0.996, 0.9925, 0.995, 0.7068, 0.9961, 0.9918, 0.9951, 0.6225, 0.9962, 0.9577, 0.9951, 0.8264, 0.9962, 0.9585, 0.9951, 0.8292, 0.9962, 0.963, 0.9951, 0.8557, 0.9962, 0.9644, 0.8909, 0.7389, 0.9962, 0.9912, 0.8909, 0.7389, 0.9962, 0.9183, 0.8928, 0.6225, 0.9962, 0.9468, 0.8909, 0.6457, 0.9962, 0.8849, 0.8889, 0.7231, 0.9962, 0.8984, 0.8889, 0.8532, 0.9962, 0.8676, 0.8889, 0.735, 0.9961, 0.7389, 0.8889, 0.8053, 0.9961, 0.6035, 0.735, 0.0, 0.9951, 0.0, 0.8849, 0.8205, 0.9739, 0.6083, 0.887]},90,2,0.75)
    grid.show()