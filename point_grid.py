import numpy as np
import json
import matplotlib.pyplot as plt

class Point_grid:
    def __init__(self,data,resolution=1,ratio=0.75):
        self.resolution = resolution
        self.point = data["point"]
        self.size = np.array([600,600],dtype=np.int16) //self.resolution
        self.grid = np.ones((self.size)) * 0.5
        self.center = np.array([300,300])//self.resolution #0,0 point in the grid
        self.beta = np.deg2rad(5) #cone angle
        self.ratio = ratio
        self.generate(data)
        self.save_json()
        
    def generate(self,data):
        #x,y = data['point']
        
        x1 = np.round((np.array(data['distances']) * np.cos(np.deg2rad(np.array(data['angles'])) + self.beta))//self.resolution).astype(np.int16)
        y1 = np.round((np.array(data['distances']) * np.sin(np.deg2rad(np.array(data['angles'])) + self.beta))//self.resolution).astype(np.int16)
        
        x2 = np.round((np.array(data['distances']) * np.cos(np.deg2rad(np.array(data['angles'])) - self.beta))//self.resolution).astype(np.int16)
        y2 = np.round((np.array(data['distances']) * np.sin(np.deg2rad(np.array(data['angles'])) - self.beta))//self.resolution).astype(np.int16)
        
        obs_lines = []
        empty_space = set()
        for i in range(len(x1)):
            if x1[i] == 0:continue
            
            obs_line = self.bresenham((x1[i],y1[i]),(x2[i],y2[i]))
            obs_lines.append(obs_line)
            for point in obs_line:
                empty_space.update(tuple(self.bresenham((0,0), tuple(point))))
        
        empty_space = np.array(list(empty_space))
        empty_space += self.center
        empty_x,empty_y = np.hsplit(empty_space,2)
        empty_x = empty_x.flatten()
        empty_y = empty_y.flatten()
        self.grid [empty_y ,empty_x] = self.grid [empty_y ,empty_x] * (self.ratio) + 0
        
        for i in range(len(obs_lines)):
            obs_x,obs_y = np.hsplit(np.array(obs_lines[i]),2)
            obs_x = obs_x.flatten() + self.center[0]
            obs_y = obs_y.flatten() + self.center[1]
            
            self.grid [obs_y ,obs_x] = self.grid [obs_y ,obs_x] * (1-self.ratio) + data['dis_probs'][i] * self.ratio
            
        
    def get_grid(self):
        out = self.grid.copy()
        out[self.grid>0.8] = 1
        out[self.grid<0.4] = 0
        out[(out>=0.4) & (out<=0.8)] = 0.5
        return np.flipud(out)
    
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
        out = self.get_grid()
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
            json.dump(self.get_grid().tolist(), outfile)
    
grid = Point_grid({"point": [0, 0], "angles": [45, 135, 225, 315, 50, 140, 230, 320, 55, 145, 235, 
325, 60, 150, 240, 330, 65, 155, 245, 335, 70, 160, 250, 340, 75, 165, 255, 345, 80, 170, 260, 350, 
85, 175, 265, 355, 90, 180, 270, 0, 95, 185, 275, 5, 100, 190, 280, 10, 105, 195, 285, 15, 110, 200, 
290, 20, 115, 205, 295, 25, 120, 210, 300, 30, 125, 215, 305, 35, 130, 220, 310, 40, 135, 225, 315, 45],
 "distances": [72.0, 63.0, 62.0, 72.5, 74.0, 68.5, 59.0, 66.5, 71.0, 68.5, 55.5, 66.5, 63.5, 73.0, 43.5,
 73.0, 59.5, 74.5, 42.0, 59.0, 54.5, 95.0, 42.0, 57.0, 54.0, 92.0, 41.0, 55.5, 53.0, 152.5, 41.0, 55.0, 
 52.5, 86.0, 41.0, 55.0, 52.5, 167.5, 41.0, 55.0, 52.0, 153.0, 40.5, 54.5, 52.0, 140.0, 41.0, 55.0, 52.0,
 107.0, 41.0, 55.5, 52.0, 100.0, 41.0, 56.0, 52.5, 91.5, 41.0, 57.5, 53.5, 65.0, 42.0, 60.5, 54.0, 63.5, 
 42.5, 78.0, 101.0, 63.0, 57.0, 75.5, 144.0, 77.0, 67.5, 74.0], "dis_probs": [1, 1, 1, 1, 1, 1, 1, 1, 1, 
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]},2,0.75)
grid.show()