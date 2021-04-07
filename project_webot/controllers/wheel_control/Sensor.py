import numpy as np
import datetime
import time
import json
import numpy as np
from func_timeout import func_timeout, FunctionTimedOut
from threading import Thread
from prob_functions import sigmoidProbability,expoProbability

#pip install func-timeout

class Sensor:
    def __init__(self,id,us):
        
        self.id = id 
        self.distance = None
        self.us = us
        self.us.enable(8)
        
    def get(self):
        try:
            dis = round(self.us.getValue()*100)
            self.distance = dis
        except:
            pass
            
        return 
    def getdistance(self):
        if self.distance > 300: return 300
        return self.distance
        
    

class MultiSensor:
    def __init__(self,robot,sweep_angle,step):
        self.robot = robot
        self.num_of_sensors = 4 #num_of_sensors
        self.sweep_angle = sweep_angle
        self.sensor_angles = [i*(360//self.num_of_sensors) for i in range(self.num_of_sensors)]
        
        self.servo_encoder = robot.getDevice('servo_encoder')
        self.servo = robot.getDevice('servo')
        self.servo.setVelocity(0)
        self.servo.setPosition(float('inf'))
        self.servo_encoder.enable(2)
        self.step = step
        self.mul = 1
        self.readings = []
        self.sensors=[]
        for i in range(self.num_of_sensors):
            self.sensors.append(Sensor(i,self.robot.getDevice('us'+str(i))))
        #self.threads=[Thread(target = sensor.run) for sensor in self.sensors ]            
            
        
        
    def setAngle(self,angle):
        
        try:    
            prev= self.servo_encoder.getValue()
            round(prev)
        except:
            prev = 0
        
        sign = 1 if angle>0 else -1
        self.servo.setVelocity(6.28*0.05*sign)
        while self.robot.step(1):
           
            if angle>0 and self.servo_encoder.getValue()>=prev+np.deg2rad(angle):
                break
            if angle<0 and self.servo_encoder.getValue()<=prev+np.deg2rad(angle):
                break
        
        self.servo.setVelocity(0)
        
    def write_json(self):      
        with open("readings.json", 'w') as outfile:
            json.dump(self.readings, outfile)
        #print("distances saved to "+path)
      
    def turnAndGetDistance(self,location,direction):
        
        forward=[]
        backward=[]
        angles=[]
        point=location 
        start = 45
        end = 45+self.sweep_angle        
        if self.mul == 1:
            self.mul=-1
        elif self.mul == -1:
            start = end
            end = 45
            self.mul=1
        for i in range(start,end,self.step*self.mul*-1):
            self.setAngle(self.step*self.mul*-1)
            
            threads=[Thread(target = sensor.get) for sensor in self.sensors ]
            for sensor in threads:
                sensor.start()
            
            for sensor in threads: sensor.join()
            d=[]
            for sensor in self.sensors: d.append(sensor.getdistance())
            
            forward.append(d)
            angles.append([(i+self.sensor_angles[j])%360 for j in range(self.num_of_sensors)])
        
            
        angles = np.array(angles).flatten()#np.deg2rad(np.array(angles).flatten())
        forward=np.array(forward).flatten().astype(np.double)
        # backward=np.array(backward).flatten().astype(np.double)
        
        # forward[np.where(forward==0)] = backward[np.where(forward==0)]
        # backward[np.where(backward==0)] = forward[np.where(backward==0)]
        # #forward[np.where(forward==None)] = 0
        # #backward[np.where(backward==None)] = 0
        
        # rounded=np.round(forward*0.25+backward*0.75)
        rounded = np.round(forward)
        #print(rounded)
        #rounded[np.where(rounded==)]
        dis_probs = self.dis_prob(rounded)
        
        distances={'point':point,'direction': direction,'angles':angles.tolist(),'distances':rounded.tolist(),'dis_probs':dis_probs}
        self.readings.append(distances)
        return distances
        
    def while_moving(self):
        #self.setAngle(90)
        front = self.sensors[0]
        front.run()
        dis = front.getdistance()
        if dis == 3.15: return 3#no stopping when none. robot detects objects 20 away and stops
        return dis
    
    
    def dis_prob(self,distances):#probability based on distance
        return sigmoidProbability(distances).tolist()
        
    
    