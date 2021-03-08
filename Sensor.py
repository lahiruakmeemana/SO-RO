import numpy as np
import RPi.GPIO as GPIO
import datetime
import time
import json
import numpy as np
from func_timeout import func_timeout, FunctionTimedOut
from threading import Thread
from prob_functions import sigmoidProbability,expoProbability
GPIO.setmode(GPIO.BOARD)
#pip install func-timeout
class Sensor:
    def __init__(self,id,trig,echo,distance_to_sensor,angle):
        self.id = id
        self.trig = trig
        self.echo = echo
        self.distance_to_sensor = distance_to_sensor
        self.angle = angle
        self.distance = None
        GPIO.setup(self.trig,GPIO.OUT)
        GPIO.setup(self.echo,GPIO.IN)
        
    def get(self):
        GPIO.output(self.trig, True)
        time.sleep(0.00001)
        GPIO.output(self.trig, False)
        initial=datetime.datetime.now()
        start = 0.
        end = 0.
        while GPIO.input(self.echo) == False :
            start = datetime.datetime.now()

        while GPIO.input(self.echo) == True and (datetime.datetime.now()-initial).microseconds<17500:
            end = datetime.datetime.now()

        sig_time = end-start

        distance = round(sig_time.microseconds / 58)   
        if distance<20 or distance >300: distance=None
        #print('Distance {}: {} centimeters'.format(self.id,distance))
        self.distance = distance
        return self.distance
    def getdistance(self):
        if self.distance == None: return None
        return self.distance + self.distance_to_sensor
    def run(self):
        try:
            func_timeout(0.02,self.get)
        except FunctionTimedOut:
            pass

class MultiSensor:
    def __init__(self,num_of_sensors,sweep_angle,TRIG,ECHO,servo,step):
        self.cleanup()
        self.num_of_sensors = 4 #num_of_sensors
        self.sweep_angle = 90
        self.sensor_angles = [i*(360//self.num_of_sensors) for i in range(self.num_of_sensors)]
        self.TRIG = TRIG #[11,13,15,19]
        self.ECHO = ECHO #[37,38,35,40]
        self.servo = 5
        self.step = step
        start=0.
        end=0.
        self.distance_to_sensor = 5 #cm
        self.readings = []
        self.sensors=[]
        for i in range(self.num_of_sensors):
            self.sensors.append(Sensor(i,self.TRIG[i],self.ECHO[i],self.distance_to_sensor,self.sensor_angles[i]))
        #self.threads=[Thread(target = sensor.run) for sensor in self.sensors ]            
            
        GPIO.setup(self.servo,GPIO.OUT)
        self.pwm=GPIO.PWM(self.servo,50)
        self.pwm.start(0)
        
    def setAngle(self,angle):
        angle=(angle/18)+2
        self.pwm.ChangeDutyCycle(angle)
        time.sleep(0.5)
        self.pwm.ChangeDutyCycle(0)
        time.sleep(0.1)                     #check for optimal value   
    
    def write_json(self):      
        with open("readings.json", 'w') as outfile:
            json.dump(self.readings, outfile)
        #print("distances saved to "+path)
      
    def turnAndGetDistance(self,location):
        
        forward=[]
        backward=[]
        angles=[]
        point=location    
        for i in range(45,46+self.sweep_angle,self.step):
            self.setAngle(i)
               
            threads=[Thread(target = sensor.run) for sensor in self.sensors ]
            for sensor in threads:
                sensor.start()
            
            for sensor in threads: sensor.join()
            d=[]
            for sensor in self.sensors: d.append(sensor.getdistance())
            
            forward.append(d)
            angles.append([(i+self.sensor_angles[j])%360 for j in range(self.num_of_sensors)])
        #print(forward)
        for i in range(45+self.sweep_angle,44,-self.step):
            self.setAngle(i)
            threads=[Thread(target = sensor.run) for sensor in self.sensors ]
            for sensor in threads:
                sensor.start()
                
            for sensor in threads: sensor.join()
            d=[]
            for sensor in self.sensors: d.append(sensor.getdistance())
            
            backward.insert(0,d)
        #print(backward)
        angles = np.array(angles).flatten()#np.deg2rad(np.array(angles).flatten())
        forward=np.array(forward).flatten()
        backward=np.array(backward).flatten()
        forward[np.where(forward==None)] = backward[np.where(forward==None)]
        backward[np.where(backward==None)] = forward[np.where(backward==None)]
        forward[np.where(forward==None)] = 0
        backward[np.where(backward==None)] = 0
        
        rounded=(forward+backward)//2
        dis_probs = self.dis_prob(rounded , angles)
        distances={'point':point,'angles':angles.tolist(),'distances':rounded.tolist(),'dis_probs':dis_probs}
        self.readings.append(distances)
        return distances
        
    def while_moving(self):
        #self.setAngle(90)
        front = self.sensors[0]
        front.run()
        dis = front.getdistance()
        if dis == None: return 30#no stopping when none. robot detects objects 20 away and stops
        return dis
    
    
    def dis_prob(self,distances):#probability based on distance
        return sigmoidProbability(distances).tolist()
        
    def cleanup(self):
        try:
            self.write_json()
            pwm.stop()
            GPIO.cleanup()
        except:
            pass
            
    