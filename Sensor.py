import numpy as np
import RPi.GPIO as GPIO
import datetime
import time
import json
import numpy as np
from threading import Thread

class Sensor:
    def __init__(self,trig,echo,distance_to_sensor,angle):
        self.trig = trig
        self.echo = echo
        self.distance_to_sensor = distance_to_sensor
        self.angle = angle
        GPIO.setup(trig,GPIO.OUT)
        GPIO.setup(echo,GPIO.IN)
        
    def getdistance(self):
        GPIO.output(trig, True)
        time.sleep(0.00001)
        GPIO.output(trig, False)
        initial=datetime.datetime.now()
        start = 0.
        end = 0.
        while GPIO.input(echo) == False :
            start = datetime.datetime.now()

        while GPIO.input(echo) == True and (datetime.datetime.now()-initial).microseconds<17500:
            end = datetime.datetime.now()

        sig_time = end-start

        distance = round(sig_time.microseconds / 58,2)   
        if distance<20 or distance >400: distance=None
        #print('Distance: {} centimeters'.format(distance))
        return distance
        

class MultiSensor:
    def __init__(self,num_of_sensors,sweep_angle,TRIG,ECHO,servo,step):
        self.num_of_sensors = num_of_sensors
        self.sweep_angle = 90
        self.sensor_angles = [i*(360//self.num_of_sensors) for i in range(self.num_of_sensors)]
        self.TRIG = [11]
        self.ECHO = [13]
        self.servo = 5
        self.step = 15
        start=0.
        end=0.
        distance_to_sensor = 5 #cm
        sensors=[]
        for i in range(self.num_of_sensors):
            sensors.append(Sensor(self.TRIG[i],self.ECHO[i],self.distance_to_sensor,self.sensor_angles[i]))
                    
            
        GPIO.setup(servo,GPIO.OUT)
        pwm=GPIO.PWM(servo,50)
        pwm.start(0)
        
     def setAngle(self,angle):
        angle=(angle/18)+2
        pwm.ChangeDutyCycle(angle)
        time.sleep(0.5)
        pwm.ChangeDutyCycle(0)
        time.sleep(0.5)   
    
    def write_json(path,distances):      
        with open(path, 'w') as outfile:
            json.dump(distances, outfile)
        #print("distances saved to "+path)
      
    def turnAndGetDistance(self,step,location):
        
        forward=[]
        backward=[]
        angles=[]
        point=location    
        for i in range(45,45+self.sweep_angle,step):
            self.setAngle(i)
            threads=[]
            for sensor in sensors:
                Thread(target=sensor).start()
            d=[]
            for sensor in sensors: d.append(Sensor.getdistance())
            for sensor in sensors: sensor.join()
            
            forward.append([d])
            angles.append([i+self.sensor_angles[i] for j in range(self.num_of_sensors)])
        for i in range(45+self.sweep_angle,44,-step):
            setAngle(i)
            
            for sensor in sensors:
                Thread(target=sensor).start()
            d=[]
            for sensor in sensors: d.append(Sensor.getdistance())
            for sensor in sensors: sensor.join()

            backward.insert(0,[d])
        angles = np.deg2rad(angles.flatten())
        forward=np.array(forward).flatten()
        backward=np.array(backward).flatten()
        rounded=np.round((forward+backward)/2,2)
        dis_probs = distance_prob(rounded , angles)
        distances={'point':point,'angles':angles.tolist(),'distances':rounded.tolist(),'dis_probs':dis_probs}
        write_json("distances.json",distances)
        return distances
        
    def dis_probs(self,rounded,angles):#implement probability
        return [1]*len(rounded)
        
    def cleanup(self):
        try:
            pwm.stop()
            GPIO.cleanup()
        except:
            pass
            
    