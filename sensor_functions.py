
import RPi.GPIO as GPIO
import datetime
import time
import json
import numpy as np
GPIO.setmode(GPIO.BCM)

TRIG = 17
ECHO = 27
servo=3
start=0.
end=0.
GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)
GPIO.setup(servo,GPIO.OUT)
pwm=GPIO.PWM(servo,50)
pwm.start(0)


def write_json(distances):
    
    with open('distances.json', 'w') as outfile:
        json.dump(distances, outfile)
    print("distances saved to distances.json")
    
def load_json(input_file):
    with open(input_file) as f:
        all_points = json.load(f)
    return all_points
    
def getdistance():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    initial=datetime.datetime.now()
    while GPIO.input(ECHO) == False and (datetime.datetime.now()-initial).microseconds<17500:
        start = datetime.datetime.now()

    while GPIO.input(ECHO) == True:
        end = datetime.datetime.now()

    sig_time = end-start

    distance = round(sig_time.microseconds / 58,2)   #(1/(speed/10**4))*2
    #print('Distance: {} centimeters'.format(distance))
    return distance
    
    
def setAngle(angle):
    angle=(angle/18)+2
    pwm.ChangeDutyCycle(angle)
    time.sleep(0.5)
    pwm.ChangeDutyCycle(0)
    time.sleep(0.5)
def getlocation():
    x,y=map(int,input().split())    #for now
    return (x,y)    
    

def turnAndGetDistance(step):
    
    forward=[]
    backward=[]
    angles=[]
    point=getlocation()        #get x,y coordinate
    for i in range(0,181,step):
        duty=(i/18)+2
        pwm.ChangeDutyCycle(duty)
        time.sleep(0.5)
        pwm.ChangeDutyCycle(0)
        time.sleep(0.5)
        d=getdistance()
        forward.append(d)
        angles.append(i)
    for i in range(180,-1,-step):
        duty=(i/18)+2
        pwm.ChangeDutyCycle(duty)
        time.sleep(0.5)
        pwm.ChangeDutyCycle(0)
        time.sleep(0.5)
        d=getdistance()
        backward.insert(0,d)
    
    forward=np.array(forward)
    backward=np.array(backward)
    rounded=np.round((forward+backward)/2,2)
    distances={'point':point,'angles':angles,'distance':rounded}
    return distances   

def convertToXY(data):
    all_xy=[]
    
    for point in data:
        point_x,point_y=point['point']
        angles=np.deg2rad(point['angles'])
        distances=point['distance']
           
        x=distances*np.cos(angles) +point_x
        y=distances*np.sin(angles) +point_y
        xy=np.round(np.dstack((x,y)),2).tolist()
        all_xy.append(xy)
    return np.array(all_xy).reshape(-1,2).tolist()

def cleanup():
    try:
        pwm.stop()
        GPIO.cleanup()
    except:
        pass

        
readings=[]
setAngle(0)
count=5
for i in range(count):
    dis=turnAndGetDistance(45)
    readings.append(dis)
print(readings)
converted=convertToXY(readings)
print(converted)
write_json(converted)
cleanup()