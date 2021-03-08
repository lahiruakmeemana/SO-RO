import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

#set GPIO Pins
class Wheels:
    def __init__(self):
        self.tire = 20.7345#measure tire size diameter 66mm
        self.holes = 19 #encoder holes count
        self.dis_per_hole = self.tire / self.holes
        self.Motor1A = 18 #16
        self.Motor1B = 16 #18
        self.Motor1E = 22 #22

        self.Motor2A = 21 #23
        self.Motor2B = 23 #21
        self.Motor2E = 29 #27
        
        self.clkA = 24 #29
        self.dtA = 11 #31
        
        self.clkB = 36 #32
        self.dtB = 12 #36
        

        ##-------From Online--------
        #https://github.com/modmypi/Rotary-Encoder/blob/master/rotary_encoder.py
        GPIO.setup(self.clkA, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.dtA, GPIO.IN, pull_up_down=GPIO.PUD_UP) #was pud down

        GPIO.setup(self.clkB, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.dtB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        ##-------End From Online--------

        #set GPIO direction (IN / OUT)


        GPIO.setup(self.Motor1A,GPIO.OUT)
        GPIO.setup(self.Motor1B,GPIO.OUT)
        GPIO.setup(self.Motor1E,GPIO.OUT)

        GPIO.setup(self.Motor2A,GPIO.OUT)
        GPIO.setup(self.Motor2B,GPIO.OUT)
        GPIO.setup(self.Motor2E,GPIO.OUT)

        #set motor speeds
        GPIO.output(self.Motor1E,GPIO.LOW)
        GPIO.output(self.Motor2E,GPIO.LOW)
            
        self.pwm1 = GPIO.PWM(self.Motor1A, 1000) #frequency @ 1000
        self.pwm2 = GPIO.PWM(self.Motor1B, 1000) #frequency @ 1000

        self.pwm3 = GPIO.PWM(self.Motor2A, 1000) #frequency @ 1000
        self.pwm4 = GPIO.PWM(self.Motor2B, 1000) #frequency @ 1000
    
    def setSpeed(self,speed):
        self.pwm1.ChangeFrequency(speed) #frequency @ 1000
        self.pwm2.ChangeFrequency(speed) #frequency @ 1000

        self.pwm3.ChangeFrequency(speed) #frequency @ 1000
        self.pwm4.ChangeFrequency(speed) #frequency @ 1000

    

    #Below functions control basic
    def distance(self,left,right,sensor):
        encoder_count_left = 0
        encoder_count_right = 0
        lastleft = GPIO.input(self.dtA)
        lastright = GPIO.input(self.dtB)
        #print(lastleft,lastright)
        dis_left = round(left/ self.dis_per_hole)
        dis_right = round(right/ self.dis_per_hole)
        print("dis",dis_left,dis_right)
        sensor.setAngle(90)
        self.setSpeed(300)
        while True:
            if encoder_count_left>=dis_left:self.stopA()
            if encoder_count_right>=dis_right:self.stopB()
            if (encoder_count_left >= dis_left) and (encoder_count_right >= dis_right) : break
            
            currleft = GPIO.input(self.dtA)
            currright = GPIO.input(self.dtB)
            print("left",encoder_count_left,dis_left)
            print("right",encoder_count_right,dis_right)
            
            if sensor.while_moving()<30:
                self.stopA()
                self.stopB()
                return False
            
            if GPIO.input(self.dtA) == 0 and lastleft == 1:
                encoder_count_left += 1
                
            if GPIO.input(self.dtB) == 0 and lastright == 1:
                encoder_count_right += 1
            lastleft = currleft
            lastright = currright
            
            
        
        # self.stopA()
        # self.stopB()
        return True
    
    def turning(self,left,right):
        encoder_count_left = 0
        encoder_count_right = 0
        lastleft = GPIO.input(self.dtA)
        lastright = GPIO.input(self.dtB)
        #print(lastleft,lastright)
        dis_left = round(left/ self.dis_per_hole)
        dis_right = round(right/ self.dis_per_hole)
        #print(dis_left,dis_right)
        while True:
            if encoder_count_left>=dis_left:self.stopA()
            if encoder_count_right>=dis_right:self.stopB()
            if (encoder_count_left >= dis_left) and (encoder_count_right >= dis_right) : break
            
            currleft = GPIO.input(self.dtA)
            currright = GPIO.input(self.dtB)
            print("left",encoder_count_left)
            print("right",encoder_count_right)
            
            
            if GPIO.input(self.dtA) == 0 and lastleft == 1:
                encoder_count_left += 1
                
            if GPIO.input(self.dtB) == 0 and lastright == 1:
                encoder_count_right += 1
            lastleft = currleft
            lastright = currright
            if encoder_count_left>dis_left:self.stopA()
            if encoder_count_right>dis_right:self.stopB()
            
        
        self.stopA()
        self.stopB()
        return True

    def stopA(self):
        GPIO.output(self.Motor1E,GPIO.LOW)
        GPIO.output(self.Motor1A,GPIO.LOW)
        GPIO.output(self.Motor1B,GPIO.LOW)
        
    def stopB(self):
        GPIO.output(self.Motor2E,GPIO.LOW)
        GPIO.output(self.Motor2A,GPIO.LOW)
        GPIO.output(self.Motor2B,GPIO.LOW)
    def goForward(self,cm,sensor):
        
        GPIO.output(self.Motor1E,GPIO.HIGH)
        GPIO.output(self.Motor2E,GPIO.HIGH)
        self.pwm1.start(20)
        self.pwm2.start(0)
        self.pwm3.start(0)
        self.pwm4.start(20)
        return self.distance(cm,cm,sensor)
        

    # def goBackward(self,cm):
        # GPIO.output(self.Motor1E,GPIO.HIGH)
        # GPIO.output(self.Motor2E,GPIO.HIGH)
        # self.pwm1.start(0)
        # self.pwm2.start(20)
        # self.pwm3.start(20)
        # self.pwm4.start(0)
        # self.turning(cm,cm)

    def turn90(self):
        GPIO.output(self.Motor1E,GPIO.HIGH)
        GPIO.output(self.Motor2E,GPIO.HIGH)
        self.pwm1.start(0)
        self.pwm2.start(20)
        self.pwm3.start(0)
        self.pwm4.start(20)
        self.turning(23, 23)

    def turn270(self):
        GPIO.output(self.Motor1E,GPIO.HIGH)
        GPIO.output(self.Motor2E,GPIO.HIGH)
        self.pwm1.start(20)
        self.pwm2.start(0)
        self.pwm3.start(20)
        self.pwm4.start(0)
        self.turning(23,23)
    
    def turn180(self):
        GPIO.output(self.Motor1E,GPIO.HIGH)
        GPIO.output(self.Motor2E,GPIO.HIGH)
        self.pwm1.start(20)
        self.pwm2.start(0)
        self.pwm3.start(20)
        self.pwm4.start(0)
        self.turning(46,46)

    def turn(self,angle):
        if angle == 0: pass
        elif angle == 90: self.turn90()
        elif angle == 180: self.turn180()
        elif angle == 270: self.turn270()
        else: return 0
        return 1
