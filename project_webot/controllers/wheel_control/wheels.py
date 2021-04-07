from controller import Motor

class Wheels:
    def __init__(self,robot):
        self.robot = robot
        self.tire = 0.2041#3.25*2*3.14
        self.left_encoder = robot.getDevice('left_encoder')
        self.right_encoder = robot.getDevice('right_encoder')
        self.left_motor = robot.getDevice('left_motor')
        self.right_motor = robot.getDevice('right_motor')
        
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        self.speed = 6.28
        self.first = True
        self.left_encoder.enable(8)
        self.right_encoder.enable(8)

    def setSpeed(self,l_speed,r_speed):
        self.left_motor.setVelocity(l_speed)
        self.right_motor.setVelocity(r_speed)
    
    def goForward(self,dis):
        prev_left = self.left_encoder.getValue()
        prev_right = self.right_encoder.getValue()
        if self.first: 
            prev_left,prev_right = 0,0
            self.first = False
        left_limit = prev_left + 2*3.14*(dis/self.tire)
        right_limit = prev_right + 2*3.14*(dis/self.tire)
        
        #sensor.setAngle(90)
        self.setSpeed(self.speed*0.75,self.speed*0.75)
        while self.robot.step(10)!=-1:
            if self.left_encoder.getValue()>=left_limit:
                self.stopLeft()
            if self.right_encoder.getValue()>=right_limit:
                self.stopRight()
            if (self.left_encoder.getValue() >= left_limit) and (self.right_encoder.getValue() >= right_limit) : break
            
            
            # if sensor.while_moving()<30:
                # self.stopLeft()
                # self.stopRight()
                # return False
        
        self.stopLeft()        
        self.stopRight()
        return True
    
    def turning(self,left,right):
        prev_left = self.left_encoder.getValue()
        prev_right = self.right_encoder.getValue()
        
        if self.first: 
            prev_left,prev_right = 0,0
            self.first = False
        left_limit = prev_left + 2*3.14*(left/self.tire)
        right_limit = prev_right + 2*3.14*(right/self.tire)
        if left < right:
            while self.robot.step(10)!= -1:
                if self.left_encoder.getValue()<=left_limit:
                    self.stopLeft()
                if self.right_encoder.getValue()>=right_limit:
                    self.stopRight()
                if (self.left_encoder.getValue() <= left_limit) and (self.right_encoder.getValue() >= right_limit) : break
                
        else:
            while self.robot.step(10)!= -1:
                if self.left_encoder.getValue()>=left_limit:
                    self.stopLeft()
                if self.right_encoder.getValue()<=right_limit:
                    self.stopRight()
                if (self.left_encoder.getValue() >= left_limit) and (self.right_encoder.getValue() <= right_limit) : break

        self.stopLeft()        
        self.stopRight()
        return True

    def stopLeft(self):
        self.left_motor.setVelocity(0.0)
        
    def stopRight(self):
        self.right_motor.setVelocity(0.0)
        
    
    def turn90(self):
        self.setSpeed(-self.speed*0.25,self.speed*0.25)
        self.turning(-0.1256687, 0.1256687)

    def turn270(self):
        self.setSpeed(self.speed*0.25,-self.speed*0.25)
        self.turning(0.1256687, -0.1256687)
    
    def turn180(self):
        self.setSpeed(-self.speed*0.25,self.speed*0.25)
        self.turning(-0.2513374, 0.2513374)

    def turn(self,angle):
        if angle == 0: pass
        elif angle == 90: self.turn90()
        elif angle == 180: self.turn180()
        elif angle == 270: self.turn270()
        else: return 0
        return 1
