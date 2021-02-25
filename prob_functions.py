import math

def probability_2D(d_cm, theta, z):
    d=d_cm/10
    delta_d=d*0.02
    delta_theta=math.pi/6

    expart=(-(d-z)**2)/(2*delta_d**2) + (theta**2)/(delta_theta**2)
    P = (1/2*math.pi*delta_d*delta_theta)*math.e**expart
    return P

def probability_1D(d_cm,z):
    d=d_cm/10
    delta_d=d*0.02

    P = (1/math.sqrt(2*math.pi*delta_d))*math.e**((-(d-z)**2)/(2*delta_d**2))
    return P

def sigmoidProbability(d):
    x = ((320-d)/300)*6
    P = 1/(1 + math.e**(-x))
    return round(P,4)

def expoProbability(d):
    x = (d)/300
    P = 0.75**x
    return round(P,4)

print(sigmoidProbability(50))
print(sigmoidProbability(100))
print(sigmoidProbability(150))
print(sigmoidProbability(200))
print(sigmoidProbability(250))
print(sigmoidProbability(300))

print("---")

print(expoProbability(50))
print(expoProbability(100))
print(expoProbability(150))
print(expoProbability(200))
print(expoProbability(250))
print(expoProbability(300))
