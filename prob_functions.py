import math
import numpy as np
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
    #P = P.astype(np.double)
    return np.round(P,4)

def expoProbability(d):
    x = (d)/300
    P = 0.75**x
    P = P.astype(np.double)
    return np.round(P,4)

def lineprob(arrlen,dis,prob):
    #prob = sigmoidProbability(dis*2)

    step = round(prob/arrlen,6)
    arr = np.arange(prob,0,-step) + 1-prob
    if arr.shape[0] > arrlen: arr = arr[0:arrlen]
    print("in prob",arrlen, arr.shape,step,prob,dis, prob)
    a_ = np.round(1/(dis),4)
    out = np.round(np.e**((200-dis)*(arr-1)*(dis//4)*a_)*prob,4)
    out[out<0.1] = 0.1
    out[out==0.5] -= 0.1
    #print(out.tolist())
    #input()
    return out[::-1]
    
if __name__ == "__main__":    
    
    #print(sigmoidProb(0.9879,20))
    print(sigmoidProbability(20))
    # print(sigmoidProbability(250))
    # print(sigmoidProbability(300))

    # print("---")

    # print(expoProbability(50))
    # print(expoProbability(100))
    # print(expoProbability(150))
    # print(expoProbability(200))
    # print(expoProbability(250))
    # print(expoProbability(300))
