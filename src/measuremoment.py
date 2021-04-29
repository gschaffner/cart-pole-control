import Encoder 
import RPi.GPIO as GPIO
import time
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

def doaread(enc):
    ang=enc.read()*conversion
    ## restrict to domain of -pi to pi, (if necessary)
    ang+=np.pi
    modulus=np.floor(ang/(2*np.pi))
    ang=ang-(modulus+1)*np.pi
    return ang

def sin(t,w,g,q,c):
    return np.exp(-q)*np.sin(w*t+g)+c

if __name__=="__main__":
    timestep=0.1 #in seconds
    conversion=2*np.pi/2000
    enc = Encoder.Encoder(24,23)
    ang=enc.read()*conversion #in radians
    last=time.time()

    timelimit=30
    timer=0
    plot=[]
    timet=[]
    
    boxposnum=2
    posind=0
    boxcarpos=[ang]*boxposnum
    ## one run to set velocity
    while time.time()-last<timestep:
        pass
    boxcarpos[posind]=doaread(enc)
    last=time.time()
    posind=(posind+1)%boxposnum
    ## so now we have the pos and vel with the revolving indices 
    ## lets get into the meat n potatas
    while timer<timelimit:
        if time.time()-last>timestep:
            last=time.time()
            timer+=timestep
            boxcarpos[posind]=doaread(enc)
            posind=(posind+1)%boxposnum

            position=np.average(boxcarpos)

            print(str(timer)+" seconds: "+str(position))
            plot.append(position)
            timet.append(timer)

    popt,pcov=curve_fit(sin, timet, plot)
    perr=np.sqrt(np.diag(pcov))
    w=popt[0]

    fig,ax=plt.subplots()
    ax.plot(timet,plot,linewidth=0,marker="o")
    ax.plot(timet,sin(timet,*w), linewidth=2)
    plt.show()

    ## enter in these values after measuring
    M=76 #mass of pendulum in grams
    d=58 #distance of center of mass from pivot in cm
    g=9.81 #gravity
    I= M*g*d*0.001*0.01/w**2
    ## If we take Mgd to be definitively defined then dI=2Mgd dw/w^3
    dI=2*M*g*d*perr[0]/w**3

    print("The moment of inertia is: "+str(I))
    print("The error of moment of inertia is: "+str(dI))
    
