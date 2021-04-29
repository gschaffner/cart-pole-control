import Encoder 
import RPi.GPIO as GPIO
import time
import numpy as np
from lqr import *
from derive import *
from sim import *
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
def doaread(enc):
    ang=enc.read()*conversion
    ## restrict to domain of -pi to pi, (if necessary)
    ang+=np.pi
    return ang

def someaction(angle,angspeed,cartpos,cartspeed,controller,massofcart,timestep): 
    ## this is for ganden not me lol
    ## right now we probably just want the motor to move sympathetically so
    
    state=[0,angle,cartspeed,angspeed]
    print("state: "+str(state))
    force_to_apply=controller.control(state)
    print("force: "+str(force_to_apply))
    ## but in general we would need to calculate the required 
    ## LINEAR VELOCITY (which is proportional to the linear force)
    ## and use that to calculate the desired change in linear velocity
    return force_to_apply*timestep/massofcart
    
    

if __name__=="__main__":
    sim=CartPoleSim(0.125,0.076,0.275,0.008,[0,np.pi,0,1])
    controller=BalanceLQR(sim,np.diag([10,10_000,1,100]),500)
    now = datetime.now()
    dt_string = now.strftime("%Y-%m-%d_%H.%M.%S")
    
    
    timeee=[]
    angposition=[]
    angspeed=[]
    cartposition=[]
    cartspeed=[]
    numpilses=[]
    
    if True:
        start=time.time()
        ## I think these pins can be anything you just need to make sure that they are 
        ## hooked up correctly and specifically, in the right order 
        ## also set the timestep, currently 1/10 of a second
        ## and then conversion needs to be fixed, looked like 1deg=50 or something?
        timestep=0.05 #in seconds
        mass=0.4 #mass in kg
        conversion=2*np.pi/2000
        enc = Encoder.Encoder(24,23)
        ang=enc.read()*conversion #in radians
        last=time.time()
        ## also some stuff for the motor
        ## so it seems like the motor too would be operating with this timestep
        ## so we want to figure out how many pulses to send it in this timestep
        ## in order for it to move with some velocity 
        ## if we have a linear vel v, with shaft radius r, it moves v*t/r radians in time t
        ## if the rotinc is p that is n=vt/rp pulses.pass
        ## whether someaction returns angle or speed is not super relevant
        ## but right now it returns angle
        GPIO.setmode(GPIO.BCM) ## this is to keep the pinnums consistent with the encoder reading
        dirt=22
        step=27
        GPIO.setup(dirt, GPIO.OUT)
        GPIO.setup(step, GPIO.OUT)

        motorinc=np.pi/100 #in radians, i think this is 1.8 deg
        shaftradius=0.014 #in meters, i think this is 2.5mm
        ## uhm so this is kind of bootstrappy but last N entries are stored in 
        ## a list that we access with a revolving index
        ## this is for ease of boxcar-ing
        ## four boxcars, one for theta, theta dot, xdot, x
        boxposnum=2
        boxvelnum=2
        cartvelnum=2
        cartposnum=2
        posind=0
        velind=0
        cartvelind=0
        cartposind=0
        boxcarpos=[ang]*boxposnum
        ## one run to set velocity
        while time.time()-last<timestep:
            pass
        position=doaread(enc)
        boxcarpos[posind]=position
        last=time.time()
        velocity=(boxcarpos[posind]-boxcarpos[posind-1])/timestep
        boxcarvel=[velocity]*boxvelnum
        cartvel=[0]*cartvelnum
        cartpos=[0]*cartposnum

        

        dcartvel=someaction(position,velocity,0,0,controller,mass,timestep)
        
        ## so now we have the pos and vel with the revolving indices 
        ## lets get into the meat n potatas
        while True: #try:
            x = input('<Enter> to exit\nTotal time [s] =')
            if x:
                try:
                    time_total = float(x)
                except ValueError:
                    print('Input format invalid\n')
                while time.time()-start <time_total:
                    if time.time()-last>timestep:
                        last=time.time()
                        posind=(posind+1)%boxposnum
                        velind=(velind+1)%boxvelnum
                        cartvelind=(cartvelind+1)%cartvelnum
                        cartposind=(cartposind+1)%cartposnum
                        boxcarpos[posind]=doaread(enc)
                        boxcarvel=[(boxcarpos[posind]-boxcarpos[posind-1])/timestep]
                        #cartvel[cartvelind]=cartvel[cartvelind-1]+dcartvel
                        cartvel[cartvelind]=dcartvel
                        cartpos[cartposind]=cartpos[cartposind-1]+cartvel[cartvelind-1]*timestep
                        position=np.average(boxcarpos)
                        velocity=np.average(boxcarvel)
                        poscart=np.average(cartpos)
                        velcart=np.average(cartvel)
                        dcartvel=someaction(position,velocity,velcart,poscart,controller,mass,timestep) ##as m/sec
                        numpulses=velcart*timestep/(motorinc*shaftradius)
                        mag=int(np.abs(numpulses))
                        angposition.append(position)
                        angspeed.append(velocity)
                        cartposition.append(poscart)
                        cartspeed.append(velcart)
                        numpilses.append(numpulses)
                        timeee.append(last-start)
                
                        shutter=timestep/(2*mag+1)
                        if shutter<0.0005:
                            shutter=0.0005
                            mag=int(2*timestep/0.0005-1)
                        print("numpulses: "+str(mag))
                        if numpulses>0:
                            GPIO.output(dirt,GPIO.HIGH)
                        else:
                            GPIO.output(dirt,GPIO.LOW)
                        for i in range(mag):
                            time.sleep(shutter)
                            GPIO.output(step,GPIO.HIGH)
                            time.sleep(shutter)
                            GPIO.output(step,GPIO.LOW)
                    
            else: #except:
                break
                print('Goodbye!')
            np.savetxt(str(dt_string)+"_time.csv", np.array(timeee), delimiter=",")
            np.savetxt(str(dt_string)+"_theta.csv", np.array(angposition), delimiter=",")
            np.savetxt(str(dt_string)+"_thetadot.csv", np.array(angspeed), delimiter=",")
            np.savetxt(str(dt_string)+"_x.csv", np.array(cartposition), delimiter=",")
            np.savetxt(str(dt_string)+"_xdot.csv", np.array(cartspeed), delimiter=",")
            np.savetxt(str(dt_string)+"_numpulses.csv", np.array(numpilses), delimiter=",")
            
            fig,ax=plt.subplots()
            ax.plot(timeee,angposition,label=r"$\theta$ (in rad)")
            ax.plot(timeee,angspeed,label=r"$\overset{\cdot}{\theta}$ (in rad/s)")
            ax.plot(timeee,cartposition,label=r"$x$ (in m)")
            ax.plot(timeee,cartspeed,label=r"$\overset{\cdot}{x}$  (in m/s)")
            ax.plot(timeee,numpilses,label="num/dir of pulses")
            ax.set_ylabel("no units")
            ax.set_xlabel("time (in seconds)")
            plt.show()
    '''
    except:
        GPIO.cleanup()
        raise NameError("Bye Bye!")
'''
