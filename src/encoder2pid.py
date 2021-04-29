import Encoder 
import RPi.GPIO as GPIO
import time
import numpy as np
import matplotlib.pyplot as plt
from simple_pid import PID
from datetime import datetime
import os

def doaread(enc):
    ang=enc.read()*conversion
    ## restrict to domain of -pi to pi, (if necessary)
    ang+=np.pi
    return ang
'''
def someaction(angle,angspeed,cartpos,cartspeed,controller,massofcart,timestep): 
    ## this is for ganden not me lol
    ## right now we probably just want the motor to move sympathetically so
    print("angle boxcar: "+str(angle))
    state=[0,angle,cartspeed,angspeed]
    print("state: "+str(state))
    force_to_apply=controller.control(state)
    print("force: "+str(force_to_apply))
    ## but in general we would need to calculate the required 
    ## LINEAR VELOCITY (which is proportional to the linear force)
    ## and use that to calculate the desired change in linear velocity
    return force_to_apply*timestep/massofcart
 '''   
    
    

if __name__=="__main__":
    pid=PID(1.0,0.0,0.15,setpoint=np.pi) #D=0.275
    now = datetime.now()
    dt_string = now.strftime("%Y-%m-%d_%H.%M.%S")
    path=os.getcwd()
    GPIO.setwarnings(False)
    try:
        end=int(input("How long should trial run?: "))
    except:
        end=10
    start=time.time()
    end=start+end
    
    timeee=[]
    angposition=[]
    angspeed=[]
    cartposition=[]
    cartspeed=[]
    numpilses=[]
    
    if True:
        
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
        boxcarvel=[0]*boxvelnum
        cartvel=[0]*cartvelnum
        cartpos=[0]*cartposnum

        dcartvel=pid(np.average(boxcarpos))
        
        ## so now we have the pos and vel with the revolving indices 
        ## lets get into the meat n potatas
        while time.time()<end:
            if time.time()-last>timestep:
                last=time.time()
                
                posind=(posind+1)%boxposnum
                velind=(velind+1)%boxvelnum
                cartvelind=(cartvelind+1)%cartvelnum
                cartposind=(cartposind+1)%cartposnum
                
                boxcarpos[posind]=doaread(enc)
                boxcarvel[velind]=(boxcarpos[posind]-boxcarpos[posind-1])/timestep
                #cartvel[cartvelind]=cartvel[cartvelind-1]+dcartvel
                

                position=np.average(boxcarpos)
                velocity=np.average(boxcarvel)
                poscart=np.average(cartpos)
                velcart=np.average(cartvel)

                dcartvel=pid(np.average(boxcarpos)) ##as m/sec

                numpulses=(0*velcart+dcartvel)*timestep/(motorinc*shaftradius)
                factor=1
                mag=int(np.abs(numpulses*factor))
                
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
                    cartpos[cartposind]=cartpos[cartposind-1]+(mag*motorinc*shaftradius)
                    cartvel[cartvelind]=(cartpos[cartposind]-cartpos[cartposind-1])/timestep
                else:
                    GPIO.output(dirt,GPIO.LOW)
                    cartpos[cartposind]=cartpos[cartposind-1]-(mag*motorinc*shaftradius)
                    cartvel[cartvelind]=(cartpos[cartposind]-cartpos[cartposind-1])/timestep
                for i in range(mag):
                    time.sleep(shutter)
                    GPIO.output(step,GPIO.HIGH)
                    time.sleep(shutter)
                    GPIO.output(step,GPIO.LOW)
                
                    
        else:
            path=path+"/lab_data/"
            if os.path.isdir(path):
                pass
            else:
                os.mkdir(path)
            path=path+"/"+str(dt_string)
            timeee=np.array(timeee)
            angposition=np.array(angposition)
            angspeed=np.array(angspeed)
            cartposition=np.array(cartposition)
            cartspeed=np.array(cartspeed)
            numpilses=np.array(numpilses)
            np.savetxt(path+"_time.csv", timeee, delimiter=",")
            np.savetxt(path+"_theta.csv", angposition, delimiter=",")
            np.savetxt(path+"_thetadot.csv", angspeed, delimiter=",")
            np.savetxt(path+"_x.csv", cartposition, delimiter=",")
            np.savetxt(path+"_xdot.csv", cartspeed, delimiter=",")
            np.savetxt(path+"_numpulses.csv", numpilses, delimiter=",")
            
            fig,ax=plt.subplots()
            ax.plot(timeee,(angposition-np.pi)*180/np.pi,label=r"$\theta$ of pendulum from eq (in deg)")
            ax.plot(timeee,angspeed*180/np.pi,label=r"$\omega$ of pendulum (in deg/s)")
            ax.plot(timeee,cartposition*100,label=r"$x$ of cart (in cm)")
            ax.plot(timeee,cartspeed*100,label=r"$v$ of cart  (in cm/s)")
            ax.plot(timeee,numpilses,label="num/dir of pulses")
            ax.set_ylabel("no units")
            ax.set_xlabel("time (in seconds)")
            plt.legend()
            plt.savefig(path+"_figure.png")
            plt.show()
    '''
    except:
        GPIO.cleanup()
        raise NameError("Bye Bye!")
'''
