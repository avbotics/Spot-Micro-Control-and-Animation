#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@author: Arnaud Villeneuve

This file contains the main program to run the Spotmicro Controller

"""
 

from time import sleep, time
from math import pi, sin, cos, atan, atan2, sqrt
import numpy as np

import pygame
pygame.init()
screen = pygame.display.set_mode((600, 600)) 
pygame.display.set_caption("SPOTMICRO")


import Spotmicro_Inverse_Kinematics_and_Position_Library_v01
Spot = Spotmicro_Inverse_Kinematics_and_Position_Library_v01.Spot()
import Spotmicro_Gravity_Center_Library_v01
SpotCG = Spotmicro_Gravity_Center_Library_v01.SpotCG()
import Spotmicro_Animation_Library_v01
SpotAnim = Spotmicro_Animation_Library_v01.SpotAnim()


""" Walking parameters """
b_height = 200
h_amp = 100# horizontal step length
v_amp = 40 #vertical step length
track = 58.09
x_offset = 0 #body offset in x direction 
ra_longi = 30# body distance 
ra_lat = 30#20
steering =200 #Initial steering radius (arbitrary)
walking_direction = 90/180*pi #Initial steering angle (arbitrary)
stepl = 0.125 #duration of leg lifting typically between 0.1 and 0.2

Angle = [0, 0]

center_x = steering*cos(walking_direction) #steering center x relative to body center 
center_y = steering*sin(walking_direction) #steering center y relative to body center
cw =1

""" Joystick Init """

pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

""" XBOX One controller settings """
""" use Essai_Joystick_01.py utility to find out the right parameters """

nj = 6 # Number of joysticks
nb = 10 # number of buttons

but_walk = 7
but_sit = 2
but_lie = 3
but_twist = 1
but_pee = 0
but_move = 4
but_anim = 5

pos_frontrear = 4
pos_leftright = 3
pos_turn = 0    
pos_rightpaw = 5
pos_leftpaw = 2




joypos =np.zeros(6) #xbox one controller has 6 analog inputs
joybut = np.zeros(10) #xbox one controller has 10 buttons

continuer = True
clock = pygame.time.Clock()
t = 0 #Initializing timing/clock
tstart = 1 #End of start sequence
tstop = 1000 #Start of stop sequence by default
tstep = 0.01 #Timing/clock step
tstep1 = tstep

distance =[] #distance to support polygon edge
balance =[] #balance status (True or False)
timing = [] #timing to plot distance


Free = True #Spot is ready to receive new command
sitting = False 
walking = False #walking sequence activation
lying = False 
twisting = False
pawing = False
shifting = False
peeing = False
stop = False # walking stop sequence activation
lock = False # locking key/button stroke as a "rebound filter"
lockmouse = False
mouseclick = False
cw = 1
walking_speed = 0
walking_direction = 0
steeering = 1e6
module = 0
joype = -1 # Initial joysick value for peeing
joypar = -1 # Initial joysick value for pawing right
joypal = -1 # Initial joysick value for pawing left

Tcomp = 0.02

x_spot = [0, x_offset, Spot.xlf, Spot.xrf, Spot.xrr, Spot.xlr,0,0,0]
y_spot = [0,0,Spot.ylf+track, Spot.yrf-track, Spot.yrr-track, Spot.ylr+track,0,0,0]
z_spot = [0,b_height,0,0,0,0,0,0,0]
theta_spot = [0,0,0,0,0,0]

"""theta_spot = [x angle ground, y angle ground, z angle body in space, x angle body, y angle body, z angle body] """

stance = [True, True, True, True] # True = foot on the ground, False = Foot lifted

#theta xyz of ground then theta xyz of frame/body
pos_init = [-x_offset,track,-b_height,-x_offset,-track,-b_height,-x_offset,-track,-b_height,-x_offset,track,-b_height]

thetalf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[0], pos_init[1], pos_init[2], 1)[0]
thetarf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[3], pos_init[4], pos_init[5], -1)[0]
thetarr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[6], pos_init[7], pos_init[8], -1)[0]
thetalr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[9], pos_init[10], pos_init[11], 1)[0]

CG = SpotCG.CG_calculation (thetalf,thetarf,thetarr,thetalr)
#Calculation of CG absolute position
M = Spot.xyz_rotation_matrix(theta_spot[0],theta_spot[1],theta_spot[2],False)
CGabs = Spot.new_coordinates(M,CG[0],CG[1],CG[2],x_spot[1],y_spot[1],z_spot[1])
dCG = SpotCG.CG_distance(x_spot[2:6],y_spot[2:6],z_spot[2:6],CGabs[0],CGabs[1],stance)

x_spot = [0, x_offset, Spot.xlf, Spot.xrf, Spot.xrr, Spot.xlr,CG[0],CGabs[0],dCG[1]]
y_spot = [0,0,Spot.ylf+track, Spot.yrf-track, Spot.yrr-track, Spot.ylr+track,CG[1],CGabs[1],dCG[2]]
z_spot = [0,b_height,0,0,0,0,CG[2],CGabs[2],dCG[3]]

pos = [-x_offset,track,-b_height,-x_offset,-track,-b_height,-x_offset,-track,-b_height,-x_offset,track,-b_height,theta_spot,x_spot,y_spot,z_spot]


"""
Main Loop
"""
while (continuer):
        clock.tick(50)     
        
        for event in pygame.event.get(): # User did something.
            if event.type == pygame.QUIT: # If user clicked close window.
                continuer = False     
            if event.type == pygame.MOUSEBUTTONDOWN: 
                mouseclick = True
            else:
                mouseclick = False
                
        for i in range (0,nj): #read analog joystick position
            joypos[i] = joystick.get_axis(i)                        
        for i in range (0,nb):  #read buttons
            joybut[i] = joystick.get_button(i)
        joyhat = joystick.get_hat(0)  #read hat  
        
        """Animation"""
        
        if (joybut[but_walk] == 0)&(joybut[but_pee] == 0)&(joybut[but_twist] == 0)&(joybut[but_sit] == 0)&(joybut[but_lie] == 0)&(joybut[but_anim] == 0)&(joybut[but_move] == 0)&(lock == True):
            lock = False
        
        #WALKING        
        if (joybut[but_walk] == 1)&(walking == True)&(stop == False)&(lock == False): #Quit walk mode
            stop = True
            lock = True
            if (abs(t-int(t))<=tstep):
                tstop = int(t)
            else:
                tstop = int(t)+1
            
            if (t==0):
                tstop = 1
        
        if (joybut[but_walk] == 1)&(walking == False)&(Free == True): #Enter in walk mode
            walking = True
            stop = False
            Free = False
            t=0
            tstart = 1
            tstop = 1000
            lock = True
            trec = int(t)
            

         #SITTING and GIVING PAW
        if (joybut[but_sit] == 1)&(sitting == False)&(Free == True): #Enter in sitting mode
            sitting = True
            stop = False
            Free = False
            t=0
            lock = True
            

        if (joybut[but_sit] == 1)&(sitting == True)&(stop == False)&(lock == False): #Quit sitting mode
            stop = True
            lock = True


        #SHIFTING and PEEING
        if (joybut[but_pee] == 1)&(shifting == False)&(Free == True): #Enter in sitting mode
            shifting = True
            stop = False
            Free = False
            t=0
            lock = True
        
            
        if (joybut[but_pee] == 1)&(shifting == True)&(stop == False)&(lock == False): #Quit sitting mode
            stop = True
            lock = True
                       

        #LYING
        if (joybut[but_lie] == 1)&(lying == False)&(Free == True): #Enter in sitting mode
            lying = True
            stop = False
            Free = False
            t=0
            lock = True
            

        if (joybut[but_lie] == 1)&(lying== True)&(stop == False)&(lock == False): #Quit sitting mode
            stop = True
            lock = True
            


        #TWISTING
        if (joybut[but_twist] == 1)&(twisting == False)&(Free == True): #Enter in sitting mode
            twisting = True
            Free = False
            t=0
            lock = True
        
        
        
        if (walking == True):  
            coef = 1.2
            #set walking direction and speed            
            #set steering radius
            
            if (joybut[but_move] == True)&(tstep > 0)&(lock == False):
                tstep = 0
                lock = True
              
            if (joybut[but_move] == True)&(tstep == 0)&(lock == False):
                tstep = tstep1
                lock = True    
                
            print (tstep)    
                
            if (abs(joypos[pos_leftright])>0.2)|(abs(joypos[pos_frontrear])>0.2)|(stop == True):                
                t=t+tstep
                trec = int(t)+1
                
                module_old = module
                walking_direction_old = walking_direction
                steering_old = steering
                
                x_old = module_old*cos(walking_direction_old)
                y_old = module_old*sin(walking_direction_old)
                
                #update request
                module = sqrt(joypos[pos_leftright]**2 + joypos[pos_frontrear]**2)
                walking_direction = (atan2(-joypos[pos_leftright],-joypos[pos_frontrear])%(2*pi)+pi/2)%(2*pi)
                
                x_new = module*cos(walking_direction)
                y_new = module*sin(walking_direction)
                                
                #steering update                
                if (abs(joypos[pos_turn]) < 0.2):
                    cw = 1
                    if (steering<2000):
                        steering = min(1e6,steering_old*coef) 
                    else:
                        steering = 1e6
                else:
                    steering = 2000-(abs(joypos[0])-0.2)*2000/0.8+0.001
                    if ((steering/steering_old)>coef):                       
                        steering = steering_old*coef
                    if ((steering_old/steering)>coef):                       
                        steering = steering_old/coef   
                        if (steering <0.001):
                            steering = 0.001
                    cw = -np.sign(joypos[0])
                
                
                gap = sqrt((x_new-x_old)**2+(y_new-y_old)**2)
                
                if (gap>0.01):
                    x_new = x_old+ (x_new-x_old)/gap*0.01
                    y_new = y_old+ (y_new-y_old)/gap*0.01
                    module = sqrt(x_new**2+y_new**2)
                    walking_direction = atan2(y_new,x_new)
                                                       
                #reduces speed sideways and backwards  
                min_h_amp = h_amp*(1/2e6*steering+1/2)               
                xa = 1+cos(walking_direction-pi/2) 
                walking_speed = min (1, module) * min(h_amp,min_h_amp) * (1/8*xa**2+1/8*xa+1/4)                
                
                
            if ((abs(joypos[pos_leftright])<0.2)&(abs(joypos[pos_frontrear])<0.2))&(stop == False):  
                t=t+tstep                
                module = max (0, module-0.01)
                walking_speed = module* h_amp * ((1+cos(walking_direction-pi/2))/2*0.75+0.25)
                if (steering<2000):
                    steering = min(1e6,steering*coef) 
                else:
                    steering = 1e6
                cw=1    
                if (t>trec): 
                    t=trec

            """ 
            If you have an IMU that measures Angle[0] and Angle [1] 
            values can be transferred to theta_spot
            """                
            theta_spot[3] = Angle [0] # angle around x axis
            theta_spot[4] = Angle [1] # angle around y axis
            theta_spot[0] = Angle [0] # angle around x axis
            theta_spot[1] = Angle [1] # angle around y axis
                        
            if (t< tstart):           
                pos = Spot.start_walk_stop (track,x_offset,steering,walking_direction,cw,walking_speed,v_amp,b_height,stepl,t,tstep,theta_spot,x_spot,y_spot,z_spot,'start')
            else:
                if (t<tstop):
                    pos = Spot.start_walk_stop (track,x_offset,steering,walking_direction,cw,walking_speed,v_amp,b_height,stepl,t,tstep,theta_spot,x_spot,y_spot,z_spot,'walk')
                else:
                    pos = Spot.start_walk_stop (track,x_offset,steering,walking_direction,cw,walking_speed,v_amp,b_height,stepl,t,tstep,theta_spot,x_spot,y_spot,z_spot,'stop')    
                           
            theta_spot = pos[12]
            x_spot = pos[13]
            y_spot = pos[14]                 
            z_spot = pos[15]
            
            
            if (t>(tstop+1-tstep)):
                stop = False
                walking = False
                Free = True

        if (sitting == True):
            alpha_sitting = -30/180*pi 
            alpha_pawing = 0/180*pi
            L_paw = 220
            
            x_end_sitting = Spot.xlr-Spot.L2 + Spot.L1*cos(pi/3) +Spot.Lb/2*cos(-alpha_sitting) - Spot.d*sin (-alpha_sitting)
            z_end_sitting = Spot.L1*sin(pi/3)+ Spot.Lb/2*sin(-alpha_sitting) + Spot.d*cos(-alpha_sitting)
            start_frame_pos = [0,0,0,x_offset,0,b_height] # x,y,z rotations then translations

            #end_frame_pos = [0,0,0,x_offset,0,b_height-20] # x,y,z rotations then translations
            end_frame_pos = [0,alpha_sitting,0, x_end_sitting,0,z_end_sitting] # x,y,z rotations then translations
            pos = Spot.moving (t, start_frame_pos,end_frame_pos, pos)
            
            if (t==1)&(pawing == False):
                pos_sit_init = pos
            
            if (t == 1): #pawing is possible
                if (pawing == True):
                    #print (pos_sit_init[3],pos_sit_init[5])
                    pos[3] = pos_sit_init[3]+ (L_paw*cos(alpha_pawing)-pos_sit_init[3])*(joypar+1)/2
                    pos[5] = pos_sit_init[5]+ (-Spot.d-L_paw*sin(alpha_pawing)-pos_sit_init[5])*(joypar+1)/2
                    
                    pos[0] = pos_sit_init[0]+ (L_paw*cos(alpha_pawing)-pos_sit_init[0])*(joypal+1)/2
                    pos[2] = pos_sit_init[2]+ (-Spot.d-L_paw*sin(alpha_pawing)-pos_sit_init[2])*(joypal+1)/2
                    
                    thetarf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[3], pos[4], pos[5], -1)[0]
                    thetalf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[0], pos[1], pos[2], -1)[0]
                    #update of right front leg absolute position
                    legrf = Spot.FK(thetarf,-1) 
                    leglf = Spot.FK(thetalf,-1) 
                    xlegrf =Spot.xrf+pos[3]
                    ylegrf =Spot.yrf+pos[4]
                    zlegrf =pos[5]
                    xleglf =Spot.xlf+pos[0]
                    yleglf =Spot.ylf+pos[1]
                    zleglf =pos[2]
                    
                    theta_spot_sit = pos[12]
                    
                    x_spot_sit = pos[13]
                    y_spot_sit = pos[14]
                    z_spot_sit = pos[15]
                    
                    M = Spot.xyz_rotation_matrix(theta_spot_sit[3],theta_spot_sit[4],theta_spot_sit[2]+theta_spot_sit[5],False)
                    
                    paw_rf = Spot.new_coordinates(M,xlegrf,ylegrf,zlegrf,x_spot_sit[1],y_spot_sit[1],z_spot_sit[1])
                    paw_lf = Spot.new_coordinates(M,xleglf,yleglf,zleglf,x_spot_sit[1],y_spot_sit[1],z_spot_sit[1])
                    
                    x_spot_sit[3] = paw_rf[0]
                    y_spot_sit[3] = paw_rf[1]
                    z_spot_sit[3] = paw_rf[2]
                    x_spot_sit[2] = paw_lf[0]
                    y_spot_sit[2] = paw_lf[1]
                    z_spot_sit[2] = paw_lf[2]
                    
                    
                    pos[13] = x_spot_sit
                    pos[14] = y_spot_sit
                    pos[15] = z_spot_sit
                    
                joypar_old = joypar    
                if (joypal == -1):
                    if (((joypos[pos_rightpaw] != 0)&(joypos[pos_rightpaw] !=-1))|(joypar != -1)):
                        pawing = True
                        if (joypos[pos_rightpaw]>= joypar):
                             joypar = min (joypos[pos_rightpaw],joypar+0.05)
                        else:
                             joypar = max (joypos[pos_rightpaw],joypar-0.05)
                    else:
                        pawing = False     
                    
                if (joypar_old == -1):
                    if (((joypos[pos_leftpaw] != 0)&(joypos[pos_leftpaw] !=-1))|(joypal != -1)):
                        pawing = True
                        if (joypos[pos_leftpaw]>= joypal):
                             joypal = min (joypos[pos_leftpaw],joypal+0.05)
                        else:
                             joypal = max (joypos[pos_leftpaw],joypal-0.05)
                    else:
                        pawing = False         
                    

            if (stop == False):
                t=t+4*tstep
                if (t>=1):
                    t= 1
            elif (pawing == False):
                t=t-4*tstep
                if (t<= 0):
                    t= 0
                    stop  = False
                    sitting = False
                    Free = True

        if (shifting == True):                        
            x_end_shifting = ra_longi
            y_end_shifting = -ra_lat
            start_frame_pos = [0,0,0,x_offset,0,b_height] # x,y,z rotations then translations
            end_frame_pos = [0,0,0, x_end_shifting+x_offset,y_end_shifting,b_height] # x,y,z rotations then translations
            pos = Spot.moving (t, start_frame_pos,end_frame_pos, pos)
            
            if (t==1)&(peeing == False):
                pos_shift_init = pos
            
            if (t == 1): #lifting left hid leg is possible 
                
                if (peeing == True): 
                    pos[9] = pos_shift_init[9]+ (0-pos_shift_init[9])*(joype+1)/2
                    pos[10] = pos_shift_init[10]+ (130-pos_shift_init[10])*(joype+1)/2
                    pos[11] = pos_shift_init[11]+ (-20-pos_shift_init[11])*(joype+1)/2
                    
                    thetalr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[9], pos[10], pos[11], 1)[0]
                    #update of left hind leg absolute position
                    leglr = Spot.FK(thetalr,1)                    
                    xleglr =Spot.xlr+pos[9]
                    yleglr =Spot.ylr+pos[10]
                    zleglr =pos[11]
                    theta_spot_shift = pos[12]
                    x_spot_shift = pos[13]
                    y_spot_shift = pos[14]
                    z_spot_shift = pos[15]
                    M = Spot.xyz_rotation_matrix(theta_spot_shift[3],theta_spot_shift[4],theta_spot_shift[2]+theta_spot_shift[5],False)
                    pee_lr = Spot.new_coordinates(M,xleglr,yleglr,zleglr,x_spot_shift[1],y_spot_shift[1],z_spot_shift[1])
                    
                    x_spot_shift[5] = pee_lr[0]
                    y_spot_shift[5] = pee_lr[1]
                    z_spot_shift[5] = pee_lr[2]
                    pos[13] = x_spot_shift
                    pos[14] = y_spot_shift
                    pos[15] = z_spot_shift
                    
                if ((joypos[pos_leftpaw] != 0)&(joypos[pos_leftpaw] !=-1))|(joype != -1):
                    peeing = True
                    if (joypos[pos_leftpaw]>= joype):
                         joype = min (joypos[pos_leftpaw],joype+0.1)
                    else:
                         joype = max (joypos[pos_leftpaw],joype-0.1)
                else:
                    peeing = False
                
                    
                    

            if (stop == False):
                t=t+4*tstep
                if (t>=1):
                    t= 1
            elif (peeing == False):
                t=t-4*tstep
                if (t<= 0):
                    t= 0
                    stop  = False
                    shifting = False
                    Free = True

        
        if (lying == True):
            angle_lying = 40/180*pi
            x_end_lying= Spot.xlr-Spot.L2 + Spot.L1*cos(angle_lying)+Spot.Lb/2
            z_end_lying = Spot.L1*sin(angle_lying)+Spot.d
            start_frame_pos = [0,0,0,x_offset,0,b_height] # x,y,z rotations then translations
            end_frame_pos = [0,0,0, x_end_lying,0,z_end_lying] # x,y,z rotations then translations
            pos = Spot.moving (t, start_frame_pos,end_frame_pos, pos)
            if (stop == False):
                t=t+3*tstep
                if (t>=1):
                    t= 1
            else:
                t=t-3*tstep
                if (t<= 0):
                    t= 0
                    stop  = False
                    lying = False
                    Free = True
        
        if (twisting == True):
            x_angle_twisting = 0/180*pi
            y_angle_twisting = 0/180*pi
            z_angle_twisting = 30/180*pi
            start_frame_pos = [0,0,0,x_offset,0,b_height] # x,y,z rotations then translations
            
            t=t+4*tstep
            if (t>=1):
               t=1
               twisting = False
               Free = True
                
            if (t<0.25):
                end_frame_pos = [x_angle_twisting,y_angle_twisting,z_angle_twisting, x_offset,0,b_height] # x,y,z rotations then translations
                pos = Spot.moving (t*4, start_frame_pos,end_frame_pos, pos)                
            if (t>=0.25)&(t<0.5):
                end_frame_pos = [x_angle_twisting,y_angle_twisting,z_angle_twisting, x_offset,0,b_height] # x,y,z rotations then translations
                pos = Spot.moving ((t-0.25)*4, end_frame_pos,start_frame_pos, pos)  
            if (t>=0.5)&(t<0.75):
                end_frame_pos = [-x_angle_twisting,-y_angle_twisting,-z_angle_twisting, x_offset,0,b_height]
                pos = Spot.moving ((t-0.5)*4, start_frame_pos,end_frame_pos, pos) 
            if (t>=0.75)&(t<=1):
                end_frame_pos = [-x_angle_twisting,-y_angle_twisting,-z_angle_twisting, x_offset,0,b_height]
                pos = Spot.moving ((t-0.75)*4, end_frame_pos,start_frame_pos, pos)     
             
            
        xc = steering* cos(walking_direction)
        yc = steering* sin(walking_direction)
            
        center_x = x_spot[0]+(xc*cos(theta_spot[2])-yc*sin(theta_spot[2])) #absolute center x position
        center_y = y_spot[0]+(xc*sin(theta_spot[2])+yc*cos(theta_spot[2])) #absolute center y position


        
        thetalf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[0], pos[1], pos[2], 1)[0]
        thetarf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[3], pos[4], pos[5], -1)[0]
        thetarr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[6], pos[7], pos[8], -1)[0]
        thetalr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[9], pos[10], pos[11], 1)[0]
        
        """
        ************************************************************************************************
        
        thetalf, thetarf, thetarr, thetalr are the sets of angles thant can be sent to the servos 
        to generate the motion of Spotmicro
        
        This is where you can place the call to the servo moving function
        Moving function depends on the type of servos and drivers that are used to control them 
        -I2c shields, PWM generators
        -Servos maximum race 180°, 270°, 360°...
           
        Servos zero positions and races must be tuned 
        
        ************************************************************************************************ 
        """
        

 
        stance = [False, False, False, False]
        if (pos[15][2] < 0.01):            
            stance[0] = True
        if (pos[15][3] < 0.01):           
            stance[1] = True
        if (pos[15][4] < 0.01):           
            stance[2] = True
        if (pos[15][5] < 0.01):              
            stance[3] = True
        
        
        SpotAnim.animate(pos,t,pi/12,-135/180*pi,Angle,center_x,center_y,thetalf,thetarf,thetarr,thetalr,walking_speed,walking_direction,steering,stance)
        #SpotAnim.animate(pos,t,pi/2,-0/180*pi,Angle,center_x,center_y,thetalf,thetarf,thetarr,thetalr,walking_speed,walking_direction,steering,stance)
        #SpotAnim.animate(pos,t,0,-0/180*pi,Angle,center_x,center_y,thetalf,thetarf,thetarr,thetalr,walking_speed,walking_direction,steering,stance)

        pygame.display.flip()
        if (Free == True):
            sleep(0.1)
        
        """ CG update """
        CG = SpotCG.CG_calculation (thetalf,thetarf,thetarr,thetalr)
        #Calculation of CG absolute position
        M = Spot.xyz_rotation_matrix(theta_spot[0],theta_spot[1],theta_spot[2],False)
        CGabs = Spot.new_coordinates(M,CG[0],CG[1],CG[2],x_spot[1],y_spot[1],z_spot[1])
        dCG = SpotCG.CG_distance(x_spot[2:6],y_spot[2:6],z_spot[2:6],CGabs[0],CGabs[1],stance)
        
        
        pos[13][6] = CG[0] #x
        pos[14][6] = CG[1] #y
        pos[15][6] = CG[2] #z
        
        pos[13][7] = CGabs[0] #x
        pos[14][7] = CGabs[1] #y
        pos[15][7] = CGabs[2] #z
        
        pos[13][8] = dCG[1] #xint
        pos[14][8] = dCG[2] #yint
        pos[15][8] = dCG[3] #balance
        
        distance.append(dCG[0])
        timing.append(t) 
                                          
pygame.quit()

            