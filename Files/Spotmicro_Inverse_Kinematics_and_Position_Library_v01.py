# -*- coding: utf-8 -*-
"""
@author: Arnaud Villeneuve

This file contains a class of paramaters and functions to calculate Spot micro position
It includes :
-The main dimensions of Spot micro
-The center of gravity position and weight of each spotmicro limbs and body
-The forward kinematics function
-The forward kinematics functions
-The function that generates walking positions
-The function that generates positions from known start and end positions


"""

from math import pi, sin, cos, asin, acos, atan2, sqrt
import numpy as np


class Spot:
    """" Spotmicro dimensions """
    Wb = 78 #Shoulder/hip width
    Lb = 187.1 #Shoulder to hip length
    L0 = 58.09 #Shoulder articulation width
    d = 10.73 #Shoulder articulation height
    L1 = 108.31 #Leg length
    L2 = 138 #Foreleg length
    
    """ Anchor points """
    
    """ Front left shoulder"""
    xlf = 93.55
    ylf = 39
    zlf = 0
    
    """Front right shoulder"""
    xrf = 93.55
    yrf = -39
    zrf = 0
    
    """Rear  left hip """
    xlr = -93.55
    ylr = 39
    zlr = 0
    
    """Rear right hip """
    xrr = -93.55
    yrr = -39
    zrr = 0
    
    
    """Inertia Centers"""
    
    """ Body"""
    xCG_Body = 0
    yCG_Body = 0
    zCG_Body = 0
    Weight_Body = 897
    
    """ Left Shoulder """
    xCG_Shoulder = 0
    yCG_Shoulder = 5 #minus for right leg
    zCG_Shoulder = -9
    Weight_Shoulder = 99.3
    
    """ Left Leg """
    xCG_Leg = 0
    yCG_Leg = 0 #minus for right leg
    zCG_Leg = -31
    Weight_Leg = 133.3
    
    """ Left Leg """
    xCG_Foreleg = 0
    yCG_Foreleg = 0 #minus for right leg
    zCG_Foreleg = -28
    Weight_Foreleg = 107
    
    
    seq= [0, 0.5, 0.25, 0.75]
    phase = pi/8 # optimum position when leg is fully lifted
        
    def interp(x1,x2,steps):
        out = np.zeros(steps)
        for i in range (steps):
            out[i] = x1 +(x2-x1)/(steps-1)*i
        return out
     
    def interp1(x1,x2,i,steps):
        out = x1 +(x2-x1)/(steps-1)*i
        return out
    
    
    
    def xyz_rotation_matrix (self,thetax,thetay,thetaz,inverse):
        if (inverse == True):
            #Rx*Ry*Rz
            t2 = cos(thetay)
            t3 = sin(thetaz)
            t4 = cos(thetaz)
            t5 = sin(thetay)
            t6 = cos(thetax)
            t7 = sin(thetax)
            M = [t2*t4,t3*t6+t4*t5*t7,t3*t7-t4*t5*t6,-t2*t3,t4*t6-t3*t5*t7,t4*t7+t3*t5*t6,t5,-t2*t7,t2*t6]  
        else:
            #Rz*Ry*Rx
            t2 = cos(thetaz);
            t3 = sin(thetax);
            t4 = sin(thetaz);
            t5 = cos(thetax);
            t6 = sin(thetay);
            t7 = cos(thetay);
            M = [t2*t7,t4*t7,-t6,-t4*t5+t2*t3*t6,t2*t5+t3*t4*t6,t3*t7,t3*t4+t2*t5*t6,-t2*t3+t4*t5*t6,t5*t7]
        return M
    

    def new_coordinates (self,M,x,y,z,x0,y0,z0):
        xout= x0 + M[0]*x + M[3]*y + M[6]*z
        yout= y0 + M[1]*x + M[4]*y + M[7]*z
        zout= z0 + M[2]*x + M[5]*y + M[8]*z
        return [xout,yout,zout]
        

    
    def foot_coordinate (self,x,y,z,thetax,thetay):
        cx = cos(thetax)
        sx = sin(thetax)
        cy = cos(thetay)
        sy = sin(thetay)
        xf = x*cy+ z*sy
        yf = y*cx - z*cy*sx + x*sx*sy
        zf = y*sx + z*cx*cy - x*cx*sy
        return [xf,yf,zf]

        
    def IK (self,L0, L1, L2, d, x, y, z, side): #Inverse Kinematics
        """
          s = 1 for left leg
          s = -1 for right leg
        """
        t2 = y**2
        t3 = z**2
        t4 = t2+t3
        t5 = 1/sqrt(t4)
        t6 = L0**2
        t7 = t2+t3-t6
        t8 = sqrt(t7)
        t9 = d-t8
        t10 = x**2
        t11 = t9**2
        t15 = L1**2
        t16 = L2**2
        t12 = t10+t11-t15-t16
        t13 = t10+t11
        t14 = 1/sqrt(t13)
        error = False
        try:
            theta1 = side*(-pi/2+asin(t5*t8))+asin(t5*y)
            theta2= -asin(t14*x)+asin(L2*t14*sqrt(1/t15*1/t16*t12**2*(-1/4)+1))
            theta3 =-pi + acos(-t12/2/(L1*L2))
            
        except ValueError:
            print ('ValueError IK')
            error = True  
            theta1=90
            theta2=90
            theta3=90
        
        theta = [theta1, theta2, theta3]
        return (theta,error)

    def FK (self, theta, side): #Forward Kinematics
        """ Calculation of articulation points """
        """
        s = 1 for left leg
        s = -1 for right leg
        """
        x_shoulder1 = 0
        y_shoulder1 = Spot.d*sin(theta[0])
        z_shoulder1 = -Spot.d*cos(theta[0])
        
        x_shoulder2 = 0
        y_shoulder2 =side*Spot.L0*cos(theta[0])+Spot.d*sin(theta[0])
        z_shoulder2 =side*Spot.L0*sin(theta[0])-Spot.d*cos(theta[0]) 
               
        x_elbow = -Spot.L1*sin(theta[1])
        y_elbow = side*Spot.L0*cos(theta[0])-(-Spot.d-Spot.L1*cos(theta[1]))*sin(theta[0])
        z_elbow = side*Spot.L0*sin(theta[0]) +(-Spot.d-Spot.L1*cos(theta[1]))*cos(theta[0])
        
        return [x_shoulder1,x_shoulder2,x_elbow,y_shoulder1,y_shoulder2,y_elbow,z_shoulder1,z_shoulder2,z_elbow]
    
    
    def FK_Weight (self, theta, side): #Forward Kinematics for calculation of Center of Gravity
        """ Calculation of articulation points """
        """
        side = 1 for left leg
        side = -1 for right leg
        """
                
        xCG_Shoulder1 = Spot.xCG_Shoulder
        yCG_Shoulder1 =side*Spot.yCG_Shoulder*cos(theta[0])-Spot.zCG_Shoulder*sin(theta[0])
        zCG_Shoulder1 =side*Spot.yCG_Shoulder*sin(theta[0])+Spot.zCG_Shoulder*cos(theta[0]) 
               
        xCG_Leg1 = Spot.xCG_Leg*cos(theta[1]) + Spot.zCG_Leg*sin(theta[1])
        yCG_Leg1 = cos(theta[0])*(Spot.L0*side + side*Spot.yCG_Leg) + sin(theta[0])*(Spot.d - Spot.zCG_Leg*cos(theta[1]) + Spot.xCG_Leg*sin(theta[1]))
        zCG_Leg1 = sin(theta[0])*(Spot.L0*side + side*Spot.yCG_Leg) - cos(theta[0])*(Spot.d - Spot.zCG_Leg*cos(theta[1]) + Spot.xCG_Leg*sin(theta[1]))
        
        xCG_Foreleg1 = cos(theta[1])*(Spot.xCG_Foreleg*cos(theta[2]) + Spot.zCG_Foreleg*sin(theta[2])) - sin(theta[1])*(Spot.L1 - Spot.zCG_Foreleg*cos(theta[2]) + Spot.xCG_Foreleg*sin(theta[2]))
        yCG_Foreleg1 = cos(theta[0])*(Spot.L0*side + side*Spot.yCG_Foreleg) + sin(theta[0])*(Spot.d + sin(theta[1])*(Spot.xCG_Foreleg*cos(theta[2]) + Spot.zCG_Foreleg*sin(theta[2])) + cos(theta[1])*(Spot.L1 - Spot.zCG_Foreleg*cos(theta[2]) + Spot.xCG_Foreleg*sin(theta[2])))
        zCG_Foreleg1 = sin(theta[0])*(Spot.L0*side + side*Spot.yCG_Foreleg) - cos(theta[0])*(Spot.d + sin(theta[1])*(Spot.xCG_Foreleg*cos(theta[2]) + Spot.zCG_Foreleg*sin(theta[2])) + cos(theta[1])*(Spot.L1 - Spot.zCG_Foreleg*cos(theta[2]) + Spot.xCG_Foreleg*sin(theta[2])))
        
        return [xCG_Shoulder1,xCG_Leg1,xCG_Foreleg1,yCG_Shoulder1,yCG_Leg1,yCG_Foreleg1,zCG_Shoulder1,zCG_Leg1,zCG_Foreleg1]
                          
    """
    Walking Function that generates the walking positions
    """
    
    def start_walk_stop (self,track,x_offset,steering_radius, steering_angle,cw,h_amp,v_amp,height,stepl,t,tstep,theta_spot,x_spot,y_spot,z_spot,step_phase):            

         alpha = np.zeros(4)
         alphav =np.zeros(4)

         theta_spot_updated = theta_spot
 
         CG = [x_spot[6],y_spot[6],z_spot[6]]
       
         """ Steering center coordinates in spot frame """
         xc = steering_radius* cos(steering_angle)
         yc = steering_radius* sin(steering_angle)
         
         #rotation matrix for frame position
         #Mf = Spot.xyz_rotation_matrix (self,frame_pos[0],frame_pos[1],frame_pos[2],False)
         
         Ms = Spot.xyz_rotation_matrix (self,0,0,theta_spot_updated[2],False)
 
         s = Spot.new_coordinates(self,Ms,xc,yc,0,x_spot[0],y_spot[0],z_spot[0])
         xs = s[0]
         ys = s[1]
         
         """ Nominal Foot Position """        
         xn = [Spot.xlf, Spot.xrf,Spot.xrr, Spot.xlr]
         yn = [Spot.ylf+track,Spot.yrf-track,Spot.yrr-track,Spot.ylr+track]
         
         radii = np.zeros(4)
         an = np.zeros(4)
         for i in range (0,4): 
             """ Steering radius """ 
             radii[i] = sqrt((xc-xn[i])**2+(yc-yn[i])**2)
             """ Foot nominal angle"""  
             an[i] = atan2(yn[i]-yc,xn[i]-xc)   
         
         """ Motion angle """
         maxr = max(radii)
         mangle = h_amp/maxr 
         
         """ Rotation angle and translation calculation"""    
         if (step_phase =='start')|(step_phase == 'stop'):           
             dtheta = mangle/(1-stepl)*tstep/2*cw
         else:
             dtheta = mangle/(1-stepl)*tstep*cw
         theta_spot_updated[2] = dtheta + theta_spot[2]
         
         #Matrix from local body frame to absolute space frame
         Ms_updated = Spot.xyz_rotation_matrix (self,theta_spot_updated[3],theta_spot_updated[4],theta_spot_updated[2]+theta_spot_updated[5],False)
         #Matrix from absolute space frame to local body frame
         Msi_updated = Spot.xyz_rotation_matrix (self,-theta_spot_updated[3],-theta_spot_updated[4],-(theta_spot_updated[2]+theta_spot_updated[5]),True)
         #Delta rotation matric from from body center to absolute space frame
         dMs = Spot.xyz_rotation_matrix (self,0,0,dtheta,False)
                  
         """ Foot nominal center absolute position"""
         foot_center = Spot.new_coordinates(self,dMs,x_spot[0]-xs, y_spot[0]-ys,0,xs,ys,0)         
                         
         t1 = t%1
         kcomp = 1
         stance = [True,True,True,True]
        
         for i in range (0,4):
            alphav[i] =0 
            if (t1<=Spot.seq[i]):
                 stance[i] = True #Leg is on the ground (absolute position value unchanged)
            else:        
                 if (t1<(Spot.seq[i]+stepl)):
                     
                     stance[i] = False #leg is lifted (absolute position value changes)
                     alphav[i] = -pi/2+2*pi/stepl*(t1-Spot.seq[i])
                     t2 = Spot.seq[i]+stepl
                     if (step_phase == 'start'):
                         #End position alpha 
                         alpha[i] = -Spot.seq[i]/(1-stepl)/2 + (t2-Spot.seq[i])/stepl/(1-stepl)*Spot.seq[i]  
                     if (step_phase == 'stop'):                          
                         alpha[i] = -1/2 + Spot.seq[i]/(1-stepl)/2 + (t2-Spot.seq[i])/stepl*(1-Spot.seq[i]/(1-stepl)) 
                     if (step_phase == 'walk'):                                                 
                         alpha[i] = -1/2  + ((t2-Spot.seq[i])/stepl) 
                 else:         
                     stance[i] = True #Leg is on the ground (absolute position value unchanged)

         """ Compensation Calculation """
         stance_test = np.sum(stance) #if sum = 4 all feet are on the floor --> body balance
         
         #absolute stance area target point
         #Barycenter of sustentation area with higher weight of diagonal points
         weight = 1.2
         x_abs_area = np.zeros(4)
         y_abs_area = np.zeros(4)
         
         
         x_abs_area[0] = ((x_spot[3]+x_spot[5])*weight+x_spot[4])/(2*weight+1) 
         y_abs_area[0] = ((y_spot[3]+y_spot[5])*weight+y_spot[4])/(2*weight+1)                                 
         x_abs_area[1] = ((x_spot[2]+x_spot[4])*weight+x_spot[5])/(2*weight+1) 
         y_abs_area[1] = ((y_spot[2]+y_spot[4])*weight+y_spot[5])/(2*weight+1)                   
         x_abs_area[2] = ((x_spot[3]+x_spot[5])*weight+x_spot[2])/(2*weight+1) 
         y_abs_area[2] = ((y_spot[3]+y_spot[5])*weight+y_spot[2])/(2*weight+1)  
         x_abs_area[3] = ((x_spot[2]+x_spot[4])*weight+x_spot[3])/(2*weight+1) 
         y_abs_area[3] = ((y_spot[2]+y_spot[4])*weight+y_spot[3])/(2*weight+1) 

         if  (stance_test == 4): 
             istart = 0
             iend = 0
             #identify transition start and target
             tstart = (int(t1/0.25)*0.25)
             tend = tstart+0.25
             if (tend==1):
                 tend = 0
              
             for i in range (0,4):
                 if (tstart == Spot.seq[i]):
                     istart = i
                 if (tend  == Spot.seq[i]):
                     iend = i
             
             if (t1>(Spot.seq[istart]+stepl)):        
                 x_abs_comp= x_abs_area[istart]+(x_abs_area[iend]-x_abs_area[istart])*(t1-tstart-stepl)/(0.25-stepl)
                 y_abs_comp= y_abs_area[istart]+(y_abs_area[iend]-y_abs_area[istart])*(t1-tstart-stepl)/(0.25-stepl) 
             else:
                 x_abs_comp = x_abs_area[istart]
                 y_abs_comp = y_abs_area[istart] 
         else:
            for i in range (0,4):
                if (stance[i]==0):
                    x_abs_comp = x_abs_area[i] 
                    y_abs_comp = y_abs_area[i]
             
         Msi_comp = Spot.xyz_rotation_matrix (self,0,0,-theta_spot_updated[2],True) 
         #compensation calculation in body center frame
         comp= Spot.new_coordinates(self,Msi_comp,x_abs_comp-x_spot[0],y_abs_comp-y_spot[0],0,0,0,0)                     
         """ Compensation calculation with theta """
         v_amp_t = v_amp
         ts = 0.25
         if (step_phase == 'start'):
             if (t1< ts):
                 kcomp = t1/ts
                 v_amp_t = 0
         elif (step_phase == 'stop'):  
             if (t1 > (1-ts)):
                 kcomp = (1-t1)/ts
                 v_amp_t = 0
         Ms_comp  =  Spot.xyz_rotation_matrix (self,0,0,theta_spot_updated[2],False)  
         #Compensation calculation absoltute space frame
         compt =  Spot.new_coordinates(self,Ms_comp,(comp[0]-CG[0])*kcomp+x_offset,(comp[1]-CG[1])*kcomp,0,0,0,0)
         """ Frame center new position with gravity center compensation, offset and height """
         x_framecenter_comp = foot_center[0] + compt[0]
         y_framecenter_comp = foot_center[1] + compt[1] 
         z_framecenter_comp = height
                 
         """ New Frame corners position absolute including compensation """
         x_frame = [Spot.xlf, Spot.xrf, Spot.xrr, Spot.xlr]
         y_frame = [Spot.ylf, Spot.yrf, Spot.yrr, Spot.ylr]
         z_frame = [0,0,0,0]
         
         x_framecorner = np.zeros(4)
         y_framecorner = np.zeros(4)
         z_framecorner = np.zeros(4)

         for i in range (0,4): 
             #Body corners calculation in absolute space frame
             frame_corner = Spot.new_coordinates(self,Ms_updated,x_frame[i],y_frame[i],z_frame[i],x_framecenter_comp,y_framecenter_comp,z_framecenter_comp)
             x_framecorner[i] = frame_corner[0]
             y_framecorner[i] = frame_corner[1]
             z_framecorner[i] = frame_corner[2]

        
         xleg = np.zeros(4)
         yleg = np.zeros(4)
         zleg = np.zeros(4)
         xabs = np.zeros(4)
         yabs = np.zeros(4)
         zabs = np.zeros(4)
         xint = np.zeros(4)
         yint = np.zeros(4)
         zint = np.zeros(4)
         
         for i in range (0,4):              
            if stance[i] == False:
                 #relative position calculation (used for inverse kinematics)
                 alphah = an[i]+mangle*alpha[i]*cw
                 xleg_target = xc + radii[i]*cos(alphah) -(comp[0]-CG[0])*kcomp -x_offset -x_frame[i]
                 yleg_target = yc + radii[i]*sin(alphah) -(comp[1]-CG[1])*kcomp -y_frame[i]
                 
                 leg_current = Spot.new_coordinates(self,Msi_comp,x_spot[i+2]-x_framecorner[i],y_spot[i+2]-y_framecorner[i],-z_framecorner[i],0,0,0)
                 #interpolate between current position and targe
                 if ((Spot.seq[i]+stepl-t1)>tstep):
                     xint[i] = leg_current[0]+(xleg_target - leg_current[0])*(tstep)/(Spot.seq[i]+stepl-t1)
                     yint[i] = leg_current[1]+(yleg_target - leg_current[1])*(tstep)/(Spot.seq[i]+stepl-t1)
                 else:
                     xint[i] = xleg_target 
                     yint[i] = yleg_target   
                 zint[i] = leg_current[2] + v_amp_t*(1+sin(alphav[i]))/2                 
                 #print (leg_current[2],zint[i],leg_current[2]-zint[i])
                 Msi_body = Spot.xyz_rotation_matrix (self,-theta_spot_updated[3],-theta_spot_updated[4],-theta_spot_updated[5],True) 
                 legs = Spot.new_coordinates(self,Msi_body,xint[i],yint[i],zint[i],0,0,0)
                 xleg[i]= legs[0]
                 yleg[i]= legs[1]
                 zleg[i]= legs[2]
                 
                 #absolute foot position 
                 #Msb_updated = Spot.xyz_rotation_matrix (self,0,0,theta_spot_updated[2]+theta_spot_updated[5],False)
                 foot_abs = Spot.new_coordinates(self,Ms_updated,xleg[i],yleg[i],zleg[i],x_framecorner[i],y_framecorner[i],z_framecorner[i])
                 

                 xabs[i] = foot_abs[0]
                 yabs[i] = foot_abs[1]
                 zabs[i] = foot_abs[2]
                          
            else:
                 xabs[i] = x_spot[i+2]
                 yabs[i] = y_spot[i+2]
                 zabs[i] = 0
                 
                 #relative foot position of foot on the ground/floor for inverse kinematics
                 leg = Spot.new_coordinates(self,Msi_updated,xabs[i]-x_framecorner[i],yabs[i]-y_framecorner[i],zabs[i]-z_framecorner[i],0,0,0)
                 xleg[i] = leg[0]
                 yleg[i] = leg[1]
                 zleg[i] = leg[2]
                 
         x_spot_updated =  [foot_center[0],x_framecenter_comp, xabs[0], xabs[1], xabs[2], xabs[3],x_spot[6],x_spot[7],x_spot[8]] 
         y_spot_updated =  [foot_center[1],y_framecenter_comp, yabs[0], yabs[1], yabs[2], yabs[3],y_spot[6],y_spot[7],y_spot[8]] 
         z_spot_updated =  [foot_center[2],z_framecenter_comp, zabs[0], zabs[1], zabs[2], zabs[3],z_spot[6],z_spot[7],z_spot[8]] 
         
         
         pos = [xleg[0],yleg[0],zleg[0],xleg[1],yleg[1],zleg[1],xleg[2],yleg[2],zleg[2],xleg[3],yleg[3],zleg[3],theta_spot_updated,x_spot_updated,y_spot_updated,z_spot_updated]    
         return pos


    """"
    Moving Function from known start and end positions (used for sitting, lying, etc...)
    """
    
    def moving (self,t, start_frame_pos,end_frame_pos, pos):
        
        theta_spot_updated = pos[12]
        x_spot_updated =  pos[13]
        y_spot_updated =  pos[14]
        z_spot_updated =  pos[15]
        
    
        #interpolate new frame position 
        frame_pos = np.zeros(6)
        
        for i in range (0,6):
            frame_pos[i] = start_frame_pos[i]  + (end_frame_pos[i]- start_frame_pos[i])*t
        
        theta_spot_updated [3] = frame_pos[0]
        theta_spot_updated [4] = frame_pos[1]
        theta_spot_updated [5] = frame_pos[2]
        #rotation matrix for frame position
        Mf = Spot.xyz_rotation_matrix (self,frame_pos[0],frame_pos[1],frame_pos[2],False)
        
        #rotation matrix for spot position (only around z axis)
        Ms = Spot.xyz_rotation_matrix (self,0,0,theta_spot_updated[2],False)
        
        # frame corners position coordinaterelative to frame center
        x_frame = [Spot.xlf, Spot.xrf, Spot.xrr, Spot.xlr]
        y_frame = [Spot.ylf, Spot.yrf, Spot.yrr, Spot.ylr]
        z_frame = [0,0,0,0]
        
        #New absolute frame center position
        frame_center_abs = Spot.new_coordinates(self,Ms,frame_pos[3],frame_pos[4],frame_pos[5],x_spot_updated[0],y_spot_updated[0],z_spot_updated[0])
   
        #absolute frame corners position coordinates  
        x_frame_corner_abs = np.zeros(4)
        y_frame_corner_abs = np.zeros(4)
        z_frame_corner_abs = np.zeros(4)
                    
        for i in range (0,4):
            frame_corner = Spot.new_coordinates(self,Mf,x_frame[i],y_frame[i],z_frame[i],0,0,0)
            frame_corner_abs = Spot.new_coordinates(self,Ms,frame_corner[0],frame_corner[1],frame_corner[2],frame_center_abs[0],frame_center_abs[1],frame_center_abs[2])
            x_frame_corner_abs[i] = frame_corner_abs[0]
            y_frame_corner_abs[i] = frame_corner_abs[1]
            z_frame_corner_abs[i] = frame_corner_abs[2]
        
        #calculate current relative position
        xleg = np.zeros(4)
        yleg = np.zeros(4)
        zleg = np.zeros(4)
               
        
        #Leg relative position to front corners
        Mi = Spot.xyz_rotation_matrix(self,-theta_spot_updated[3],-theta_spot_updated[4],-(theta_spot_updated[2]+theta_spot_updated[5]),True)   

        for i in range (0,4):                        
            leg = Spot.new_coordinates(self,Mi,x_spot_updated[i+2]-x_frame_corner_abs[i],y_spot_updated[i+2]-y_frame_corner_abs[i],z_spot_updated[i+2]-z_frame_corner_abs[i],0,0,0)
            xleg[i] = leg[0]
            yleg[i] = leg[1]
            zleg[i] = leg[2]
        
        
        x_spot_updated[1] = frame_center_abs [0]     
        y_spot_updated[1] = frame_center_abs [1]          
        z_spot_updated[1] = frame_center_abs [2]    

                
        pos = [xleg[0],yleg[0],zleg[0],xleg[1],yleg[1],zleg[1],xleg[2],yleg[2],zleg[2],xleg[3],yleg[3],zleg[3],theta_spot_updated,x_spot_updated,y_spot_updated,z_spot_updated]    
        return pos
        
        
    
