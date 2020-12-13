#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: Arnaud Villeneuve

This file contains a class of functions to calculate the center of gravity position
and distance to the support polygon edge

"""

from math import sqrt
import Spotmicro_Inverse_Kinematics_and_Position_Library_v01
Spot = Spotmicro_Inverse_Kinematics_and_Position_Library_v01.Spot()


class SpotCG:
    def CG_calculation (self,thetalf,thetarf,thetarr,thetalr):
        cgposlf=(Spot.FK_Weight(thetalf,1))
        cgposrf=(Spot.FK_Weight(thetarf,-1))
        cgposrr=(Spot.FK_Weight(thetarr,-1))
        cgposlr=(Spot.FK_Weight(thetalr,1))
        
        Weightsum = Spot.Weight_Body+4*(Spot.Weight_Shoulder+Spot.Weight_Leg+Spot.Weight_Foreleg)
        
        xcglf=(cgposlf[0]+Spot.xlf)*Spot.Weight_Shoulder+(cgposlf[1]+Spot.xlf)*Spot.Weight_Leg+(cgposlf[2]+Spot.xlf)*Spot.Weight_Foreleg
        xcgrf=(cgposrf[0]+Spot.xrf)*Spot.Weight_Shoulder+(cgposrf[1]+Spot.xrf)*Spot.Weight_Leg+(cgposrf[2]+Spot.xrf)*Spot.Weight_Foreleg
        xcgrr=(cgposrr[0]+Spot.xrr)*Spot.Weight_Shoulder+(cgposrr[1]+Spot.xrr)*Spot.Weight_Leg+(cgposrr[2]+Spot.xrr)*Spot.Weight_Foreleg
        xcglr=(cgposlr[0]+Spot.xlr)*Spot.Weight_Shoulder+(cgposlr[1]+Spot.xlr)*Spot.Weight_Leg+(cgposlr[2]+Spot.xlr)*Spot.Weight_Foreleg
        xcg= (xcglf+xcgrf+xcgrr+xcglr+Spot.xCG_Body*Spot.Weight_Body)/Weightsum
        
        ycglf=(cgposlf[3]+Spot.ylf)*Spot.Weight_Shoulder+(cgposlf[4]+Spot.ylf)*Spot.Weight_Leg+(cgposlf[5]+Spot.ylf)*Spot.Weight_Foreleg
        ycgrf=(cgposrf[3]+Spot.yrf)*Spot.Weight_Shoulder+(cgposrf[4]+Spot.yrf)*Spot.Weight_Leg+(cgposrf[5]+Spot.yrf)*Spot.Weight_Foreleg
        ycgrr=(cgposrr[3]+Spot.yrr)*Spot.Weight_Shoulder+(cgposrr[4]+Spot.yrr)*Spot.Weight_Leg+(cgposrr[5]+Spot.yrr)*Spot.Weight_Foreleg
        ycglr=(cgposlr[3]+Spot.ylr)*Spot.Weight_Shoulder+(cgposlr[4]+Spot.ylr)*Spot.Weight_Leg+(cgposlr[5]+Spot.ylr)*Spot.Weight_Foreleg
        ycg= (ycglf+ycgrf+ycgrr+ycglr+Spot.yCG_Body*Spot.Weight_Body)/Weightsum
        
        zcglf=(cgposlf[6]+Spot.zlf)*Spot.Weight_Shoulder+(cgposlf[7]+Spot.zlf)*Spot.Weight_Leg+(cgposlf[8]+Spot.zlf)*Spot.Weight_Foreleg
        zcgrf=(cgposrf[6]+Spot.zrf)*Spot.Weight_Shoulder+(cgposrf[7]+Spot.zrf)*Spot.Weight_Leg+(cgposrf[8]+Spot.zrf)*Spot.Weight_Foreleg
        zcgrr=(cgposrr[6]+Spot.zrr)*Spot.Weight_Shoulder+(cgposrr[7]+Spot.zrr)*Spot.Weight_Leg+(cgposrr[8]+Spot.zrr)*Spot.Weight_Foreleg
        zcglr=(cgposlr[6]+Spot.zlr)*Spot.Weight_Shoulder+(cgposlr[7]+Spot.zlr)*Spot.Weight_Leg+(cgposlr[8]+Spot.zlr)*Spot.Weight_Foreleg
        zcg= (zcglf+zcgrf+zcgrr+zcglr+Spot.zCG_Body*Spot.Weight_Body)/Weightsum
        
        return (xcg,ycg,zcg)
    
    
    def CG_distance (self,x_legs,y_legs,z_legs,xcg,ycg,stance):
        
        #line equation c * x + s * y - p  = 0
        # with c = a/m et s = b/m
         
        a1 = (y_legs[0]-y_legs[2])
        b1 = -(x_legs[0]-x_legs[2])
        m1 =sqrt(a1**2 + b1**2)
        c1 = a1/m1
        s1 = b1/m1
        
        a2 = (y_legs[1]-y_legs[3])
        b2 = -(x_legs[1]-x_legs[3])
        m2 =sqrt(a2**2 + b2**2)
        c2 = a2/m2
        s2 = b2/m2   
        
        p1 = c1*x_legs[0] + s1*y_legs[0]
        p2 = c2*x_legs[1] + s2*y_legs[1]
        
        """ Dstance calculation """
        d1 = c1*xcg + s1*ycg - p1
        d2 = c2*xcg + s2*ycg - p2
        
        """ intersection calculation """
        #perpendicalar line equation -s * x + c * y - q = 0
        
        q1 = -s1*xcg +c1*ycg
        q2 = -s2*xcg +c2*ycg
        
        xint1 = c1*p1 - s1*q1
        yint1 = c1*q1 + s1*p1
        
        xint2 = c2*p2 - s2*q2
        yint2 = c2*q2 + s2*p2
        
        """ Check if inside sustentation triangle """
        d = 0
        xint = xcg
        yint = ycg
        if (stance[0]== False)|(stance[2]== False): 
            d = d2
            xint = xint2
            yint = yint2
            
        
        if (stance[1]== False)|(stance[3]== False): 
            d = d1
            xint = xint1
            yint = yint1  
            
        balance = True
    
        if (stance[0] == False)&(d< 0):
            balance = False
            
        if (stance[1] == False)&(d> 0):
            balance = False    
        
        if (stance[2] == False)&(d> 0):
            balance = False    
            
        if (stance[3] == False)&(d< 0):
            balance = False  
            
        if (balance == False):
            d=-abs(d)
        else:
            d=abs(d)
        
        return (d,xint,yint,balance)
