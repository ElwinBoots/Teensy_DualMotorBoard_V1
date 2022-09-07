# -*- coding: utf-8 -*-
"""
Created on Wed Jan 16 21:26:08 2019

@author: Elwin
"""
import numpy as np

def make3( p , v , a , j , Ts):
    
    p=abs(p)
    
    
    jd = j;      # required for discrete time calculations
    
    # Calculation t1
    t1 = (p/(2*j))**(1/3) ; # largest t1 with bound on jerk
    t1 = np.ceil(t1/Ts)*Ts; 
    jd = 1/2*p/(t1**3); 
    
    # velocity test
    if v < jd*t1**2:         # v bound violated ?
       t1 = (v/j)**(1/2) ;  # t1 with bound on velocity not violated
       t1 = np.ceil(t1/Ts)*Ts; 
       jd = v/(t1**2); 

    # acceleration test
    if a < jd*t1:     # a bound violated ?
       t1 = a/j ;    # t1 with bound on acceleration not violated
       t1 = np.ceil(t1/Ts)*Ts; 
       jd = a/t1; 

    j = jd;  # as t1 is now fixed, jd is the new bound on jerk
    
    # Calculation t2
    t2 = (t1**2/4+p/j/t1)**(1/2) - 3/2*t1 ;   # largest t2 with bound on acceleration
    t2 = np.ceil(t2/Ts)*Ts; 
    jd = p/( 2*t1**3 + 3*t1**2*t2 + t1*t2**2 ); 
    
    # velocity test
    if v < (jd*t1**2 + jd*t1*t2):   # v bound violated ?
       t2 = v/(j*t1) - t1 ;       # t2 with bound on velocity not violated
       t2 = np.ceil(t2/Ts)*Ts; 
       jd = v/( t1**2 + t1*t2 ); 
    
    j = jd;  # as t2 is now fixed, jd is the new bound on jerk
    
    # Calculation t3
    t3 = (p - 2*j*t1**3 - 3*j*t1**2*t2 - j*t1*t2**2)/v ; # t3 with bound on velocity
    t3 = np.ceil(t3/Ts)*Ts; 
    jd = p/( 2*t1**3 + 3*t1**2*t2 + t1*t2**2 + t1**2*t3 + t1*t2*t3 ); 

    return t1 , t2 , t3, jd