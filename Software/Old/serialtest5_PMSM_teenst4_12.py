#%%
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 29 19:25:07 2018

@author: Elwin
"""

from os import chdir
chdir('c:\\users\\elwin\\desktop\\python')


import serial
import numpy as npgeget
from scipy import signal as sig
import matplotlib.pyplot as plt
import matplotlib.ticker as tickercom
import struct
import warnings
import matplotlib.cbook
import time
import make3
import math
import pandas as pd
from tqdm import tqdm
import numpy as np
plt.rcParams["figure.dpi"] = 200
import time
import random
import pyqtgraph as pg
from collections import deque
from pyqtgraph.Qt import QtGui, QtCore

from PyQt5.QtWidgets import (
    QApplication,
    QGridLayout,
    QPushButton,
    QWidget,
)

f_pwm = 30e3
Ts = 1/(2*f_pwm)

n_trace = 14; #Was 10. USB packets are up to 2048 kB. 4 B per value -> 512 values. 16 (14 + 2 overhead)values fit exactly 32 times in this 512 values. 
# n_trace = 30; 

# com = 'COM10' 1
com = 'COM5'

motor = 'motor1'


#for i in tqdm(range(10)):
#    time.sleep(0.1)

global ser

try:
    ser = serial.Serial( com  , timeout= 0.1 )  # open serial port
    print(ser.name)         # check which port was really used
except:
    print(ser.name)         # check which port was really used

from numpy import (atleast_1d, poly, polyval, roots, real, asarray,
                   resize, pi, absolute, logspace, r_, sqrt, tan, log10,
                   arctan, arcsinh, sin, cos, exp, cosh, arccosh, ceil, conjugate,
                   zeros, sinh, append, concatenate, prod, ones, array,
                   mintypecode)
from numpy.polynomial.polynomial import polyval as npp_polyval
warnings.filterwarnings("ignore",category=matplotlib.cbook.mplDeprecation)

def tracesignal( signals, t ):
    setTrace( signals )
    t,s = readData( int(t /Ts) )
    plt.figure();
    plt.plot(t,s[:,0:len(signals)], );
    plt.legend( signals )
    plt.show()
    return t,s[:,0:len(signals)]


def bode( H ,f  , name = '' , title = 'bode plot'):
    mag = np.abs( H )
    phase = np.angle( H )
    ax1 = plt.subplot(2,1,1)
#    plt.loglog(f, mag)    # Bode magnitude plot
    plt.semilogx(f, 20*np.log10(mag) , label = name)    # Bode magnitude plot
    plt.grid( 1 , 'both' , 'both')
    plt.ylabel('Magnitude [dB]')
    plt.title(title)
    plt.legend()
#    ax1.xaxis.set_minor_formatter(ticker.ScalarFormatter())
    ax2 = plt.subplot(2,1,2, sharex=ax1)
    plt.plot(f, phase * 180 / np.pi)  # Bode phase plot
    plt.grid( 1 , 'both')
    plt.xlabel('Frequency [Hz]')
    plt.ylabel('Phase [deg]')
    plt.ylim( -182, 182)
    plt.tight_layout()
    plt.show()   

def restart( ser=ser ):
    ser.write( b'o' + struct.pack('f',  0) ) # Restart controller

def setcont( contnum ):
    ser.write( b'C' + struct.pack('I',  contnum ) )

def setenc( encnum ):
    if encnum == 2:
        setpar('EncSelect_FB' , 2)
        ser.write( b'C' + struct.pack('I',  2) ) # 30 Hz BW
        ser.write( b'o' + struct.pack('f',  0) ) # Restart controller
    else:
        setpar('EncSelect_FB' , 1)
        ser.write( b'C' + struct.pack('I',  2) ) # 30 Hz BW
        ser.write( b'o' + struct.pack('f',  0) ) # Restart controller

def moverel( pos ):
    p = pos
    v = 30*np.pi
    a = 300*np.pi
    j = 40000*np.pi
    [t1, t2, t3, jd] = prepSP(  p , v , a , j )
    ser.write( b'p' + struct.pack('f',  0) )
    ser.write( b'p' + struct.pack('f',  0) )

def moveabs( pos ):
    posrel = pos - getsig('rmech')
    if posrel != 0:
        moverel( posrel )

def nyquist( H ,f  , name = '' , title = 'nyquist plot'):
    plt.plot( np.real( H ) , np.imag( H ) , label = name)    # Bode magnitude plot
    tmp = 0.5 * np.exp( -1j * np.arange( 0 , 2*np.pi , 0.01) ) - 1
    plt.plot( np.real( tmp ) , np.imag( tmp ) , label = '6 dB')    # Bode magnitude plot
    plt.grid( 1 , 'both')
    plt.title(title)
    plt.legend()
    plt.xlabel('Real')
    plt.ylabel('Imaginary')
    plt.xlim( -2 , 2)
    plt.ylim( -2 , 2)
    plt.tight_layout()
    plt.show()


def freqrespd( num , den ,f , Ts ):
    zm1 = exp(-1j * 2*np.pi * f*Ts )
    H = (npp_polyval(zm1, num, tensor=False) / npp_polyval(zm1, den, tensor=False))
    return H
   
def leadlag( fBW , alpha1, alpha2, Ts):
    fs = 1/Ts
    alpha = np.sqrt( 1/ alpha2**2 + 1 ) / np.sqrt( alpha1**2 + 1 )
    w0 = fBW * 2 * np.pi / alpha1
    wp = fBW * 2 * np.pi * alpha2
    a1 = (wp -2*fs) / (wp + 2*fs)
    b0 = alpha * ( ( 1 + 2*fs/w0 ) / ( 1 + 2*fs/wp ))
    b1 = alpha * ( ( 1 - 2*fs/w0 ) / ( 1 + 2*fs/wp ))
    num = [b0 , b1]
    den = [1 , a1]
    return num, den

def lowpass2( f0, damp,  Ts):
    w0 = 2 * np.pi * f0 * Ts
    alpha =sin(w0) * damp
    b0 = 0.5 * (1 - cos(w0)) / (1 + alpha) 
    b1 = 2 * b0
    b2 = b0 
    a1 = (-2 * cos(w0)) / (1 + alpha)
    a2 = (1 - alpha) / (1 + alpha)
    num = [b0 , b1, b2]
    den = [1  , a1, a2]
    return num, den

def Integrator( f0, Ts):
    b0 = f0 * 2 * pi * Ts/2
    b1 = b0
    num = [b0, b1]
    den = [1  , -1]
    return num, den

def fig():
    plt.figure()
    line1, = plt.plot( t , sens , label='sens')
    line1, = plt.plot( t , e , label='e')
    line1, = plt.plot( t , r , label='r')
    line1, = plt.plot( t , dist , label='dist')
    line1, = plt.plot( t , Vout , label='Vout')
    line1, = plt.plot( t , ss_f , label='ss_f')
    line1, = plt.plot( t , enc2 , label='enc2')
    line1, = plt.plot( t , emech , label='emech')
    plt.grid( 1 , 'both')
    plt.legend()
    plt.show()
    
#    tbegin = 1
#    tlength = 1
#    
#    jbegin = int( tbegin / Ts)
#    jend = jbegin + int( tlength / Ts)
#    
#    plt.figure()
#    line1, = plt.step( t[jbegin:jend] , sens[jbegin:jend] , label='sens')
#    line1, = plt.step( t[jbegin:jend] , e[jbegin:jend] , label='e')
#    line1, = plt.step( t[jbegin:jend] , r[jbegin:jend] , label='r')
#    line1, = plt.step( t[jbegin:jend] , dist[jbegin:jend] , label='dist')
#    line1, = plt.step( t[jbegin:jend] , Vout[jbegin:jend] , label='Vout')
#    line1, = plt.step( t[jbegin:jend] , ss_f[jbegin:jend] , label='ss_f')
#    line1, = plt.step( t[jbegin:jend] , enc2[jbegin:jend] , label='enc2')
#    plt.grid( 1 , 'both')
#    plt.legend()
#    plt.show()

def readData(N , ser=ser):
    Nsig = 2+n_trace
    N_total = 0
    setpar('Nsend' , N , ser)
    if N < 100:
        buffer = ser.read(N*4*Nsig)
    else:
        buffer = ser.readall()

    n = np.zeros( int(N) , dtype=np.uint32 )
    t = np.zeros( int(N) ) 
    s = np.zeros( ( int(N) , Nsig-2 ))
    
    sigtypes = ['I' , 'I' ] + ser.tracesigtypes
    for sigtype in set(sigtypes):
        indices = [i for i in range(len(sigtypes)) if sigtypes[i] == sigtype]
        indices = np.array(indices)
        if sigtype == 'b':
            tmp = np.ndarray( ( int(N) , Nsig ) , 'bool', buffer[0::4] )
        else:
            tmp = np.ndarray( ( int(N) , Nsig ) , sigtype, buffer )
        if sigtype == 'I':
            n = tmp[:,indices[0]]
            t = tmp[:,indices[1]] / 1e6
            s[:,indices[2:]-2] = tmp[:,indices[2:]]
        else:
            s[:,indices-2] = tmp[:,indices]

    if N > 1:
        fit = np.polyfit( np.arange(0,N) , t , 1 )
        
        t -= t[0]
        Ndownsample = np.min( np.diff(n ))
        if max( np.diff(n , axis=0) ) > Ndownsample:
            print('data seems BAD (diff n > 1)')
        elif (max(abs((np.diff( t) - Ts*Ndownsample))) < 5e-6) &  (np.abs(Ts*Ndownsample -fit[0]) < 5e-6):
            print('data seems OK')
        else:
            print('data seems BAD, Ts not OK')
            plt.figure(100)
            line1, = plt.plot( np.diff( t ,axis=0) *1e6 , label='Using set_dashes()')
            plt.grid( 1 , 'both')
            plt.show()
    return t,s


def readall( ser=ser):
    setpar('sendall' , 1 , ser)
    buffer = ser.readall()
    
    sigtypes = ser.sigtypes
    s = np.zeros( ( 1, len(sigtypes)  ))
    for sigtype in set(sigtypes):
        indices = [i for i in range(len(sigtypes)) if sigtypes[i] == sigtype]
        indices = np.array(indices)
        if sigtype == 'b':
            tmp = np.ndarray( ( 1, len(sigtypes) ) , 'bool', buffer[0::4] )
        else:
            tmp = np.ndarray( ( 1 , len(sigtypes) ) , sigtype, buffer )

        s[0 ,indices] = tmp[:,indices]
    df = pd.DataFrame( s[0: , 0:len(sigtypes)], columns=ser.signames )
    return df


def trace( time ):
    t,s = readData( np.ceil(time/Ts).astype('int') )
    df = pd.DataFrame( s[:,0:len(ser.signalsnames)], columns=ser.signalsnames )
    df.index = t
    return df

def getFFT( signal , Ndownsample , j0 = int(1/Ts)  ):
#    j0 = int(1/Ts) # These amount of samples are ignored
    L = 2047 * Ndownsample
    Naver = int( (len(signal)-j0)/L  )
    print( 'Naver =' , Naver )
    SIGNAL = np.fft.fft( signal[j0:j0+L*Naver] , axis = 0)
    f  = np.fft.fftfreq(L*Naver, Ts)
    SIGNAL = SIGNAL[f>0]
    SIGNAL = 2*SIGNAL[Naver-1::Naver]   
    f = f[f>0]
    f = f[Naver-1::Naver]
    SIGNAL = SIGNAL[f< 1/(2*Ts*Ndownsample)]
    f = f[f< 1/(2*Ts*Ndownsample)]
    return SIGNAL, f

def getFFT_SS( signal ):
    SIGNAL = np.fft.fft( signal, axis = 0)
    f  = np.fft.fftfreq( len(signal) , Ts)
    SIGNAL = 2* SIGNAL[f>0]; # / len(signal)
    f = f[f>0]
    return SIGNAL, f

def fft( signal , name = ''):
    L = len(signal)
    SIGNAL = np.fft.fft( signal , axis = 0) / L
    f  = np.fft.fftfreq(L, Ts)
    SIGNAL = 2 * SIGNAL[f>0]
    f = f[f>0]
    bode( SIGNAL ,f  , name , title = 'FFT')
    return SIGNAL, f

def fftpsd( signal , name = '' , Ts=Ts ):
    L = len(signal)
    SIGNAL = np.fft.fft( signal , axis = 0) / L
    f  = np.fft.fftfreq(L, Ts)
    SIGNAL = 2 * SIGNAL[f>0] 
    f = f[f>0]   

    ax1 = plt.subplot(2,1,1)
    plt.loglog( f , abs(SIGNAL)**2 /Ts )
    plt.grid( 1 , 'both' , 'both')
    plt.ylabel('PSD [-²/Hz]')
    # plt.title(title)
    plt.legend()
    ax2 = plt.subplot(2,1,2, sharex=ax1)
    plt.semilogx( f , np.cumsum(abs(SIGNAL)**2 /Ts) *Ts )
    plt.grid( 1 , 'both')
    plt.xlabel('Frequency [Hz]')
    plt.ylabel('CPS [-²]')
    plt.tight_layout()
    plt.show()   

    return SIGNAL, f



def analyseSS( freq ):
    I = ss_f == freq
    plt.figure();
    plt.plot( t2[I]-t2[I][0] , dist[I] , label='disturbance')
    plt.plot( t2[I]-t2[I][0] , r[I] , label='current setpoint')
    plt.plot( t2[I]-t2[I][0] , Vout[I] , label='Vout')
    plt.plot( t2[I]-t2[I][0] , sens[I] , label='current sensor')    
    plt.plot( t2[I]-t2[I][0] , emech[I] , label='error')
    plt.grid( 1 , 'both')
    plt.legend()
    plt.show()

    plt.figure();
    fft( dist[I] , 'disturbance');
    fft( r[I] , 'current setpoint') ;
    fft( Vout[I] , 'Vout') ;
    fft( sens[I] , 'cursens' );
    fft( emech[I] , 'error' );

def makebodesOL( Ndownsample = 1 , fignum = None):
    DIST,f = getFFT( dist , Ndownsample )
    VOUT,f = getFFT( Vout , Ndownsample )
    SENS,f = getFFT( sens , Ndownsample )
    E,f = getFFT( e , Ndownsample )
    ENC2,f = getFFT( enc2 , Ndownsample )
    R,f = getFFT( r , Ndownsample )
    EMECH,f = getFFT( emech , Ndownsample )

    S = E / DIST
    CL = SENS / DIST
    P = SENS / VOUT
    OL = 1/S - 1
    
    if (fignum == None):
        plt.figure( fignum )
    else:
        plt.figure( fignum ); fignum += 1;    
    bode( CL , f , 'Closed loop')
      
#    plt.figure()
#    nyquist( OL , f , 'Open loop')
    
#    Relec = 1/0.45
#    Lelec = 1/((5e3*2*np.pi*0.07))
#    F = (Lelec * 1j * 2*np.pi*f + Relec)
#    delay = np.exp( 1.7 * Ts * -f * 2 * np.pi * 1j )
    if (fignum == None):
        plt.figure( fignum )
    else:
        plt.figure( fignum ); fignum += 1;    
    bode( P , f , 'Plant')
#    bode( 1/F , f , 'Plant fit')
#    bode( 1/F * delay , f , 'Plant fit, delayed')
    
#    plt.figure()
#    bode( OL / (1 + OL), f , 'closed loop, no FF')
#    bode( (OL + P*Relec )/ (1 + OL), f , 'closed loop, with R FF')
#    bode( (OL + P*F )/ (1 + OL) , f , 'closed loop, with R and L FF')
#    bode( (OL*delay + P*F )/ (1 + OL) , f , 'closed loop, with R and L FF, delay compensated')
    
    Kp = 2.7
    fInt = 700
    num, den = Integrator( fInt , Ts)
    INTEGRATOR = freqrespd( num , den ,f , Ts )
    
#    Use these if disturbance was input at voltage without a controller:
#    plt.figure()
#    bode( CL * Kp * (INTEGRATOR+1) , f )
#    plt.figure()
#    nyquist( CL * Kp * (INTEGRATOR+1) , f )
    
    
#    num, den = lowpass2( fLP , 0.7 , Ts)
#    LP2 = freqrespd( num , den ,f , Ts )

    if (fignum == None):
        plt.figure( fignum )
    else:
        plt.figure( fignum ); fignum += 1;    
    bode( OL , f , 'Open loop')
#    bode( P * Kp * (INTEGRATOR+1) , f , 'Open loop reconstructed')
    
    if (fignum == None):
        plt.figure( fignum )
    else:
        plt.figure( fignum ); fignum += 1;    
    nyquist( OL , f , 'Open loop')
#    nyquist( P * Kp * (INTEGRATOR+1) , f , 'Open loop reconstructed')

#    plt.figure()
#    bode( 1/(P * Kp * (INTEGRATOR+1) +1) , f , 'Sensitivity reconstructed')
#    bode( S, f , 'Sensitivity')


def makebodesCL( Ndownsample = 1 , fignum = None):
    DIST,f = getFFT( dist , Ndownsample )
    VOUT,f = getFFT( Vout , Ndownsample )
    SENS,f = getFFT( sens , Ndownsample )
    E,f = getFFT( e , Ndownsample )
    ENC2,f = getFFT( enc2 , Ndownsample )
    R,f = getFFT( r , Ndownsample )
    EMECH,f = getFFT( emech , Ndownsample )

    S = R / DIST
    OL = 1/S - 1
    
    Kt = 0.068 # Nm/A
#    enc2rad = -2*np.pi/2048
    PS = -EMECH / DIST/ Kt

    Pmech = PS / S

    Jmeas = 1/(abs( Pmech[20] ) * (2*np.pi*f[20])**2  )
    Jmot = 0.0000325
    
    if (fignum == None):
        plt.figure( fignum )
    else:
        plt.figure( fignum ); fignum += 1;   
    bode( Pmech , f , 'Pmech [rad/Nm]')
    
    #plt.figure()
    #bode( np.exp( -f * 2 * np.pi * Ts * 1j ) , f , 'time delay')
    
#    Kp = 4
#    fBW = 50
#    alpha1 = 3
#    alpha2 = 4
#    fInt = fBW / 6
#    fLP = fBW * 6
    Kp = 15
    fBW = 100
    alpha1 = 3
    alpha2 = 4
    fInt = fBW / 6
    fLP = fBW * 8
    
    num, den = leadlag( fBW , alpha1, alpha2, Ts)
    CONT = freqrespd( num , den ,f , Ts )
    num, den = lowpass2( fLP , 0.7 , Ts)
    LP2 = freqrespd( num , den ,f , Ts )
    num, den = Integrator( fInt , Ts)
    INTEGRATOR = freqrespd( num , den ,f , Ts )

#    plt.figure()
#    bode( CONT * LP2 * Pmech * Kp* (INTEGRATOR+1)  , f , 'OL reconstruct')


#    plt.figure()
#    bode( CONT * Kp , f , 'lead-lag')
#    bode( LP2 , f , 'LP')
#    bode( (INTEGRATOR+1) , f , 'INTEGRATOR')
#    bode( CONT * LP2 * Kp * (INTEGRATOR+1) , f , 'total')
    
    if (fignum == None):
        plt.figure( fignum )
    else:
        plt.figure( fignum ); fignum += 1;   
#    bode( CONT * LP2 * Pmech * Kp* (INTEGRATOR+1)  , f , 'OL reconstruct')
    bode( OL , f , 'Open loop')

    if (fignum == None):
        plt.figure( fignum )
    else:
        plt.figure( fignum ); fignum += 1;   
    nyquist( OL , f , 'Open loop')

    if (fignum == None):
        plt.figure( fignum )
    else:
        plt.figure( fignum ); fignum += 1;   
    bode( R / DIST  , f , 'S')

#    Kp = 37.6
#    fBW = 150
#    alpha1 = 3
#    alpha2 = 4
#    fInt = fBW / 6
#    fLP = fBW * 6
#    
#    num, den = leadlag( fBW , alpha1, alpha2, Ts)
#    CONT = freqrespd( num , den ,f , Ts )
#    num, den = lowpass2( fLP , 0.7 , Ts)
#    LP2 = freqrespd( num , den ,f , Ts )
#    num, den = Integrator( fInt , Ts)
#    INTEGRATOR = freqrespd( num , den ,f , Ts )
#
#    plt.figure()
#    bode( CONT * LP2 * Pmech * Kp* (INTEGRATOR+1)  , f , 'OL Tuned')
#    
#    plt.figure()
#    bode( 1/(1+CONT * LP2 * Pmech * Kp* (INTEGRATOR+1))  , f , 'S Tuned')



def start( ser ):
    try:
        ser.close();
        # ser.set_buffer_size(rx_size = 128000, tx_size = 128000)
        ser.open();
    except:
        print('Failed to open serial port')    
    ser.signames, ser.sigtypes = getsignalnames( ser );  # This generates overloads, TODO fix  
    ser.tracesigtypes= n_trace*['f']
    ser.signals= n_trace*[0]
    return ser

def setTrace( signals , ser=ser ):
    if isinstance( signals , str) or isinstance( signals , int):
        signals = [ signals ];
        
    ser.signalsnames = signals
    i=0
    for signal in ser.signalsnames:
        if isinstance( signal , str):
            signal = ser.signames.index( signal )
        ser.signals[i] = signal
        ser.tracesigtypes[i] = ser.sigtypes[signal]
        ser.write( b't' + struct.pack('I', signal) + bytes(bytearray([i])) )
        i+=1
    return signals
    
    
def getsig( signals , ser=ser):
    if isinstance( signals , str):
        signals = [ signals ];
    signals = setTrace( signals , ser)
    t,s = readData( 1 , ser)
    
    if len(signals) == 1:
        s = s[0,0];
    else:
        s = s[0,0:len(signals)];
#    print( s)
    return(s)


def setpar( signal , value , ser=ser):
    if isinstance( signal , str):
        signal = ser.signames.index( signal )
    ser.write( b'S' + struct.pack('I',  signal) + struct.pack( ser.sigtypes[signal] ,  value)  )

# def setpar_ser( signal , value ):
#     if isinstance( signal , str):
#         signal = ser.signames.index( signal )
#     ser.write( b'S' + struct.pack('I',  signal) + struct.pack( ser.sigtypes[signal] ,  value)  )



def getsignalnames( ser ):
    ser.write( b'T' + struct.pack('I', 0));
    a = ser.readlines()
    signames = []
    for signame in a[:10000:2]:
        signames.append(signame.strip().decode("utf-8"))
    sigtypes = []
    for signame in a[1:10000:2]:
        sigtypes.append(signame.strip().decode("utf-8"))
    return signames, sigtypes

#CLose the loop
def CL(ser=ser):
    ser = start( ser )
    # if (getsig( 'IndexFound1' , ser )+ getsig( 'IndexFound2' ,ser) ) < 2:
    #     setpar('ridethewave' , 1 , ser)
    #     time.sleep( 1 )
    # if getsig( 'IndexFound1' ,ser ) < 1:
    #     print('index not found')
    # else:  
        
    Lq = 2.6e-4 
    Ld = 2.2e-4
    R = 0.33
        
    # Lq = 5e-6 
    # Ld = 5e-6 
    # R = 0.069      
    # setpar('Lambda_m' , 60 / (sqrt(3) * pi * 2200 * 2 * 7) )
    
    setpar( 'Lq' , Lq , ser)
    setpar( 'Ld' , Ld , ser)
    setpar( 'R' , R , ser)
    setpar('useIlowpass' , 0 , ser)
    
    Jload = 6.77e-5 #kgm²
    vFF = 7e-5  #Nm/(rad/s)
    setpar( 'Jload' ,  Jload  , ser)
    setpar( 'velFF' ,  vFF  , ser)
    setpar( 'rmechoffset' , 0 , ser)
    setpar( 'rmechoffset' , -getsig( 'emech1'  , ser) , ser)
    getsig('emech1'  , ser)
    
    # setpar( 'Icontgain' , 1.54 ) # Current loop gain
    # ser.write( b'9' + struct.pack('f',  500) ) # Current loop integrator frequency
    
    L = (Ld + Lq) / 2
    
    setpar( 'Icontgain' , L * 1000 * 2 *pi  ) # Current loop gain
    ser.write( b'9' + struct.pack('f',  R/L/2/pi ) ) # Current loop integrator frequency
    
    # ser.write( b'C' + struct.pack('I',  1) ) # 100 Hz BW

    # setpar('encoderPos2offset' , 0 , ser)
    # setpar('encoderPos2offset' , int(getsig('encoderPos1') -getsig('encoderPos2') ))
    
#        ser.write( b'C' + struct.pack('I',  2) ) # Low BW
    #ser.write( b'C' + struct.pack('I',  3) ) # Very low BW
#        ser.write( b'C' + struct.pack('I',  4) ) # 300 Hz BW
  

#CLose the loop
def CL2(ser=ser):
    ser = start( ser )
    if (getsig( 'IndexFound1' , ser )+ getsig( 'IndexFound2' ,ser) ) < 2:
        setpar('ridethewave' , 1 , ser)
        setpar('ridethewave2' , 1 , ser)
        time.sleep( 0.2 )
        
        setpar( 'commutationoffset2',0)
        
        setpar('Valpha2_offset' , 2)
        time.sleep( 0.5 )
        
        offset = getsig('thetaPark2')
        
        setpar('Valpha2_offset' , 0)
        setpar( 'commutationoffset2', -offset ) 
        time.sleep( 1 )
    if (getsig( 'IndexFound1' , ser )+ getsig( 'IndexFound2' ,ser) ) < 2:
        print('index not found')
    else:  
        setpar( 'commutationoffset',0.75) #Motor 1
        # setpar( 'commutationoffset2',3.05) #Motor 2
        # setpar( 'commutationoffset2',0) #Motor 2 (maxon)

        Lq = 2.6e-4 
        Ld = 2.2e-4
        R = 0.33
        setpar( 'Lq' , Lq , ser)
        setpar( 'Ld' , Ld , ser)
        setpar( 'R' , R , ser)
        setpar('useIlowpass' , 0 , ser)
        
        Lq2 = 0.0002143
        Ld2 = 0.00023500
        R2 = 0.55
        setpar( 'Lq2' , Lq2 , ser)
        setpar( 'Ld2' , Ld2 , ser)
        
        Jload = 0 #kgm²
        vFF = 0 #Nm/(rad/s)
        setpar( 'Jload' ,  Jload  , ser)
        setpar( 'velFF' ,  vFF  , ser)
        setpar( 'rmechoffset' , 0 , ser)
        setpar( 'rmechoffset' , -getsig( 'emech1'  , ser) , ser)
        getsig('emech1'  , ser)
        
        setpar( 'rmechoffset2' , 0 , ser)
        setpar( 'rmechoffset2' , -getsig( 'emech2'  , ser) , ser)
        getsig('emech2'  , ser)
        # setpar( 'Icontgain' , 1.54 ) # Current loop gain
        # ser.write( b'9' + struct.pack('f',  500) ) # Current loop integrator frequency
        
        setpar( 'Icontgain' , 3  , ser) # Current loop gain
        ser.write( b'9' + struct.pack('f',  300) ) # Current loop integrator frequency
        
        setpar( 'Icontgain2' , Lq2*1000*2*pi  , ser) # Current loop 2 gain
        ser.write( b'0' + struct.pack('f',  R2/Lq2/2/pi ) ) # Current loop 2 integrator frequency
        
        # ser.write( b'C' + struct.pack('I',  4) ) # 300 Hz BW
        # ser.write( b'C' + struct.pack('I',  3) ) # 300 Hz BW
        
        fBW = 100
        setpar( 'fBW' , fBW)
        setpar( 'alpha1' , 3)
        setpar( 'alpha2' , 4)
        setpar( 'fInt' , fBW / 6)
        setpar( 'fLP' , fBW * 8)
        setpar( 'Kp' , 23.3)
               
        fBW = 70
        setpar( 'fBW2' , fBW)
        setpar( 'alpha1_2' , 3)
        setpar( 'alpha2_2' , 3)
        setpar( 'fInt2' , fBW / 6)
        setpar( 'fLP2' , fBW * 8)
        setpar( 'Kp2' , 3.84)
        setpar( 'Kp2' , 0)

        ser.write( b'C' + struct.pack('I',  0) ) # Set controllers
        
        # setpar('encoderPos2offset' , 0 , ser)
        # setpar('encoderPos2offset' , int(getsig('encoderPos1') -getsig('encoderPos2') ))
        
#        ser.write( b'C' + struct.pack('I',  2) ) # Low BW
        #ser.write( b'C' + struct.pack('I',  3) ) # Very low BW
#        ser.write( b'C' + struct.pack('I',  4) ) # 300 Hz BW
    

def prepSP(  p , v , a , j , ser=ser ):
    t1, t2, t3, jd = make3.make3( p , v , a , j , Ts );
    #double tstart, double t1, double t2, double t3, double p, double v_max, double a_max, double j_max
    ser.write( b'1' + struct.pack('f',  t1) )
    ser.write( b'2' + struct.pack('f',  t2) )
    ser.write( b'3' + struct.pack('f',  t3) )
    ser.write( b'4' + struct.pack('f',  p) )
    ser.write( b'5' + struct.pack('f',  v) )
    ser.write( b'6' + struct.pack('f',  a) )
    ser.write( b'7' + struct.pack('f',  jd) )
    return t1, t2, t3, jd
    
def res():
    ser.write( b'o' + struct.pack('f',  0) ) # Restart controller
    

# If you lose samples and/or timing of trace data is not OK it is likely due to 
# the PC not being fast enough to empty the buffers. e.g. running youtube in the 
# background on my laptop causes this problem. 

ser = start( ser )


#%% Find commutation offset M1
ser = start( ser )


setpar('ridethewave' , 1 , ser)
time.sleep( 1 )

setpar( 'commutationoffset',0) 


setpar('Valpha_offset' , 1)
time.sleep( 0.5 )
Valpha1 = getsig('Valpha_offset')
Ialpha1 = getsig('Ialpha')
Ibeta1 = getsig('Ibeta')
Ia1 = getsig('ia')
bus1 = getsig('sensBus')

setpar('Valpha_offset' , 2)

time.sleep( 0.5 )
Valpha2 = getsig('Valpha_offset')
Va = getsig('Va')
Vb = getsig('Vb')
Ialpha2 = getsig('Ialpha')
Ibeta2 = getsig('Ibeta')
bus2 = getsig('sensBus')
Ia2 = getsig('ia')

R = (Valpha2 - Valpha1) / (Ialpha2 - Ialpha1)

offset = getsig('thetaPark_enc')

setpar('Valpha_offset' , 0)
setpar( 'commutationoffset', -offset) 

thetaPark = getsig('thetaPark')

# CL()

#%% Find commutation offset M2 

setpar('ridethewave2' , 1 , ser)
time.sleep( 0.2 )

setpar( 'commutationoffset2',0)

setpar('Valpha2_offset' , 2)
time.sleep( 0.5 )

offset = getsig('thetaPark2')

setpar('Valpha2_offset' , 0)
setpar( 'commutationoffset2', -offset ) 


#%%
ser = start( ser )
CL()


#%%
# setpar('Lambda_m' , 1 * 60 / (sqrt(3) * pi * 2200 * 2 * 7) )
setpar('I_max' , 20)
# setpar('maxDutyCycle' , 0.99)

setpar('anglechoice' ,1 )
# setpar('observer_gain' , 6786*2)

# signals = [ 'Q' , 'Id_meas' , 'Iq_meas' , 'Va' , 'Vb', 'Vc' , 'Iq_SP'   , 'thetaPark', 'thetaPark_enc' , 'thetaPark_obs'   ,'BEMFa' , 'BEMFb']
signals = [ 'Iq_SP'  , 'Iq_meas' , 'Id_meas'  , 'Vq' , 'Vd' , 'Va' , 'Vb', 'Vc' , 'x1' , 'x2', 'thetaPark',  'thetaPark_obs', 'thetaPark_vesc' , 'erpm']
setTrace( signals )

setpar( 'Iq_SP' , 1)
time.sleep(0.1)
setpar( 'Iq_SP' , 7.5)

df = trace( 2 ) 
setpar( 'Iq_SP' , 0)
# t2,s2 = readData( int(1 /Ts) )

df.plot()

# df.filter(regex='thetaPark').plot()

# df.filter(regex='x').plot()


# df.filter(regex='thetaPark_obs').plot()


# erpm = pd.DataFrame( np.unwrap( (df['thetaPark'].values) , axis=0) , columns=['erpm']).diff( ) /2/pi*60 / Ts 
# erpm.index = df.index
# (erpm.rolling(20).sum()/20).plot( grid=1 , xlabel='Time [s]')


#%% HFI
maxsamples = int(100)
hfi_v = 10
i = 0;
Nangles = 360
curtotvec = np.zeros( int(Nangles) ) 
curorttotvec = np.zeros( int(Nangles) ) 
ANGLES = np.linspace( 0 , 2*pi, Nangles , 0)
# ANGLES = np.linspace( 0.7 , 0.8, Nangles , 0)
# ANGLES = np.linspace( pi/2 , pi/2, Nangles , 0)
DF = list()
for angle in ANGLES:

    setpar('hfi_cursample' , -2)
    setpar('hfi_maxsamples', maxsamples)
    setpar('hfi_curtot' , 0)
    setpar('hfi_curorttot' , 0)
    setpar('hfi_V' , hfi_v)
    # setpar('hfi_dir' , 0.5* pi)
    setpar('hfi_dir' , angle)
    setpar('hfi_on' , 1)
    
    
    # signals = [ 'Ialpha', 'Ibeta' , 'Valpha' , 'Vbeta' , 'hfi_curtot' ]
    # setTrace( signals )
    
    # DF.append(trace( 0.05 ) )
    while( getsig('hfi_cursample') < maxsamples) :
        a=1;
        
    # df.filter(regex='^hfi_curtot').plot()
    curtot = getsig('hfi_curtot')
    curorttot = getsig('hfi_curorttot')
        
    setpar('hfi_on' , 0)
    curtotvec[i] = curtot
    curorttotvec[i] = curorttot
   
    i += 1
    
angle = getsig('thetaPark')

L = hfi_v * (maxsamples/2) / (curtotvec ) * Ts * 1e6 #Not correct yet

SIGNAL = np.fft.fft( L / len(L) , axis = 0) 
f  = np.fft.fftfreq( len(L) , 1/len(L))
SIGNAL = 2 * SIGNAL[f>0]
f = f[f>0]
angleguess = np.angle(SIGNAL[1])
if angleguess < 0:
    angleguess += 2*pi

plt.figure()
plt.plot( ANGLES , L)
plt.plot( [angle , angle] , [ 175 , 200])
plt.plot( [angleguess , angleguess] , [ 175 , 200])


 



# plt.figure()
# plt.plot( ANGLES/2/pi*360  , curtotvec / maxsamples)
# plt.plot( ANGLES/2/pi*360 , curorttotvec / maxsamples)
# # plt.plot( [angle , angle] , [-0.25, 0.25])
# plt.grid()



#%% 


plt.figure()
plt.plot( ANGLES , hfi_v * maxsamples / (curtotvec ) * Ts * 1e6)




plt.figure()
plt.plot(np.abs(np.fft.fft( DF[0].Valpha.values , axis = 0) ) /len(DF[0].Valpha.values) )
plt.plot(np.abs(np.fft.fft( DF[0].Vbeta.values , axis = 0) ) /len(DF[0].Valpha.values) )

plt.figure()
plt.plot(np.abs(np.fft.fft( DF[1].Valpha.values , axis = 0) ) /len(DF[0].Valpha.values) )
plt.plot(np.abs(np.fft.fft( DF[1].Vbeta.values , axis = 0) ) /len(DF[0].Valpha.values) )

plt.figure()
plt.plot(np.abs(np.fft.fft( DF[0].Ialpha.values , axis = 0) ) /len(DF[0].Valpha.values) )
plt.plot(np.abs(np.fft.fft( DF[0].Ibeta.values , axis = 0) ) /len(DF[0].Valpha.values) )

plt.figure()
plt.plot(np.abs(np.fft.fft( DF[1].Ialpha.values , axis = 0) ) /len(DF[0].Valpha.values) )
plt.plot(np.abs(np.fft.fft( DF[1].Ibeta.values , axis = 0) ) /len(DF[0].Valpha.values) )





# sig , f = getFFT_SS( df.Ialpha.values )

# plt.figure()
# bode( sig , f )

# fftpsd( df.Ialpha.values )
# fftpsd( df.Valpha.values )



# signal = df.Valpha.values
# L = len(signal)
# SIGNAL = np.fft.fft( signal , axis = 0) / L
# f  = np.fft.fftfreq(L, Ts)
# SIGNAL = 2 * SIGNAL[f>0]
# f = f[f>0]
# bode( SIGNAL ,f  )



#%% RIDE THE WAVE
ser = start( ser )

signals = [ 'Q' , 'Id_meas' , 'Iq_meas' , 'tA' , 'tB', 'tC' , 'mod_alpha' , 'mod_beta' , 'encoderPos1' , 'encoderPos2']
setTrace( signals )
setpar('ridethewave' , 1)
t,s = readData( int(1 /Ts) )

plt.figure();
plt.plot(t,s[:,0:len(signals)], );
plt.legend( signals )
plt.show()

#%% 
setpar('VSP' , 1)
#%% 
setpar('VSP' , 2)
#%% 
setpar('VSP' , -1)
#%% 
setpar('VSP' , 0)

#%% Ride the wave
setpar('ridethewave' , 1)

#%% 
ser.write( b'o' + struct.pack('f',  0) ) # Restart controller

#%% Current plant FRF measurement PRBS
# setpar( 'Icontgain' , 1.54 ) # Current loop gain
# ser.write( b'9' + struct.pack('f',  500) ) # Current loop integrator frequency

# setpar( 'Icontgain' , 4 ) # Current loop gain
# ser.write( b'9' + struct.pack('f',  300) ) # Current loop integrator frequency

# setpar( 'Icontgain' , L * 2200 * 2 *pi  ) # Current loop gain

setpar('useIlowpass' , 0 , ser)

NdownsamplePRBS = 1
N = 30*NdownsamplePRBS*2047 + 1/Ts
signals = [ 'D', 'Q' , 'Id_meas' , 'Iq_meas', 'Id_meas2' , 'Iq_meas2' , 'dist' , 'emech1' , 'emech2' , 'Vq' , 'Vd', 'Vq2' , 'Vd2' ,'mechcontout']
setTrace( signals )


gain = 5
setpar('Vq_distgain' , 1)
setpar('Vd_distgain' , 1)
setpar('Iq_distgain' , 0)
setpar('Id_distgain' , 0)
setpar('mechdistgain' , 0)

setpar( 'NdownsamplePRBS' , NdownsamplePRBS) #Downsampling
setpar( 'distval' , gain) #disturbance amplitude
setpar( 'distoff' , 0) #disturbance offset
t,s = readData( int(N) )
setpar( 'distval' , 0) #disturbance amplitude
setpar( 'distoff' , 0) #disturbance offset
setpar( 'NdownsamplePRBS' , 1) #Downsampling

DIST,f = getFFT( s[:, signals.index('dist') ] , NdownsamplePRBS )
VQ,f = getFFT( s[:, signals.index('Q') ] , NdownsamplePRBS )
VD,f = getFFT( s[:, signals.index('D') ] , NdownsamplePRBS )
IQ,f = getFFT( s[:, signals.index('Iq_meas') ] , NdownsamplePRBS )
ID,f = getFFT( s[:, signals.index('Id_meas') ] , NdownsamplePRBS )
IQ2,f = getFFT( s[:, signals.index('Iq_meas2') ] , NdownsamplePRBS )
ID2,f = getFFT( s[:, signals.index('Id_meas2') ] , NdownsamplePRBS )
E1,f = getFFT( s[:, signals.index('emech1') ] , NdownsamplePRBS )
E2,f = getFFT( s[:, signals.index('emech2') ] , NdownsamplePRBS )
Vq,f = getFFT( s[:, signals.index('Vq') ] , NdownsamplePRBS )
Vd,f = getFFT( s[:, signals.index('Vd') ] , NdownsamplePRBS )

Vq2,f = getFFT( s[:, signals.index('Vq2') ] , NdownsamplePRBS )
Vd2,f = getFFT( s[:, signals.index('Vd2') ] , NdownsamplePRBS )
contout,f = getFFT( s[:, signals.index('mechcontout') ] , NdownsamplePRBS )

Sq = Vq / DIST
Sd = Vd / DIST
Sq2 = Vq2 / DIST
Sd2 = Vd2 / DIST

#CL = SENS / DIST
PQ = IQ / Vq
PD = ID / Vd
PQ2 = IQ2 / Vq2
PD2 = ID2 / Vd2

OLq = 1/Sq - 1
OLd = 1/Sd - 1
OLq2 = 1/Sq2 - 1
OLd2 = 1/Sd2 - 1

plt.figure(1)
bode( PQ , f , 'Measured Q axis plant')
bode( PD , f , 'Measured D axis plant')

Lq = 5e-6 
Ld = 5e-6 
R = 0.069
plantq = 1/ ( R + Lq * f * 2 * pi * 1j)
plantd = 1/ ( R + Ld * f * 2 * pi * 1j)
bode( plantq  , f , 'plant q')
bode( plantd  , f , 'plant d')


plt.figure(2)
plt.loglog( f , np.abs( 1/(PQ * f * 2 * np.pi)  ))
plt.grid(which='minor')

plt.figure(3)
bode( OLq / (1 + OLq) , f , 'CL Current loop (q axis)')
bode( OLd / (1 + OLd) , f , 'CL Current loop (d axis)')

import control as ct
R = 0.35
Ld = 2.0e-4
a = ct.c2d(ct.tf( 1 , [Ld , R ]) , Ts , 'zoh')  * ct.tf ([1] , [1, 0] , Ts )
y = ct.evalfr( a , x=exp(1j * 2*pi*f * Ts) )

plt.figure()
bode( PQ , f , 'Measured Q axis plant')
bode( y , f , 'Model: 1/(Ls+R), discretized')


#%%
plt.figure(1)
bode( OLq , f , 'OL Current loop (q axis)')
bode( OLd , f , 'OL Current loop (d axis)')

plt.figure(2)
bode( PQ , f , 'CL Current loop (q axis)')
bode( PD , f , 'CL Current loop (d axis)')






# Gevonden voor motor 1:
Lq = 2.6e-4 
Ld = 2.2e-4
R = 0.325
plantq = 1/ ( R + Lq * f * 2 * pi * 1j)
plantd = 1/ ( R + Ld * f * 2 * pi * 1j)
bode( plantq  , f , 'plant q')
bode( plantd  , f , 'plant d')

plt.figure(3)
nyquist( OLq , f )
nyquist( OLd , f )


plt.figure(4)
bode( OLq2 , f , 'OL Current loop (q axis)')
bode( OLd2 , f , 'OL Current loop (d axis)')


plt.figure(5)
bode( PQ2 , f , 'CL Current loop (q axis)')
bode( PD2 , f , 'CL Current loop (crosstalk to d axis)')




# Gevonden voor motor 2 (Maxon):
Lq = 0.0002143
Ld = 0.00023500
R = 0.55
plantq = 1/ ( R + Lq * f * 2 * pi * 1j)
plantd = 1/ ( R + Ld * f * 2 * pi * 1j)
bode( plantq  , f , 'plant q')
bode( plantd  , f , 'plant d')


plt.figure(5)
f2 = f[(f > 10000) & (f < 20000)]
bode( PQ2[(f > 10000) & (f < 20000)] * f2 *2 *pi  , f2  )
Lq2 = np.abs(1/(np.mean( PQ2[(f > 10000) & (f < 20000)] * f2 *2 *pi )))

bode( PD2[(f > 10000) & (f < 20000)] * f2 *2 *pi  , f2  )
Ld2 = np.abs(1/(np.mean( PD2[(f > 10000) & (f < 20000)] * f2 *2 *pi )))

plt.figure(4)
R2 = 0.48
plantq = 1/ ( R2 + Lq2 * f * 2 * pi * 1j)
plantd = 1/ ( R2 + Ld2 * f * 2 * pi * 1j)
bode( plantq  , f , 'plant q')
bode( plantd  , f , 'plant d')



#%%
#bode( P2 , f , 'Plant')
#
#plt.figure()
#bode( S , f , 'S')
#
#plt.figure()
#bode( 1/S-1 , f , 'OL')
#
Pmech = -E1 / contout
Pmech2 = -E2 / contout

plt.figure()
bode( Pmech , f , 'Plant')
bode( Pmech2 , f , 'Plant')

#%%
#Kp = 320
#fBW = 100
#alpha1 = 3
#alpha2 = 4
#fInt = fBW / 6
#fLP = fBW * 8

# Kp = 200;
# fBW = 400;
# alpha1 = 5;
# alpha2 = 5;
# fInt = fBW / 8;
# fLP = fBW * 8;

# Kp = 150;
# fBW = 400;
# alpha1 = 5;
# alpha2 = 5;
# fInt = fBW / 8;
# fLP = fBW * 8;

fBW = 80
alpha1 = 4
alpha2 = 3
fInt = fBW / 6
fLP = fBW * 6

num, den = leadlag( fBW , alpha1, alpha2, Ts)
CONT = freqrespd( num , den ,f , Ts )
num, den = lowpass2( fLP , 0.7 , Ts)
LP2 = freqrespd( num , den ,f , Ts )
num, den = Integrator( fInt , Ts)
INTEGRATOR = freqrespd( num , den ,f , Ts )

Kp = 1/np.abs(np.interp( fBW , f, CONT * LP2 * Pmech2 * (INTEGRATOR+1)))

plt.figure(1)
bode( CONT * LP2 * Pmech2 * Kp* (INTEGRATOR+1)  , f , 'OL reconstruct')
plt.figure(3)
nyquist( CONT * LP2 * Pmech2 * Kp* (INTEGRATOR+1)  , f , 'OL reconstruct')

#%% Current FRF measurement PRBS
Ndownsample = 1
N = 10*2047 + 1/Ts
setpar( 'Kp' , 0)
setpar('Icontgain' , 1.54) # Current loop gain
ser.write( b'9' + struct.pack('f',  700) ) # Current loop integrator frequency
ser.write( b'Z' + struct.pack('I',  1) ) # Send data 1
setpar( 'Ndownsample' , Ndownsample) #Downsampling
setpar( 'distval' , 1.5) #disturbance amplitude
setpar( 'distoff' , 2) #disturbance offset
n,t,t2,dist,e,Vout,sens,r,ss_f,enc2,emech,Ts = readData( int(N) )
setpar( 'distval' , 0) #disturbance amplitude
setpar( 'distoff' , 0) #disturbance offset
setpar( 'Ndownsample' , 1) #Downsampling
ser.write( b'C' + struct.pack('I',  1) ) # 50 Hz BW
ser.write( b'o' + struct.pack('f',  0) ) # Restart controller
makebodesOL( Ndownsample , 1)

#%% Overall FRF measurement PRBS
NdownsamplePRBS = 10
N = 10*NdownsamplePRBS*2047 + 1/Ts

setpar( 'offsetVel' , 5*np.pi)
setpar( 'NdownsamplePRBS' , NdownsamplePRBS)
setpar( 'distval' , 0.05) #disturbance amplitude
setpar('Vq_distgain' , 0)
setpar('Vd_distgain' , 0)
setpar('Iq_distgain' , 0)
setpar('Id_distgain' , 0)
setpar('mechdistgain' , 1)

signals = [ 'mechcontout' , 'D', 'Q' , 'Id_meas' , 'Iq_meas' , 'dist' , 'emech1' , 'emech2' , 'Vq' ,'ymech1' , 'mechcontout2' ]
setTrace( signals )
t,s = readData( int(N) )
setpar( 'offsetVel' , 0.0*np.pi)
setpar( 'distval' , 0) #disturbance amplitude
setpar( 'NdownsamplePRBS' , 1) #Downsampling


E1,f = getFFT( s[:,signals.index('emech1')] , NdownsamplePRBS )
E2,f = getFFT( s[:,signals.index('emech2')] , NdownsamplePRBS )

DIST,f = getFFT( s[:,5] , NdownsamplePRBS )
TORQUE,f = getFFT( s[:,0] , NdownsamplePRBS  )
TORQUE2,f = getFFT( s[:,10] , NdownsamplePRBS  )

Pmech  = -E1/TORQUE 
Pmech2 = -E2/TORQUE2 


plt.figure(1)
bode( DIST/TORQUE - 1 , f)
# bode( DIST/TORQUE2 - 1 , f)


plt.figure(2)
bode( Pmech , f)
bode( Pmech2, f)

plt.figure(3)
nyquist( DIST/TORQUE - 1 , f , 'OL')
nyquist( DIST/TORQUE2 - 1 , f , 'OL')

plt.figure()
bode( DIST/TORQUE2 - 1 , f)

plt.figure()
bode( Pmech2 , f)

plt.figure()
nyquist( DIST/TORQUE2 - 1 , f , 'OL')

res()


#%%
ser.write( b'o' + struct.pack('f',  0) ) # Restart controller
#%%
ser.write( b'C' + struct.pack('I',  1) ) # 100 Hz BW
#%%
ser.write( b'C' + struct.pack('I',  2) ) # 30 Hz BW
#%%
ser.write( b'C' + struct.pack('I',  3) ) # 5 Hz BW
#%%
ser.write( b'C' + struct.pack('I',  4) ) # 300 Hz BW

#%% Single sine Current loop
ss_fstart = 10;
ss_fstep = 2;
ss_fend = 9999;
ss_n_aver = 10;
ss_distgain = 0.5;
ss_distoffset = 0;

t_tot = (sum(ss_n_aver / np.arange( ss_fstart , ss_fend, ss_fstep)) + 0.1)
print( t_tot )


N = int( t_tot / Ts)
setpar( 'Kp' , 0) # Kp = 0
ser.write( b'Z' + struct.pack('I',  1) ) # Send data 1
setpar( 'ss_fstart' , ss_fstart )   
setpar( 'ss_fend' , ss_fend)
setpar( 'ss_fstep' , ss_fstep)
setpar( 'ss_n_aver' , ss_n_aver)
setpar( 'distoff' , ss_distoffset)

time.sleep(0.5)
ser.write( b's' + struct.pack('f', ss_distgain ) )
n,t,t2,dist,e,Vout,sens,r,ss_f,enc2,emech,Ts = readData(N)
ser.write( b's' + struct.pack('f',  0) )
setpar( 'distoff' , 0) #disturbance offset
ser.write( b'C' + struct.pack('I',  1) ) # 50 Hz BW
ser.write( b'o' + struct.pack('f',  0) ) # Restart controller
freqs = np.unique( ss_f[ss_f>0]  )
CL = np.zeros( len(freqs) ,dtype=np.complex_)
P = np.zeros( len(freqs) ,dtype=np.complex_)
S = np.zeros( len(freqs) ,dtype=np.complex_)
Kt = 0.068 # Nm/A
for freq in freqs:
    I = ss_f == freq
    DIST , f = getFFT_SS( dist[I] )
    R , f = getFFT_SS( r[I] )
    SENS , f = getFFT_SS( sens[I] )
    VOUT , f = getFFT_SS( Vout[I] )
    E , f = getFFT_SS( e[I] )
    index = np.argmin( abs(freq-f ))
    CL[freq==freqs] = SENS[index] / DIST[index]
    P[freq==freqs] = SENS[index] / VOUT[index]  
    S[freq==freqs] = E[index] / DIST[index]  

plt.figure(1)
bode( CL , freqs , 'CL')
plt.figure(2)
bode( P , freqs , 'P')
plt.figure(3)
bode( 1/S-1 , freqs , 'OL')

#%% Single sine
ser = start( ser ) 
ss_fstart = 100;
ss_fstep = 5;
ss_fend = 110;
ss_n_aver = 100;
ss_distgain = 2;
ss_distoffset = 0;

#ss_fstart = 1;
#ss_fstep = 1;
#ss_fend = 10;
#ss_n_aver = 10;
#ss_distgain = 10;
t_tot = (sum(ss_n_aver / np.arange( ss_fstart , ss_fend, ss_fstep)) + 0.1)
print( t_tot )

setpar('Vq_distgain' , 0)
setpar('Vd_distgain' , 0)
setpar('Iq_distgain' , 0)
setpar('Id_distgain' , 0)
setpar('mechdistgain' , 0.1)

N = int( t_tot / Ts)
setpar( 'offsetVel' , 2*np.pi)
setpar( 'ss_fstart' , ss_fstart )   
setpar( 'ss_fend' , ss_fend)
setpar( 'ss_fstep' , ss_fstep)
setpar( 'ss_n_aver' , ss_n_aver)
setpar( 'distoff' , ss_distoffset)
time.sleep(0.5)
ser.write( b's' + struct.pack('f', ss_distgain ) )

signals = [ 'ss_f' , 'dist' , 'mechcontout' ,'emech' , 'emech2' ]
setTrace( signals )
t,s = readData( int(N) )

ser.write( b's' + struct.pack('f',  0) )
setpar( 'offsetVel' , 0.0*np.pi)
#ser.write( b'o' + struct.pack('f',  0) ) # Restart controller
ss_f = s[:, signals.index('ss_f')]
freqs = np.unique( ss_f[ss_f>0]  )
S = np.zeros( len(freqs) ,dtype=np.complex_)
PS = np.zeros( len(freqs) ,dtype=np.complex_)
PS2 = np.zeros( len(freqs) ,dtype=np.complex_)
#Pelec = np.zeros( len(freqs) ,dtype=np.complex_)
for freq in freqs:
    I = ss_f == freq
    DIST , f = getFFT_SS( s[I, signals.index('dist')] )
    mechcontout , f = getFFT_SS( s[I, signals.index('mechcontout')] )
    EMECH , f = getFFT_SS( s[I, signals.index('emech')] )
    EMECH2 , f = getFFT_SS( s[I, signals.index('emech2')] )

    index = np.argmin( abs(freq-f ))  

#    SENS , f = getFFT_SS( sens[I] )
#    VOUT , f = getFFT_SS( Vout[I] )

#    Pelec[freq==freqs] = SENS[index] / VOUT[index]  
    S[freq==freqs] = mechcontout[index] / DIST[index]
    PS[freq==freqs] = -EMECH[index] / DIST[index]  
    PS2[freq==freqs] = -EMECH2[index] / DIST[index]     


#freqs, results = goertzel(some_samples, 44100, (400, 500), (1000, 1100))

plt.figure(1)
bode( PS/S , freqs , 'P')
# bode( PS2/S , freqs , 'P2')

plt.figure(2)
bode( 1/S-1 , freqs , 'OL')
plt.figure(3)
nyquist( 1/S-1 , freqs , 'OL')

def normalize(v):
    norm = max(abs(v))
    if norm == 0: 
       return v
    return v / norm

freq = ss_f[ np.argmin( abs(1000-ss_f ) ) ]
plt.figure();
plt.plot( normalize( s[ ss_f == freq , signals.index('dist')] ))
plt.plot( normalize( s[ ss_f == freq , signals.index('mechcontout')] ))
plt.plot(normalize( s[ ss_f == freq , signals.index('emech')] ))
plt.show();

#%%
signals = [ 'rmech', 'ymech' , 'emech' , 'Iq_SP' , 'Iq_meas', 'Id_meas' , 'acc' , 'Vq' , 'firsterror']
setTrace( signals )

ser.write( b'o' + struct.pack('f',  0) ) # Restart controller

N = int( 2/Ts ) 
t,s = readData( int(N) )

plt.figure();
plt.plot(t,s[:,0:len(signals)], );
plt.legend( signals )
plt.show()

#%%

Kt = 0.068
P = PS / S / Kt
f = freqs

Kp = 60
fBW = 200
alpha1 = 3
alpha2 = 4.5
fInt = fBW / 6
fLP = fBW * 8

#Kp = 37
#fBW = 150
#alpha1 = 3
#alpha2 = 4.5
#fInt = fBW / 6
#fLP = fBW * 8

num, den = leadlag( fBW , alpha1, alpha2, Ts)
CONT = freqrespd( num , den ,f , Ts )
num, den = lowpass2( fLP , 0.7 , Ts)
LP2 = freqrespd( num , den ,f , Ts )
num, den = Integrator( fInt , Ts)
INTEGRATOR = freqrespd( num , den ,f , Ts )

plt.figure()
bode(  1/S-1 , f , 'OL')
bode( CONT * LP2 * P * Kp* (INTEGRATOR+1)  , f , 'OL tuned')

plt.figure()
nyquist(  1/S-1 , f , 'OL')
nyquist( CONT * LP2 * P * Kp* (INTEGRATOR+1)  , f , 'OL tuned')

#%%
ser.write( b'S' + struct.pack('I',  0) + struct.pack('f',  9.99)  )

getsig('commutationoffset')


#%%
plt.close('all')
#%%
np.savez( 'testSS' ,  n=n ,t=t,t2=t2,dist=dist,e=e,Vout=Vout,sens=sens,r=r,ss_f=ss_f,enc2=enc2,emech=emech,Ts=Ts )
npzfile = np.load('testNoLP.npz')
npzfile = np.load('test4HzLP.npz')
npzfile.files

n = npzfile['n']
t = npzfile['t']
t2 = npzfile['t2']
dist = npzfile['dist']
e = npzfile['e']
Vout = npzfile['Vout']
sens = npzfile['sens']
r = npzfile['r']
ss_f = npzfile['ss_f']
enc2 = npzfile['enc2']
emech = npzfile['emech']
Ts = npzfile['Ts']
    

#%%
setpar('muziek_gain_V' , 5)
#%%
setpar('muziek_gain_V' , 0)

#%% Music on
setpar( 'offsetVel' , 2*np.pi)
setpar( 'muziek_gain' , 4) # music current
#%% Music off
setpar( 'offsetVel' , 0.0*np.pi)
setpar( 'muziek_gain' , 0) # music current

#%%
ser.write( b'o' + struct.pack('f',  0) ) # Restart controller
#%%
ser.write( b'C' + struct.pack('I',  1) ) # 100 Hz BW
#%%
ser.write( b'C' + struct.pack('I',  2) ) # 30 Hz BW
#%%
ser.write( b'C' + struct.pack('I',  3) ) # 5 Hz BW
#%%
ser.write( b'C' + struct.pack('I',  4) ) # 300 Hz BW
#%%
#ser.write( b'C' + struct.pack('I',  5) ) # 400 Hz BW
#%%
ser = start( ser )
#%%
plt.close('all')
#%%
# Jload = 0.0008
# vFF = 0.002

Jload = 6.77e-5 #kgm²
vFF = 7e-5  #Nm/(rad/s)


setpar( 'Jload' ,  Jload )
setpar( 'velFF' , vFF )
setpar( 'rdelay' , 0)
#getsig( 'Jload' )
#getsig( 'velFF' )

# 12.5 ms
# p = 0.1*np.pi
# v = 150*np.pi
# a = 3000*np.pi
# j = 4000000*np.pi
# Ts = 50e-6

# 9 ms
#p = 0.05*np.pi
#v = 150*np.pi
#a = 3000*np.pi
#j = 4000000*np.pi
#Ts = 50e-6

# p = 2*np.pi
# v = 150*np.pi
# a = 1100*np.pi
# j = 500000*np.pi
#Ts = 50e-6

p = 10
v = 30*np.pi
a = 300*np.pi
j = 40000*np.pi

p = np.pi/2
v = 120
a = 10000
j = 1.5e7
Nsp = 10


p = 2*2*pi
v = 25*2*pi
a = 1000*2*pi
j = 1000000*2*pi
Nsp = 5


p = 20*2*pi
v = 65*2*pi
a = 500*2*pi
j = 500000*2*pi
Nsp = 1

# p = 50
# v = 150
# a = 11000
# j = 1.5e7
# Nsp = 10

# p = pi/2*10
# v = 25*2*pi
# a = 2500*2*pi
# j = 400000*2*pi
# Nsp = 10

# p = pi/2
# v = 20*2*pi
# a = 1750*2*pi
# j = 2400000*2*pi
# Nsp = 10

[t1, t2, t3, jd] = prepSP(  p , v , a , j )


# signals = [ 'rmech', 'ymech1' , 'emech1' , 'vel' , 'Iq_meas', 'Id_meas' , 'acc' , 'Va', 'Vb', 'Vc' , 'mechcontout', 'sensBus']
signals = [ 'rmech', 'ymech1' , 'emech1' , 'vel' , 'Iq_meas2', 'Id_meas2' , 'acc' , 'Va2', 'Vb2', 'Vc2' , 'mechcontout', 'sensBus']
setTrace( signals )

setpar('SPdir' , 1)
setpar('spNgo',Nsp)
# N = int( (Nsp*(0.01+4*t1+2*t2+t3)+0.1) / Ts ) 
# t,s = readData( int(N) )

df = trace( (Nsp*(0.01+4*t1+2*t2+t3)+0.1) )

# df.plot()


plt.figure()
df.filter(regex='.*meas').plot()


plt.figure()
df.filter(regex='V').plot()
(df.vel/2/pi).plot()

#%%

# plt.figure()
# df.mechcontout.plot()


plt.figure()
df.mechcontout.plot()
(df.acc * Jload).plot()
(df.vel * vFF).plot()


plt.figure()
df.mechcontout.plot()
df.Iq_meas.plot()
(df.ymech/10).plot()

plt.figure()
(df.acc / 1e6 ).plot()
df.emech.plot()

# plt.figure()
# ((df.rmech.diff())/Ts).plot()
# df.vel.plot()

plt.figure()
df.rmech.plot()
df.ymech.plot()


plt.figure()
(df.filter(regex='emech')/2/pi*20000).plot()

plt.figure()
df.sensBus.plot()

#%%
plt.figure()
line1, = plt.plot( t[0:-2]+Ts , np.diff(np.diff(rmech)) /Ts**2, label='ref')
line1, = plt.plot( t , acc  , label='acc')
line1, = plt.plot( t , emech *1000000 , label='error')
plt.grid( 1 , 'both')
plt.legend()
plt.show()

#%%
getsig('EncSelect_FB')

#%%
setpar('EncSelect_FB' , 1)
ser.write( b'o' + struct.pack('f',  0) ) # Restart controller

#%%
setpar('EncSelect_FB' , 2)
ser.write( b'C' + struct.pack('I',  2) ) # 30 Hz BW
ser.write( b'o' + struct.pack('f',  0) ) # Restart controller


#%%

ser.write( b'o' + struct.pack('f',  0) ) # Restart controller

#%% Setpoint bug at end: turned out to be due to numerical precision. Upgrades SP generator calculation to doubles.
setpar( 'commutationoffset',0.75) #Motor 1
setpar( 'commutationoffset2',3.05) #Motor 2

setpar('useIlowpass' , 1 , ser)

p = 10*2*pi
v = 50*2*pi
a = 300*2*pi
j = 500000*2*pi
Nsp = 1


setpar('Id_offset_SP', 0)

[t1, t2, t3, jd] = prepSP(  p , v , a , j )

setpar('SPdir' , 1)
setpar('spNgo' , Nsp)
# setpar( 'offsetVel' , 20*np.pi)



# signals = [ 'Va' ,'Vb' , 'Vc' , 'emech1' , 'Iq_meas' , 'Id_meas' , 'rmech' ,'ymech1' , 'Vd' , 'Vq' , 'VdFF' , 'VqFF' ]
signals = [ 'Va2' ,'Vb2' , 'Vc2' , 'emech2' , 'Iq_meas2' , 'Id_meas2' , 'rmech2' ,'ymech2' , 'Vd2' , 'Vq2' , 'VdFF2' , 'VqFF2']
setTrace( signals )

df = trace( 2 )

# df.plot()

plt.figure()
df.filter(regex='V[abc]').plot()



df.filter(regex='V[dq]').plot()


df.filter(regex='I').plot()


# (df.filter(regex='emech')/2/pi*20000).plot()



# plt.figure()
# df.rmech.plot()
# df.ymech1.plot()


# plt.figure()
# df.REFstatus.plot()

setpar('Id_offset_SP', 0)


#%%
signals = [ 'Iq_SP' ,'Iq_meas' , 'Id_SP' ,'Id_meas'  ]

setTrace( signals )

df = trace( 2 )

# df.plot()

df.filter(regex='.*_meas').plot()

#%%
signals = [ 'Va' ,'Vb' , 'Vc' , 'tA', 'tB' , 'tC','Valpha' , 'Vbeta' ,'Iq_SP' ,'Iq_meas' , 'Id_SP' ,'Id_meas' , 'rmech' , 'emech' , 'ymech' , 'encoderPos1' ,'Nsend' , 'overloadcount' ,'sens1' , 'sens2' , 'ContSelect' , 'OutputOn' , 'firsterror'  , 'ia' , 'ib' , 'ic' , 'thetaPark' , 'Ialpha' , 'Ibeta' ,'dist']

setTrace( signals )

df = trace( 2 )

df.filter(regex='i').plot()

#%% ADC shift tuning. -470/2 is good for 100 kHz.

adcshift = -470/2
# adcshift = 0
ser.write( b'8' + struct.pack('i',  int(adcshift)) )



#%%
df = readall()

df.Kp

app = QtGui.QApplication([])

win2 = pg.GraphicsLayoutWidget()

layout2 = QtGui.QGridLayout()

increasebutton = QtGui.QPushButton("Restart controller")

setcont = QtGui.QPushButton("Set controller 1")
setcont2 = QtGui.QPushButton("Set controller 2")
Kp = QtGui.QLineEdit(str( df.Kp[0] ))
fBW = QtGui.QLineEdit(str( df.fBW[0] ))
alpha1 = QtGui.QLineEdit(str( df.alpha1[0] ))
alpha2 = QtGui.QLineEdit(str( df.alpha2[0] ))
fInt = QtGui.QLineEdit(str( df.fInt[0] ))
fLP = QtGui.QLineEdit(str( df.fLP[0] ))
Kp2 = QtGui.QLineEdit(str( df.Kp2[0] ))
fBW2 = QtGui.QLineEdit(str( df.fBW2[0] ))
alpha1_2 = QtGui.QLineEdit(str( df.alpha1_2[0] ))
alpha2_2 = QtGui.QLineEdit(str( df.alpha2_2[0] ))
fInt2 = QtGui.QLineEdit(str( df.fInt2[0] ))
fLP2 = QtGui.QLineEdit(str( df.fLP2[0] ))


layout2.addWidget( setcont ,1,0)
layout2.addWidget( setcont2 ,1,1)
layout2.addWidget(Kp, 2, 0)
layout2.addWidget(fBW, 3, 0)
layout2.addWidget( alpha1 , 4,0)
layout2.addWidget( alpha2 , 5,0)
layout2.addWidget( fInt , 6,0)
layout2.addWidget( fLP , 7,0)

layout2.addWidget(Kp2, 2, 1)
layout2.addWidget(fBW2, 3, 1)
layout2.addWidget( alpha1_2 , 4,1)
layout2.addWidget( alpha2_2 , 5,1)
layout2.addWidget( fInt2 , 6,1)
layout2.addWidget( fLP2 , 7,1)

def setcont_clicked():
    setpar( 'Kp' , float(Kp.text()))
    setpar( 'fBW' , float(fBW.text()))
    setpar( 'alpha1' , float(alpha1.text()))
    setpar( 'alpha2' , float(alpha2.text()))
    setpar( 'fInt' , float(fInt.text()))
    setpar( 'fLP' , float(fLP.text()))
    ser.write( b'C' + struct.pack('I',  0) ) 
    ser.write( b'o' + struct.pack('f',  0) ) # Restart controller


def setcont2_clicked():
    setpar( 'Kp2' , float(Kp2.text()))
    setpar( 'fBW2' , float(fBW2.text()))
    setpar( 'alpha1_2' , float(alpha1_2.text()))
    setpar( 'alpha2_2' , float(alpha2_2.text()))
    setpar( 'fInt2' , float(fInt2.text()))
    setpar( 'fLP2' , float(fLP2.text()))
    ser.write( b'C' + struct.pack('I',  0) ) 
    ser.write( b'o' + struct.pack('f',  0) ) # Restart controller


setcont.clicked.connect(setcont_clicked)
setcont2.clicked.connect(setcont2_clicked)

win2.resize( 300, 1)
win2.setLayout(layout2)
win2.show()

#%%

# p = pi/2
# v = 20*2*pi
# a = 1750*2*pi
# j = 2400000*2*pi

# p = 100*2*pi
# v = 40*2*pi
# a = 200*2*pi
# j = 400000*2*pi

p = 2*2*pi
v = 30*2*pi
a = 1000*2*pi
j = 1000000*2*pi

[t1, t2, t3, jd] = prepSP(  p , v , a , j )


signals = [ 'ymech', 'Iq_meas', 'Id_meas']
signals = [ 'ymech', 'Iq_meas', 'firsterror']
signals = [ 'ymech', 'rmech', 'acc' ,'Iq_meas', 'Id_meas']
signals = [ 'ymech', 'rmech', 'sensBus' ,'Iq_meas', 'Id_meas']
# signals = [ 'ymech1', 'rmech', 'sensBus' ,'Iq_meas', 'Id_meas']
signals = [ 'ymech1', 'rmech', 'sensBus' ,'Iq_meas', 'Id_meas']
# signals = [ 'ymech1', 'ymech2', 'sensBus' ,'Iq_meas2', 'Id_meas2']
# signals = [ 'ymech2', 'rmech2', 'sensBus' ,'Vq2', 'Vd2']


# signals = [ 'emech1', 'emech2', 'sensBus' ,'Iq_meas', 'Iq_meas2']
# signals = [ 'sens1', 'sens2', 'sensBus' ,'Iq_meas', 'Iq_meas2']
# signals = [ 'ymech2', 'rmech', 'sensBus' ,'Iq_meas2', 'Id_meas2']
# signals = [ 'emech1', 'emech2', 'sensBus' ,'Iq_meas2', 'Id_meas2']
# signals = [ 'encoderPos1', 'encoderPos2', 'acc' ,'IndexFound1', 'IndexFound1']
# signals = [ 'tA', 'tB', 'tC']
setTrace( signals )




y1 = deque()
y1a = deque()
y2 = deque()
y3 = deque()
y4 = deque()
app = QtGui.QApplication([])
# win = pg.GraphicsWindow()


win = pg.GraphicsLayoutWidget()

layout = QtGui.QGridLayout()


increasebutton = QtGui.QPushButton("Restart controller")

setcont = QtGui.QPushButton("Set controller")
decreasebutton = QtGui.QPushButton("Start Setpoint")
decreasebutton2 = QtGui.QPushButton("Start Setpoint2")
setspbutton = QtGui.QPushButton("Set Setpoint")
setpointdist1 = QtGui.QLineEdit(str(p/2/pi))
setpointdist2 = QtGui.QLineEdit(str(v/2/pi))
setpointdist3 = QtGui.QLineEdit(str(a/2/pi))
setpointdist4 = QtGui.QLineEdit(str(j/2/pi))
nsetpoint = QtGui.QSlider(1)
nsetpoint.setRange(1, 100)

haptic = QtGui.QPushButton("Switch haptic mode")

cont_setting = QtGui.QSlider(1)
cont_setting.setRange(1, 4)
cont_setting.setPageStep(1)


layout.addWidget(increasebutton, 0,0)
layout.addWidget( cont_setting ,1,0)
layout.addWidget(setcont , 2,0)
layout.addWidget( nsetpoint, 3,0)
layout.addWidget(decreasebutton, 4, 0)
layout.addWidget(decreasebutton2, 5, 0)
layout.addWidget(setspbutton, 6, 0)
layout.addWidget( setpointdist1 , 7,0)
layout.addWidget( setpointdist2 , 8,0)
layout.addWidget( setpointdist3 , 9,0)
layout.addWidget( setpointdist4 , 10,0)
layout.addWidget( haptic , 11,0)

hapticon = 0

def on_increasebutton_clicked():
    ser.write( b'o' + struct.pack('f',  0) ) # Restart controller

def on_decreasebutton_clicked():
    setpar('SPdir' , 1)
    setpar('spNgo' , int( nsetpoint.value() ))

def on_decreasebutton_clicked2():
    setpar('SPdir' , 0)
    setpar('spNgo' , int( nsetpoint.value() ))
    
def setcont_clicked():
    ser.write( b'C' + struct.pack('I',  int( cont_setting.value() ) )    )  
    
def on_setsp_clicked():
    p = float(setpointdist1.text())*2*pi
    v = float(setpointdist2.text())*2*pi
    a = float(setpointdist3.text())*2*pi
    j = float(setpointdist4.text())*2*pi
    [t1, t2, t3, jd] = prepSP(  p , v , a , j )

def haptic_clicked():
    global hapticon
    if hapticon ==0:
        hapticon = 1
    else:        
        hapticon = 0
    setpar('haptic' , hapticon)
    ser.write( b'o' + struct.pack('f',  0) ) # Restart controller


increasebutton.clicked.connect(on_increasebutton_clicked)
decreasebutton.clicked.connect(on_decreasebutton_clicked)
decreasebutton2.clicked.connect(on_decreasebutton_clicked2)
setspbutton.clicked.connect(on_setsp_clicked)
setcont.clicked.connect(setcont_clicked)
haptic.clicked.connect(haptic_clicked)


plot1 = pg.plot()
plot2 = pg.plot()
plot3 = pg.plot()


layout.addWidget(plot1, 0, 1)
layout.addWidget(plot2, 1, 1)
layout.addWidget(plot3, 2, 1)

   
curve1 = plot1.plot()
curve1a = plot1.plot(pen=(1,3))
curve2 = plot2.plot()
curve3 = plot3.plot()
curve4 = plot3.plot(pen=(1,3))

T = deque()

win.resize( 1000, 700)
win.setLayout(layout)
win.show()

setpar('Ndownsample' , int( 0.002/Ts ))
setpar('Nsend' , int(1e6))


Nget=5
Nsig = 2+n_trace
while not win.isHidden():
    while ser.in_waiting < Nget*(2+n_trace)*4:
        dummy = 0
        # print( ser.in_waiting ) 
    buffer = ser.read( Nget*(2+n_trace)*4)
    N = len(buffer) / ((2+n_trace)*4)
    n = np.zeros( int(N) , dtype=np.uint32 )
    t = np.zeros( int(N) ) 
    s = np.zeros( ( int(N) , Nsig-2 ))
    sigtypes = ['I' , 'I' ] + ser.tracesigtypes
    for sigtype in set(sigtypes):
        indices = [i for i in range(len(sigtypes)) if sigtypes[i] == sigtype]
        indices = np.array(indices)
        if sigtype == 'b':
            tmp = np.ndarray( ( int(N) , Nsig ) , 'bool', buffer[0::4] )
        else:
            tmp = np.ndarray( ( int(N) , Nsig ) , sigtype, buffer )
        if sigtype == 'I':
            n = tmp[:,indices[0]]
            t = tmp[:,indices[1]] / 1e6
            s[:,indices[2:]-2] = tmp[:,indices[2:]]
        else:
            s[:,indices-2] = tmp[:,indices]

    T.extend( t[:] )
    y1.extend( s[:,0]); 
    y1a.extend( s[:,1]); 
    y2.extend( s[:,2] ); 
    y3.extend( s[:,3] ) ; 
    y4.extend( s[:,4] ) ; 
    
    while len(y1) > 2000:
        y1.popleft() #remove oldest
        y1a.popleft() #remove oldest
        y2.popleft() #remove oldest
        y3.popleft()
        y4.popleft()
        T.popleft()
    curve1.setData( x=T , y=y1)
    curve1a.setData( x=T , y=y1a)
    curve2.setData( x=T , y=y2)
    curve3.setData( x=T , y=y3)
    curve4.setData( x=T , y=y4)
    app.processEvents()  

setpar('Nsend' , 0)
bla = ser.readall()
setpar('Ndownsample' , int( 1 ))



