#%%
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 29 19:25:07 2018

@author: Elwin
"""

from os import chdir
chdir('c:\\users\\elwin\\desktop\\python')


import serial
import numpy as np
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
import time
import random
import pyqtgraph as pg
from collections import deque
from pyqtgraph.Qt import QtGui, QtCore

plt.rcParams["figure.dpi"] = 200

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
# n_trace = 62;  #Too high, current plant prbs measurement shows it is not always in time

# com = 'COM10' 1
com = 'COM3'

motor = 'motor1'


#for i in tqdm(range(10)):
#    time.sleep(0.1)

global ser

try:
    ser = serial.Serial( com  , timeout= 0.1)  # open serial port
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

def tracesignal( signals, t ,plot=True):
    setTrace( signals )
    t,s = readData( int(t /Ts) )
    if plot:
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
  

def readData(N , ser=ser):
    Nsig = 2+n_trace
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
            # print('data seems OK')
            pass
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
    df.index.name = 'Time [s]'
    return df

def getFFT( signal , Ndownsample , j0 = int(1/Ts)  ):
#    j0 = int(1/Ts) # These amount of samples are ignored
    L = 2047 * Ndownsample
    Naver = int( (len(signal)-j0)/L  )
    # print( 'Naver =' , Naver )
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

def start( ser=ser ):
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
def CL( f_bw = 1e3 , ser=ser):
    # ser = start( ser )
    # if (getsig( 'IndexFound1' , ser )+ getsig( 'IndexFound2' ,ser) ) < 2:
    #     setpar('ridethewave' , 1 , ser)
    #     time.sleep( 1 )
    # if getsig( 'IndexFound1' ,ser ) < 1:
    #     print('index not found')
    # else:  
  
    # Ld = 190e-6  
    # Lq = 245e-6
    # R = 0.399
    # setpar('Lambda_m' , 0.01213)
    # setpar('N_pp' ,  4)

    # Ld = 5.1e-6 
    # Lq = 5.3e-6 
    # R = 0.069      
    # setpar('Lambda_m' , 60 / (sqrt(3) * pi * 2200 * 2 * 7) )
    # setpar('N_pp' ,  7)
    
    # Ld = 530e-6 
    # Lq = 630e-6 
    # R = 0.264  
    # setpar('Lambda_m' , 0.0215 )
    
    # Roto max 100cc
    # Ld = 6e-6  
    # Lq = 10e-6
    # R = 0.055
    # setpar('Lambda_m' , 0.0042 )
    # setpar('N_pp' ,  10)
    
    # TimB Pancake motor
    # Ld = 24e-6  
    # Lq = 27e-6
    # R = 0.07
    # setpar('Lambda_m' , 0.0027 )
    # setpar('N_pp' ,  12) $Not checked!

    #Trampa 160KV
    # Ld = 33e-6 
    # Lq = 53e-6  
    # R = 0.0735    
    Ld = 19e-6 #New board 
    Lq = 34e-6  
    R = 0.035
    setpar('Lambda_m' , 0.005405 )
    setpar('N_pp' ,  7)

    setpar( 'Lq' , Lq , ser)
    setpar( 'Ld' , Ld , ser)
    setpar( 'R' , R , ser)
    setpar('useIlowpass' , 0 , ser)
    
    Jload = 6.77e-5 #kgm²
    vFF = 7e-5  #Nm/(rad/s)
    setpar( 'Jload' ,  Jload  , ser)
    # setpar( 'velFF' ,  vFF  , ser)
    setpar( 'rmechoffset' , 0 , ser)
    setpar( 'rmechoffset' , -getsig( 'emech1'  , ser) , ser)
    getsig('emech1'  , ser)
    
    # setpar( 'Icontgain' , 1.54 ) # Current loop gain
    # ser.write( b'9' + struct.pack('f',  500) ) # Current loop integrator frequency

    setpar( 'Kp_iq' , Lq * f_bw * 2 *pi  ) # Current loop Kp
    setpar( 'Ki_iq' , R/Lq  ) # Current loop Ki
    setpar( 'Kp_id' , Ld * f_bw * 2 *pi  ) # Current loop Kp
    setpar( 'Ki_id' , R/Ld  ) # Current loop Ki
    
    # ser.write( b'C' + struct.pack('I',  1) ) # 100 Hz BW

    # setpar('encoderPos2offset' , 0 , ser)
    # setpar('encoderPos2offset' , int(getsig('encoderPos1') -getsig('encoderPos2') ))
    
#        ser.write( b'C' + struct.pack('I',  2) ) # Low BW
    #ser.write( b'C' + struct.pack('I',  3) ) # Very low BW
#        ser.write( b'C' + struct.pack('I',  4) ) # 300 Hz BW
  

def OL( ):
    setpar( 'Kp_iq' , 0  ) # Current loop Kp
    setpar( 'Ki_iq' , 0  ) # Current loop Ki
    setpar( 'Kp_id' , 0  ) # Current loop Kp
    setpar( 'Ki_id' , 0  ) # Current loop Ki

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


setpar('ridethewave' , 1 )
time.sleep( 1 )

setpar( 'commutationoffset',0) 


setpar('Valpha_offset' , 0.3)
time.sleep( 0.5 )
Valpha1 = getsig('Valpha_offset')
Ialpha1 = getsig('Ialpha')
Ibeta1 = getsig('Ibeta')
Ia1 = getsig('ia')
bus1 = getsig('sensBus')

setpar('Valpha_offset' , 0.7) 

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

CL()

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
setpar( 'Iq_offset_SP' , 12)
#%%
setpar( 'Iq_offset_SP' , 0)
#%% Detect Lambda_m
setpar('Lambda_m' , 1 )

vmax = 400 #ERPM

setpar('anglechoice' ,99 )
setpar('Id_offset_SP' , 10)
setpar('i_vector_acc' , 10)
setpar('i_vector_radpers' , vmax / 60 *2*pi)

while abs(getsig( 'i_vector_radpers_act')) < 0.99 * vmax / 60 *2*pi:
    a=1

setpar('Id_offset_SP' , 0)


data = tracesignal(['BEMFa' , 'BEMFb' ,'sensBus' , 'Iq_meas' , 'Id_meas' , 'Id_e' ,'Iq_e'] , 1.5)
lambda_m = np.mean((np.max(data[1][:,0:2] , axis=0) - np.min(data[1][:,0:2] , axis=0))/2)

setpar('i_vector_radpers' , 0)
setpar('i_vector_acc' , 1e8)

#%%
setpar('Lambda_m' , 1 )
data = tracesignal(['BEMFa' , 'BEMFb'] , 2)

lambda_m = (np.max(data[1] , axis=0) - np.min(data[1] , axis=0))/2

setpar('Lambda_m' , 0.0027 )
data = tracesignal(['BEMFa' , 'BEMFb'] , 2)

#%%
ser = start()
data = tracesignal(['sens1' , 'sens2'] , 0.3 , plot=False)
for i in range(len(data[1][0,:])):
    fftpsd(data[1][:,i])


#%%
a = plt.figure()
#%%

setpar('I_max' , 20)
setpar('maxDutyCycle' , 0.99)

# setpar('anglechoice' ,1 )
# setpar('advancefactor' , 0)

setpar('anglechoice' ,0 )
setpar('advancefactor' , 1)

# signals = [ 'Q' , 'Id_meas' , 'Iq_meas' , 'Va' , 'Vb', 'Vc' , 'Iq_SP'   , 'thetaPark', 'thetaPark_enc' , 'thetaPark_obs'   ,'BEMFa' , 'BEMFb']
signals = [ 'Iq_offset_SP'  , 'Iq_meas' , 'Id_meas'  , 'Vq' , 'Vd' , 'Va' , 'Vb', 'Vc' , 'sensBus', 'thetaPark',  'thetaPark_obs' , 'thetaPark_enc']
# signals = [ 'maxVolt'  , 'Vtot', 'Iq_meas' , 'Id_meas'  , 'Vq' , 'Vd' , 'Va' , 'Vb', 'Vc' , 'sensBus', 'thetaPark',  'thetaPark_obs' , 'erpm']

setTrace( signals )

setpar( 'Iq_offset_SP' , 3)
# time.sleep(0.1)
# setpar( 'Iq_offset_SP' , 7)

df = trace( 5 ) 
setpar( 'Iq_offset_SP' , 0)
df2 = trace( 1 ) 

# df.plot()

df.filter(regex='thetaPark').plot()

# df.filter(regex='^I').plot( )
# df.filter(regex='^V[dq]').plot( )

# ax = plt.figure()
# df.filter(regex='^erpm').plot( )


# df.filter(regex='thetaPark_obs').plot()


# df.filter(regex='^V[abc]').plot()

df2.filter(regex='sensB').plot()


#%% 
ser = start( ser )
# setpar('ridethewave' , 1 )
setpar( 'commutationoffset', -0.7376459240913391)
CL()

#%% HFI

# setpar('maxDutyCycle' , 0.7)
# setpar('I_max' , 10)

setpar('anglechoice' ,3)
# setpar('anglechoice' ,100)

Ki = 500*2*pi
hfi_v = 3

setpar('hfi_gain' , Ki )
setpar('hfi_gain_int2' ,1*2*pi)
setpar('hfi_V' , hfi_v)
setpar('hfi_on' , 1)


signals = [ 'Id_meas', 'Iq_meas' , 'Vd' , 'Vq' , 'thetaPark_enc'  , 'thetaPark_obs' , 'hfi_curangleest' , 'hfi_dir' ,'eradpers_lp' , 'delta_id' ,'delta_iq' , 'hfi_abs_pos']
setTrace( signals )



setpar( 'Iq_offset_SP' , 5)

# df1 = trace( 1 ) 

# hfi_v = 5
# setpar('hfi_V' , hfi_v)
df = trace( 2 ) 


setpar( 'Iq_offset_SP' , 0)
df2 = trace( 1 ) 


df.filter(regex='hfi_dir|thetaPark').plot()

# df.filter(regex='hfi_dir').plot()



# ax = df.filter(regex='thetaPark_').plot( grid = 1 )
# tmp = df.filter( regex= 'hfi_dir') + pi
# tmp[tmp > 2*pi] -= 2*pi 
# tmp.columns = ['hfi_dir']
# tmp.plot( ax= ax )


(df.filter(regex='eradpers_lp') /2/pi*60).plot()
# ax = df.filter(regex='hfi_dir|thetaPark_enc').plot()

# df.filter(regex='hfi_curangl|thetaPark_enc').plot()


# df.filter(regex='erpm').plot()

df.filter(regex='I').plot()
df.filter(regex='V').plot()
# df.filter(regex='delta').plot()
# df.filter(regex='hfi_curangleest_simple').plot()

df.filter(regex='hfi_abs_pos|thetaPar').plot()


  
# Ld = 190e-6  
# Lq = 235e-6

# y = -df['delta_iq'];
# # x = df['delta_id'] - 0.5 * 15 * Ts * ( 1/Ld + 1/Lq ) 
# x = df['delta_id'] - np.mean(  df['delta_id']  )

# hfi_curangleest = 0.5 * np.arctan2( y , x );

# ax = hfi_curangleest.plot( )
# df.filter(regex='delta').plot( ax=ax , grid=1 )


# ax = x.plot( grid=1  )
# y.plot( ax=ax  )



#%% 
setpar( 'Iq_offset_SP' , 5)
# tracesignal([ 'Vq' , 'Vd' , 'Ialpha' , 'Ibeta'] , 2)

#%% 
setpar( 'Iq_offset_SP' , 0)

#%% 
setpar('anglechoice' ,1)

#%% HFI
maxsamples = int(10)
hfi_v = 12
i = 0;
Nangles = 360
curtotvec = np.zeros( int(Nangles) ) 
curorttotvec = np.zeros( int(Nangles) ) 
hfi_curangleestvec = np.zeros( int(Nangles) ) 
thetaPark_encvec= np.zeros( int(Nangles) ) 
ANGLES = np.linspace( 0 , 2*pi, Nangles , 0)
# ANGLES = np.linspace( 2*pi , 0, Nangles , 0)
# ANGLES = np.linspace( 0 , 0, Nangles , 0)
# ANGLES = np.linspace( 0.7 , 0.8, Nangles , 0)
# ANGLES = np.linspace( pi/2 , pi/2, Nangles , 0)
DF = list()

setpar('hfi_V' , hfi_v)
setpar('hfi_on' , 1)
# setpar('hfi_maxsamples', maxsamples)

signals = [ 'Ialpha', 'Ibeta' , 'Valpha' , 'Vbeta' , 'hfi_curtot' ,'hfi_curorttot' , 'hfi_cursample' ,'hfi_curprev' ,'thetaPark' , 'thetaPark_enc' ]
for angle in ANGLES:
    setpar('hfi_curtot' , 0)
    setpar('hfi_curorttot' , 0)
    setpar('hfi_dir' , angle)
    # setpar('hfi_cursample' , -2)
    
    setTrace( signals )
    
    N = 40
    
    # Nsig = 2+n_trace
    # setpar('Nsend' , N , ser)
    setpar('hfi_cursample' , -2)

    # if N < 100:
    #     buffer = ser.read(N*4*Nsig)
    # else:
    #     buffer = ser.readall()
    
    # n = np.zeros( int(N) , dtype=np.uint32 )
    # t = np.zeros( int(N) ) 
    # s = np.zeros( ( int(N) , Nsig-2 ))
    
    # sigtypes = ['I' , 'I' ] + ser.tracesigtypes
    # for sigtype in set(sigtypes):
    #     indices = [i for i in range(len(sigtypes)) if sigtypes[i] == sigtype]
    #     indices = np.array(indices)
    #     if sigtype == 'b':
    #         tmp = np.ndarray( ( int(N) , Nsig ) , 'bool', buffer[0::4] )
    #     else:
    #         tmp = np.ndarray( ( int(N) , Nsig ) , sigtype, buffer )
    #     if sigtype == 'I':
    #         n = tmp[:,indices[0]]
    #         t = tmp[:,indices[1]] / 1e6
    #         s[:,indices[2:]-2] = tmp[:,indices[2:]]
    #     else:
    #         s[:,indices-2] = tmp[:,indices]



    while( getsig('hfi_cursample') < maxsamples) :
        a=1;
        
    df = pd.DataFrame( s[:,0:len(signals)], columns=signals )
    df.index = 1e6*(t - t[0])
    df.index.name = 'Time [µs]'
    DF.append(df)

    curtot = getsig('hfi_curtot')
    curorttot = getsig('hfi_curorttot')
    hfi_cursamplevec = getsig('hfi_cursample')
    hfi_curangleestvec[i] = getsig('hfi_curangleest')
    
    # setpar('hfi_on' , 0)
    curtotvec[i] = curtot
    curorttotvec[i] = curorttot
    thetaPark_encvec[i] = getsig('thetaPark_enc')
    i += 1


angle = getsig('thetaPark_enc')




# df = pd.DataFrame( s[:,0:len(signals)], columns=signals )
# df.index = 1e6*(t - t[0])
# df.index.name = 'Time [µs]'
# df.filter(regex='[VI]').plot( grid=1)


# DF[0].filter(regex='^[VIh]').plot( grid=1)
# DF[1].filter(regex='^[VIh]').plot( grid=1)


i_par = curtotvec / (maxsamples * hfi_v )
i_ort = curorttotvec / (maxsamples * hfi_v )

plt.figure(1)
plt.plot( ANGLES/2/pi*360 , i_par)
plt.plot( ANGLES/2/pi*360 , i_ort )


thetaest1 = i_ort / (Ts * (1/Lq - 1/Ld ))
thetaest2 = 0.5*np.arctan2( -i_ort/Ts , (i_par/Ts - 0.5 * ( 1/Ld + 1/Lq ))  )
plt.figure(2)
plt.plot( ANGLES , thetaest1 )
plt.plot( ANGLES , thetaest2 )
plt.plot( ANGLES , hfi_curangleestvec )
plt.xlabel('Offset from rotor angle real angle [rad]')
plt.ylabel('Estimated offset from rotor angle real angle [rad]')
# plt.legend( ['simple estimate' , 'atan2 estimate'])
plt.grid()

# plt.plot( [angle/2/pi*360,angle/2/pi*360] , [ min(i_ort) , max(i_par) ])


# plt.figure()
# plt.plot( thetaPark_encvec , curtotvec / (maxsamples * hfi_v))

# plt.figure()
# plt.plot( thetaPark_encvec , curorttotvec / (maxsamples * hfi_v))

#%%

signals = [ 'Ialpha', 'Ibeta' , 'Valpha' , 'Vbeta' , 'hfi_curtot' ,'hfi_curorttot' , 'hfi_cursample' ,'hfi_curprev' ,'thetaPark' , 'thetaPark_enc' ]
setTrace( signals )

N = 100

Nsig = 2+n_trace
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


plt.figure()
plt.plot( t , s)



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
setpar('VSP' , 0)

#%% 
ser.write( b'o' + struct.pack('f',  0) ) # Restart controller

#%% 
setpar( 'Iq_offset_SP' , 10)

#%% 
setpar( 'Iq_offset_SP' , 5)

#%% 
setpar('anglechoice' ,1)

#%% Current plant FRF measurement PRBS
# ser = start( ser )

# Lq = 2.6e-4 
# Ld = 2.2e-4
# R = 0.33
# f_bw = 5e3 /2/pi;

# setpar( 'Kp_iq' , Lq * f_bw * 2 *pi  ) # Current loop Kp
# setpar( 'Ki_iq' , R/Lq  ) # Current loop Ki
# setpar( 'Kp_id' , Ld * f_bw * 2 *pi  ) # Current loop Kp
# setpar( 'Ki_id' , R/Ld  ) # Current loop Ki

setpar('useIlowpass' , 0 , ser)

NdownsamplePRBS = 1
N = 10*NdownsamplePRBS*2047 + 1/Ts
signals = [ 'D', 'Q' , 'Id_meas' , 'Iq_meas', 'Id_meas2' , 'Iq_meas2' , 'dist' , 'emech1' , 'emech2' , 'Vq' , 'Vd', 'Vq2' , 'Vd2' ,'mechcontout']
setTrace( signals )


gain = 0.5
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

plt.figure(2)
bode( OLq , f , 'OL Current loop (q axis)')
bode( OLd , f , 'OL Current loop (d axis)')

plt.figure(3)
bode( OLq / (1 + OLq) , f , 'CL Current loop (q axis)')
bode( OLd / (1 + OLd) , f , 'CL Current loop (d axis)')

plt.figure(4)
plt.plot( f , np.abs( 1/(PQ * f * 2 * np.pi)  ) *1e6)
plt.grid(which='major')
plt.xlim([ 1e3, 10e3])
plt.ylim([ 100 , 300])
plt.title('Lq [uH]')

plt.figure(5)
plt.plot( f , np.abs( 1/(PD * f * 2 * np.pi)  ) *1e6 )
plt.grid(which='major')
plt.xlim([ 1e3, 10e3])
plt.ylim([ 100 , 300])
plt.title('Ld [uH]')

plt.figure(6)
plt.plot( f , np.abs( 1/(PD * f * 2 * np.pi)  ) *1e6 / np.sinc( f * Ts) ) #Correction for zero order hold
plt.plot( f , np.abs( 1/(PQ * f * 2 * np.pi)  ) *1e6 / np.sinc( f * Ts) ) #Correction for zero order hold
plt.xlim([ 1e3, 30e3])
plt.ylim([ 0 , 60])
plt.title('L [uH]')
plt.legend( ['Ld','Lq'] )
plt.grid()
plt.xlabel('Frequency [Hz]')


np.sinc( f * Ts)

#%%


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
# R = 0.35
# Ld = 2.0e-4

# Lq = 4.5e-6 
# Ld = 4.5e-6 
# R = 0.07

Lq = 600e-6 
Ld = 600e-6 
R = 0.264  
a = ct.c2d(ct.tf( 1 , [Ld , R ]) , Ts , 'zoh')  * ct.tf ([1] , [1, 0] , Ts )
y = ct.evalfr( a , x=exp(1j * 2*pi*f * Ts) )

plt.figure()
bode( PD , f , 'Measured D axis plant')
bode( y , f , 'Model: 1/(Lds+R), discretized')


#%%
plt.figure(1)
bode( OLq , f , 'OL Current loop (q axis)')
bode( OLd , f , 'OL Current loop (d axis)')

plt.figure(10)
bode( OLq / PQ, f , 'Cont q')
bode( OLd / PD, f , 'Cont d')



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
ser = start( ser )
CL()
#%%
setpar('anglechoice' ,3)
# setpar('anglechoice' ,100)

Ki =500*2*pi
hfi_v = 0.5

setpar('hfi_usesimple' , 1)
setpar('hfi_gain' , Ki )
setpar('hfi_gain_int2' , 1*2*pi)
setpar('hfi_V' , hfi_v)
setpar('hfi_on' , 1)

#%%
setpar('hfi_on' , 0)
#%%
setpar( 'Iq_offset_SP' , 2)
#%%
setpar( 'Iq_offset_SP' , 0)
#%%
setpar( 'hfi_useforfeedback' , 1)

setpar( 'Kp' , 0)
setpar( 'rmechoffset' , 0 , ser)
setpar( 'rmechoffset' , -getsig( 'emech1'  , ser) , ser)
getsig('emech1'  , ser)

# fBW = 30
# setpar( 'fBW' , fBW)
# setpar( 'alpha1' , 4)
# setpar( 'alpha2' , 3)
# setpar( 'fInt' , fBW / 10)
# setpar( 'fLP' , fBW * 6)
# setpar( 'Kp' , 2)

#%%
fBW = 25
setpar( 'fBW' , fBW)
setpar( 'alpha1' , 6)
setpar( 'alpha2' , 6)
setpar( 'fInt' , fBW / 20)
setpar( 'fLP' , fBW * 6)
setpar( 'Kp' , 0.3)

ser.write( b'C' + struct.pack('I',  0) ) # Set controllers

#%% Trampa 160kv
ser = start( ser )
CL(300)

setpar('Id_offset_SP', 5)
time.sleep(0.5)
setpar('Id_offset_SP', 0)


setpar('hfi_use_lowpass' , 1)

setpar('hfi_method', 3)

Ki = 1000*2*pi
hfi_v = 2

setpar('hfi_maxvel', 1e6)
setpar('hfi_gain' , Ki )
setpar('hfi_gain_int2' , 5*2*pi)
setpar('hfi_V' , hfi_v)
setpar('hfi_on' , 1)
setpar('anglechoice' ,3)

setpar( 'hfi_useforfeedback' , 1)

setpar('I_max' , 20)

#%%  
setpar( 'Kp' , 0)
setpar( 'rmechoffset' , 0 , ser)
setpar( 'rmechoffset' , -getsig( 'emech1'  , ser) , ser)
getsig('emech1'  , ser)

# Kp =4.8
# fBW = 20
# alpha1 = 3
# alpha2 = 3
# fInt = fBW / 8
# fLP = fBW * 7
# Kp =6.2
# fBW = 25
# alpha1 = 3
# alpha2 = 3
# fInt = fBW / 8
# fLP = fBW * 7

Kp =10
fBW = 30
alpha1 = 3
alpha2 = 3
fInt = fBW / 6
fLP = fBW * 7
# Kp =30
# fBW = 40
# alpha1 = 3
# alpha2 = 3
# fInt = fBW / 8
# fLP = fBW * 7

setpar( 'fBW' , fBW)
setpar( 'alpha1' , alpha1)
setpar( 'alpha2' , alpha2)
setpar( 'fInt' , fInt)
setpar( 'fLP' ,  fLP)
setpar( 'Kp' , Kp)

ser.write( b'C' + struct.pack('I',  0) ) # Set controllers


#%% Rotomax 100CC
ser = start( ser )
CL(2e3)

setpar('anglechoice' ,3)
Ki =500*2*pi
hfi_v = 1

setpar('hfi_gain' , Ki )
setpar('hfi_gain_int2' , 5*2*pi)
setpar('hfi_V' , hfi_v)
setpar('hfi_on' , 1)

setpar( 'hfi_useforfeedback' , 1)

#%%  
setpar( 'Kp' , 0)
setpar( 'rmechoffset' , 0 , ser)
setpar( 'rmechoffset' , -getsig( 'emech1'  , ser) , ser)
getsig('emech1'  , ser)

Kp =15
fBW = 15
alpha1 = 3
alpha2 = 3
fInt = fBW / 8
fLP = fBW * 7

setpar( 'fBW' , fBW)
setpar( 'alpha1' , alpha1)
setpar( 'alpha2' , alpha2)
setpar( 'fInt' , fInt)
setpar( 'fLP' ,  fLP)
setpar( 'Kp' , Kp)

ser.write( b'C' + struct.pack('I',  0) ) # Set controllers

#%% Wittenstein 1
ser = start( ser )

setpar('ridethewave' , 1 )
time.sleep( 1 )

setpar( 'commutationoffset',0) 
setpar('Valpha_offset' , 2) 
time.sleep(1)
offset = getsig('thetaPark_enc')
setpar('Valpha_offset' , 0)
setpar( 'commutationoffset', -offset) 

CL(300)

setpar('hfi_use_lowpass' , 1)

setpar('hfi_method', 3)

Ki = 1000*2*pi
hfi_v = 14

setpar('hfi_gain' , Ki )
setpar('hfi_gain_int2' , 5*2*pi)
setpar('hfi_V' , hfi_v)
setpar('hfi_on' , 1)
setpar('anglechoice' ,3)

setpar( 'hfi_useforfeedback' , 1)

#%%  
setpar( 'Kp' , 0)
setpar( 'rmechoffset' , 0 , ser)
setpar( 'rmechoffset' , -getsig( 'emech1'  , ser) , ser)
getsig('emech1'  , ser)


Kp = 0.45
fBW = 15
alpha1 = 3
alpha2 = 3
fInt = fBW / 8
fLP = fBW * 7
# Kp =1.16195
# fBW = 25
# alpha1 = 3
# alpha2 = 3
# fInt = fBW / 8
# fLP = fBW * 7

#Ecoder
# Kp = 45
# fBW = 100
# alpha1 = 3
# alpha2 = 3
# fInt = fBW / 8
# fLP = fBW * 7
# Kp = 144
# fBW = 200
# alpha1 = 3
# alpha2 = 3
# fInt = fBW / 8
# fLP = fBW * 7

setpar( 'fBW' , fBW)
setpar( 'alpha1' , alpha1)
setpar( 'alpha2' , alpha2)
setpar( 'fInt' , fInt)
setpar( 'fLP' ,  fLP)
setpar( 'Kp' , Kp)

ser.write( b'C' + struct.pack('I',  0) ) # Set controllers




#%%  

signals = ['Id_meas',  'Iq_meas','Vd' , 'Vq' , 'hfi_dir'];
setTrace( signals )

time.sleep(0.1)

N = 50
Nsig = 2+n_trace
setpar('Nsend' , N , ser)
setpar('hfi_V' , 12)

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


setpar('hfi_V' , 2)

plt.figure()
plt.plot( s[:,0])

if( abs(np.min( s[0:30,0])) > np.max( s[0:30,0]) ):
    print('Wrong')
    setpar('hfi_dir' , np.mod(getsig('hfi_dir') + np.pi , 2*pi ))
else:
    print('Right')
print(np.mean( s[:,0:26] ))

setpar( 'Iq_offset_SP' , 5)
time.sleep(1)
setpar( 'Iq_offset_SP' , 0)



#%% 
setpar( 'Iq_offset_SP' , 10)
setpar( 'Iq_offset_SP' , 0)

#%%     
setpar( 'offsetVel' , 3*np.pi)


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
# fLP = fBW * 8

Kp = 144
fBW = 200
alpha1 = 3
alpha2 = 3
fInt = fBW / 8
fLP = fBW * 7

num, den = leadlag( fBW , alpha1, alpha2, Ts)
CONT = freqrespd( num , den ,f , Ts )
num, den = lowpass2( fLP , 0.7 , Ts)
LP2 = freqrespd( num , den ,f , Ts )
num, den = Integrator( fInt , Ts)
INTEGRATOR = freqrespd( num , den ,f , Ts )

Kp = 1/np.abs(np.interp( fBW , f, CONT * LP2 * Pmech *  (INTEGRATOR+1)))

plt.figure(1)
# bode( DIST/TORQUE - 1 , f)
# bode( CONT * LP2 * Pmech * Kp* (INTEGRATOR+1)  , f , 'OL reconstruct')
bode( CONT * LP2 * Pmech * Kp* (INTEGRATOR+1)  , f , 'OL reconstruct')

plt.figure(2)
# bode( DIST/TORQUE - 1 , f)
# bode( CONT * LP2 * Pmech * Kp* (INTEGRATOR+1)  , f , 'OL reconstruct')
nyquist( CONT * LP2 * Pmech * Kp* (INTEGRATOR+1)  , f , 'OL reconstruct')




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

vel = 0*np.pi
if vel > 0:
    for i in np.arange( 0 , 1 , 0.01):
        setpar( 'offsetVel' ,  vel*i )
        time.sleep( 0.01)

setpar( 'NdownsamplePRBS' , NdownsamplePRBS)
setpar( 'distval' , 0.3) #disturbance amplitude
setpar('Vq_distgain' , 0)
setpar('Vd_distgain' , 0)
setpar('Iq_distgain' , 0)
setpar('Id_distgain' , 0)
setpar('mechdistgain' , 1)

signals = [ 'mechcontout' , 'D', 'Q' , 'Id_meas' , 'Iq_meas' , 'dist' , 'emech1' , 'emech2' , 'Vq' ,'ymech1' , 'mechcontout2' , 'hfi_abs_pos']
setTrace( signals )
t,s = readData( int(N) )
if vel > 0:
    for i in np.arange( 1 , 0 , -0.01):
        setpar( 'offsetVel' ,  vel*i )
        time.sleep( 0.05)
setpar( 'distval' , 0) #disturbance amplitude
setpar( 'NdownsamplePRBS' , 1) #Downsampling


E1,f = getFFT( s[:,signals.index('emech1')] , NdownsamplePRBS )
E2,f = getFFT( s[:,signals.index('emech2')] , NdownsamplePRBS )
hfi_pos,f = getFFT( s[:,signals.index('hfi_abs_pos')] , NdownsamplePRBS )

DIST,f = getFFT( s[:,5] , NdownsamplePRBS )
TORQUE,f = getFFT( s[:,0] , NdownsamplePRBS  )
TORQUE2,f = getFFT( s[:,10] , NdownsamplePRBS  )

Pmech  = -E1/TORQUE 
Pmech2 = -E2/TORQUE2 
N_pp = getsig('N_pp')
Pmech3 = hfi_pos/TORQUE /N_pp 

plt.figure(1)
bode( Pmech , f)

plt.figure(2)
bode( DIST/TORQUE - 1 , f)
# bode( DIST/TORQUE2 - 1 , f)

# plt.figure(2)
# bode( Pmech , f)
# bode( Pmech3, f)


plt.figure(3)
bode( Pmech * (f*2*pi)**2, f)
# bode( Pmech3 , f)


#%%
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
setpar('hfi_method', 2)

ss_fstart = 10;
ss_fstep = 50;
ss_fend = 1000;
ss_n_aver = 10;
ss_distgain = 0.3;
ss_distoffset = 0;

offsetvel = 0 * pi

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
setpar('mechdistgain' , 0)
setpar('hfi_distgain' , 1)

N = int( t_tot / Ts)
setpar( 'offsetVel' , offsetvel )
setpar( 'ss_fstart' , ss_fstart )   
setpar( 'ss_fend' , ss_fend)
setpar( 'ss_fstep' , ss_fstep)
setpar( 'ss_n_aver' , ss_n_aver)
setpar( 'distoff' , ss_distoffset)
time.sleep(0.5)

signals = [ 'ss_f' , 'dist' , 'mechcontout' ,'emech1' ,'emech2' , 'hfi_dir' , 'hfi_error']
setTrace( signals )
ser.write( b's' + struct.pack('f', ss_distgain ) )

t,s = readData( int(N) )

ser.write( b's' + struct.pack('f',  0) )
setpar( 'offsetVel' , 0.0*np.pi)
#ser.write( b'o' + struct.pack('f',  0) ) # Restart controller
ss_f = s[:, signals.index('ss_f')]
f = np.unique( ss_f[ss_f>0]  )
S = np.zeros( len(f) ,dtype=np.complex_)
PS = np.zeros( len(f) ,dtype=np.complex_)
PS2 = np.zeros( len(f) ,dtype=np.complex_)
HFI_transfer = np.zeros( len(f) ,dtype=np.complex_)
HFI_Plant = np.zeros( len(f) ,dtype=np.complex_)
#Pelec = np.zeros( len(f) ,dtype=np.complex_)
for freq in f:
    I = ss_f == freq
    DIST , f2 = getFFT_SS( s[I, signals.index('dist')] )
    mechcontout , f2 = getFFT_SS( s[I, signals.index('mechcontout')] )
    EMECH , f2 = getFFT_SS( s[I, signals.index('emech1')] )
    EMECH2 , f2 = getFFT_SS( s[I, signals.index('emech2')] )
    HFI_DIR , f2 = getFFT_SS( s[I, signals.index('hfi_dir')] )
    HFI_ERROR , f2 = getFFT_SS( s[I, signals.index('hfi_error')] )

    index = np.argmin( abs(freq-f2 ))  

#    SENS , f = getFFT_SS( sens[I] )
#    VOUT , f = getFFT_SS( Vout[I] )

#    Pelec[freq==f] = SENS[index] / VOUT[index]  
    S[freq==f] = mechcontout[index] / DIST[index]
    PS[freq==f] = -EMECH[index] / DIST[index]  
    PS2[freq==f] = -EMECH2[index] / DIST[index]     
    HFI_transfer[freq==f] = HFI_DIR[index] / DIST[index]   
    HFI_Plant[freq==f] = -HFI_ERROR[index] / HFI_DIR[index]   
    


#f, results = goertzel(some_samples, 44100, (400, 500), (1000, 1100))
# Pmech = PS/S
# plt.figure(1)
# bode( Pmech  , f , 'P')
# bode( PS2/S , f , 'P2')

# plt.figure(2)
# bode( 1/S-1 , f , 'OL')
# plt.figure(3)
# nyquist( 1/S-1 , f , 'OL')

# def normalize(v): 
#     norm = max(abs(v))
#     if norm == 0: 
#        return v
#     return v / norm

# freq = ss_f[ np.argmin( abs(1000-ss_f ) ) ]
# plt.figure();
# plt.plot( normalize( s[ ss_f == freq , signals.index('dist')] ))
# plt.plot( normalize( s[ ss_f == freq , signals.index('mechcontout')] ))
# plt.plot(normalize( s[ ss_f == freq , signals.index('emech1')] ))
# plt.show();

# plt.figure()
# bode( HFI_transfer , freqs )

# plt.figure();
# plt.plot(t , s[:,[2, 5]])
# plt.show();

# print( np.angle(HFI_transfer[0]) )



plt.figure(1)
bode( HFI_Plant  , f , 'HFI_Plant')

plt.figure(2)
bode( 1/HFI_transfer -1  , f , 'HFI_OL')
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
setpar('muziek_gain_V' , 1)
#%%
setpar('muziek_gain_V' , 0)

#%% Music on
setpar( 'offsetVel' , 2*np.pi)
setpar( 'muziek_gain' , 8) # music current
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
#%% Setpoint
setpar('hfi_method' , 3)

# setpar('hfi_gain' , 1000*2*pi )
# setpar('hfi_gain_int2' , 20*2*pi)
# setpar('hfi_use_lowpass' , 1 )
setpar('hfi_maxvel', 1e6)

Jload = 0.0003162277 #kgm² #160kv
setpar( 'Jload' ,  Jload )

#Wittenstein (with broken blue thing)
# Jload = 7e-05 #kgm²
# vFF = 0  #Nm/(rad/s)
# setpar( 'Jload' ,  Jload  )
# setpar( 'velFF' ,  vFF  )


setpar('I_max' , 20)

# setpar( 'Jload' ,  Jload )
# setpar( 'velFF' , vFF )
# setpar( 'rdelay' , 0)
#getsig( 'Jload' )
#getsig( 'velFF' )

# p = 2*np.pi
# v = 300
# a = 700*np.pi
# j = 100000*np.pi
# Nsp = 1

p = 20*np.pi
v = 100
a = 100*np.pi
j = 100000*np.pi
Nsp = 1


# p = 2*np.pi/360*0.1
# v = 0.3
# a = 3
# j = 200
# Nsp = 1

[t1, t2, t3, jd] = prepSP(  p , v , a , j )


# signals = [ 'rmech', 'ymech1' , 'emech1' , 'vel' , 'Iq_meas', 'Id_meas' , 'acc' , 'Va', 'Vb', 'Vc' , 'mechcontout', 'sensBus']
signals = [ 'rmech', 'ymech1' , 'emech1' , 'vel' , 'Iq_meas', 'Id_meas' , 'acc'  , 'mechcontout', 'sensBus','sensBus_lp' , 'encoderPos1' , 'P_tot' , 'I_bus']
setTrace( signals )


# delay = 0.3
# for p in np.linspace( 2*np.pi/360*1 , 2*np.pi/360*10 , 10):
#     [t1, t2, t3, jd] = prepSP(  p , v , a , j )
#     setpar('SPdir' , 1)
#     setpar('spNgo',Nsp)
#     while (getsig('REFstatus') > 0):
#         bla = 1;
#     time.sleep(delay)
#     setpar('SPdir' , 0)
#     setpar('spNgo',Nsp)
#     while (getsig('REFstatus') > 0):
#         bla = 1;
#     time.sleep(delay)

# delay = 0.1
# for p in np.linspace( 0.1 , 2*pi , 11):
#     [t1, t2, t3, jd] = prepSP(  p , v , a , j )
#     setpar('SPdir' , 1)
#     setpar('spNgo',Nsp)
#     while (getsig('REFstatus') > 0):
#         bla = 1;
#     time.sleep(delay)
#     setpar('SPdir' , 0)
#     setpar('spNgo',Nsp)
#     while (getsig('REFstatus') > 0):
#         bla = 1;
#     time.sleep(delay)

# time.sleep(1)
# delay = 0.00
# for p in np.linspace( 0.1 , 2*pi , 11):
#     [t1, t2, t3, jd] = prepSP(  p , v , a , j )
#     setpar('SPdir' , 1)
#     setpar('spNgo',Nsp)
#     while (getsig('REFstatus') > 0):
#         bla = 1;
#     time.sleep(delay)
#     setpar('SPdir' , 0)
#     setpar('spNgo',Nsp)
#     while (getsig('REFstatus') > 0):
#         bla = 1;
#     time.sleep(delay)


# setpar('SPdir' , 1)
# setpar('spNgo',Nsp)
# while (getsig('spNgo') > 0 or getsig('REFstatus') > 0):
#     a=1;
# setpar('SPdir' , 0)
# setpar('spNgo',Nsp)

setpar('SPdir' , 1)
setpar('spNgo',Nsp)

df = trace(Nsp*(0.01+4*t1+2*t2+t3)+0.1)  

df.filter(regex='sensBus').plot()
df.filter(regex='P_tot').plot()
df.filter(regex='I_bus').plot()

# enc = (df['encoderPos1']*2*pi/20e3)
# enc = enc - enc[0] + np.mean(df['rmech'][0])

# plt.figure()
# df['rmech'].plot()
# df['ymech1'].plot()
# enc.plot()
# plt.legend()
# plt.grid()

# df.filter(regex='I').plot()
# df.filter(regex='acc').plot()


# df.filter(regex='hfi_error').plot()
# df.filter(regex='hfi_ffw').plot()
# df.filter(regex='hfi_dir').plot()


#%%
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
v = 5*2*pi
a = 30*2*pi
j = 50000*2*pi
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

setpar('Ndownsample' , int( 0.02/Ts ))
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




#%%
ser = start( ser )
CL(300)

#%%
setpar('anglechoice' ,3)
setpar('hfi_method', 2)

Ki = 500*2*pi
hfi_v = 3

setpar('hfi_gain' , Ki )
setpar('hfi_gain_int2' , 5*2*pi)
setpar('hfi_V' , hfi_v)
setpar('hfi_on' , 1)
setpar('hfi_use_lowpass' , 0)

setpar( 'hfi_useforfeedback' , 1)

NdownsamplePRBS = 1
N = 10*NdownsamplePRBS*2047 + 1/Ts

setpar( 'NdownsamplePRBS' , NdownsamplePRBS)
setpar( 'distval' , 0.2) #disturbance amplitude
setpar('Vq_distgain' , 0)
setpar('Vd_distgain' , 0)
setpar('Iq_distgain' , 0)
setpar('Id_distgain' , 0)
setpar('mechdistgain' , 0)
setpar('hfi_distgain' , 1)

signals = [ 'mechcontout' , 'D', 'Q' , 'Id_meas' , 'Iq_meas' , 'dist' , 'emech1' , 'emech2' , 'Vq' ,'ymech1' , 'mechcontout2' , 'hfi_dir' , 'hfi_error']
setTrace( signals )
t,s = readData( int(N) )
setpar( 'distval' , 0) #disturbance amplitude
setpar( 'NdownsamplePRBS' , 1) #Downsampling

hfi_pos,f = getFFT( s[:,signals.index('hfi_dir')] , NdownsamplePRBS )
hfi_error,f = getFFT( s[:,signals.index('hfi_error')] , NdownsamplePRBS )

DIST,f = getFFT( s[:,5] , NdownsamplePRBS )


S = hfi_pos / DIST
PS = -hfi_error / DIST
P = PS / S
OL = 1/S-1


plt.figure(1)
bode( P , f)


plt.figure(2)
bode( OL , f)

plt.figure(3)
nyquist( OL , f)

#%%
setpar( 'Iq_offset_SP' , 0)


#%%
CL(50)
setpar('hfi_V' , 14)
signals = [ 'Iq_offset_SP'  , 'Iq_meas' , 'Id_meas'  , 'Vq' , 'Vd' , 'Va' , 'Vb', 'Vc' , 'sensBus', 'thetaPark',  'thetaPark_obs' , 'thetaPark_enc']
setTrace( signals )
df = trace( 2 ) 
df.filter(regex='thetaPark').plot()
