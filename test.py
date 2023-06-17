import matplotlib.pyplot as plt
import pandas as pd
import serial
import struct
import numpy as np
import time
import os 
os.chdir('C:\GIT\Teensy_DualMotorBoard_V1')
import make3
import control as ct

pi = np.pi
# plt.rcParams["figure.dpi"] = 200 #Use this with 4k screens
plt.rcParams.update({"axes.grid": True})
plt.rcParams.update({"legend.loc": 'upper left'})

try:
    ser.close()
    print('')
except:
    print('')
ser = serial.Serial('COM4', timeout=0.1)  # open serial port

ser.flush()
# Get signal names, lengths and types
ser.write(b'T')
buffer = ser.readlines()


def bode(H, f, name='Data', title='bode plot'):
    mag = np.abs(H)
    phase = np.angle(H)
    ax1 = plt.subplot(2, 1, 1)
#    plt.loglog(f, mag)    # Bode magnitude plot
    plt.semilogx(f, 20*np.log10(mag), label=name)    # Bode magnitude plot
    plt.grid(1, 'both', 'both')
    plt.ylabel('Magnitude [dB]')
    plt.title(title)
    plt.legend()
#    ax1.xaxis.set_minor_formatter(ticker.ScalarFormatter())
    ax2 = plt.subplot(2, 1, 2, sharex=ax1)
    plt.plot(f, phase * 180 / np.pi)  # Bode phase plot
    plt.grid(1, 'both')
    plt.xlabel('Frequency [Hz]')
    plt.ylabel('Phase [deg]')
    plt.ylim(-180, 180)
    # ax2.yaxis.set_major_locator(plt.LinearLocator(5))
    ax2.yaxis.set_major_locator(plt.MaxNLocator(5, steps=[1,2,2.5,5,9,10]))
    plt.tight_layout()
    plt.show()


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

def setTrace(signals, downsample=1) :
    if isinstance(signals, str) or isinstance(signals, int):
        signals = [signals]

    ser.signalsnames = signals
    i = 0
    ser.signals = []
    signalsout = []
    ser.tracebytes = 0;
    for signal in ser.signalsnames:
        signal = getsigid( signal )
        ser.signals.append(signal)
        signalsout.append(signames[signal])
        # ser.tracesigtypes[i] = ser.sigtypes[signal]
        ser.write(b't' + struct.pack('I', signal) + struct.pack('I', i))
        ser.tracebytes += sigbytes[signal]
        i += 1
    # Hardcoded high value, to be improved
    ser.write(b't' + struct.pack('I', 499) + struct.pack('I', i))
    setpar('motor.conf.Ndownsample' , int( downsample ))
    return signalsout


def trace(t, outtype='df'):
    Ts_downsample = Ts * getsig('motor.conf.Ndownsample')
    i = int(np.ceil(t/Ts_downsample).astype('int') + 1)
    ser.reset_input_buffer()
    # if i < 1e5: //This part just caused issues (long delta time) at the start of the trace. No idea why.
    #     tmp = ser.timeout
    #     ser.timeout = t * 1.2 + 0.5
    #     buffer = bytearray(int(ser.tracebytes * i))
    #     ser.write( b'b' + struct.pack('I', i ))
    #     ser.readinto(buffer)
    #     ser.timeout = tmp
    # else:
    ser.write(b'b' + struct.pack('I', i))
    buffer = ser.readall()
    # buffer = ser.read( int(ser.tracebytes * i) )
    if outtype == 'df':
        dtypestrace = []
        for isignal in ser.signals:
            if(isinstance(dtypessep2[isignal], list)):
                for dtype in dtypessep2[isignal]:
                    dtypestrace.append(dtype)
            else:
                dtypestrace.append(dtypessep2[isignal])
        arr = np.ndarray(i, dtype=dtypestrace,  buffer=buffer)
        df = pd.DataFrame(arr)
        # try:
        #     df.index = arr['motor.state.curtime'] / 1e6
        #     df.index -= df.index[0]
        #     df.index.name = 'Time [s]'
        #     df.drop(['motor.state.curtime'], axis=1, inplace=True)
        # except:
        df.index = np.linspace( 0 , len(df)-1 , len(df)) * Ts_downsample
        df.index.name = 'Time [s]'
        return df
    elif outtype == 'arr':
        dtypestrace = [dtypes[j] for j in ser.signals]
        arr = np.ndarray(i, dtype=dtypestrace,  buffer=buffer)
        return arr

types = [np.dtype('int8'), np.dtype('uint8'), np.dtype('int16'), np.dtype('uint16'), np.dtype(
    'int32'), np.dtype('uint32'), np.dtype('int64'), np.dtype('uint64'), np.dtype('float32'), np.dtype('float64')]
signames = []
for signame in buffer[::3]:
    signames.append(signame.strip().decode("utf-8"))
siglengths = []
for siglength in buffer[2::3]:
    siglengths.append(int(siglength.strip().decode("utf-8")))
sigtypes = []
sigbytes = []
i = 0
for sigtype in buffer[1::3]:
    sigtypes.append(types[int(sigtype.strip().decode("utf-8"))])
    sigbytes.append(sigtypes[-1].itemsize * siglengths[i])
    i += 1
dtypes = []
for i in range(len(sigtypes)):
    if siglengths[i] > 1:
        dtypes.append((signames[i],  siglengths[i] * sigtypes[i]))
    else:
        dtypes.append((signames[i],  sigtypes[i]))
dtypessep = []
for i in range(len(sigtypes)):
    for j in range(siglengths[i]):
        if siglengths[i] > 1:
            dtypessep.append((signames[i] + f'{[j]}',  sigtypes[i]))
        else:
            dtypessep.append((signames[i],  sigtypes[i]))
dtypessep2 = []
for i in range(len(sigtypes)):
    if siglengths[i] > 1:
        a = []
        for j in range(siglengths[i]):
            a.append((signames[i] + f'{[j]}',  sigtypes[i]))
        dtypessep2.append(a)
    else:
        dtypessep2.append((signames[i],  sigtypes[i]))


def getsigid( signal ):
    if isinstance(signal, list):
        signal = signal[0]
    if isinstance(signal, str):
        if signal[0] == 's':
            signal = 'motor.state' + signal[1:]
        if signal[0] == 'c':
            signal = 'motor.conf' + signal[1:]
        signal = signames.index(signal)
    return signal

def setpar(signal, value):
    signal = getsigid( signal )
    data = np.array(value, sigtypes[signal]).tobytes()
    if(len(data) == sigbytes[signal]):
        ser.write(b'S' + struct.pack('I',  signal) + data)
    else:
        raise ValueError(
            f'Size in bytes expected: {sigbytes[signal]}, given: {len(data)}.')


def setparpart(signal, value, startlocation=0):
    signal = getsigid( signal )
    data = np.array(value, sigtypes[signal]).tobytes()
    length = len(data)
    if((length + startlocation * sigtypes[signal].itemsize) <= sigbytes[signal]):
        ser.write(b's' + struct.pack('I',  signal) + struct.pack('I',
                  startlocation) + struct.pack('I',  length) + data)
    else:
        raise ValueError(
            f'Max size in bytes expected: {sigbytes[signal]}, given: {len(data) + startlocation * sigtypes[signal].itemsize}.')


def getsig(signal):
    signal = getsigid( signal )
    ser.write(b'G' + struct.pack('I',  signal))
    buffer = ser.read(sigbytes[signal])
    arr = np.ndarray(1, dtype=dtypes[signal][1],  buffer=buffer)
    return arr[0]


def getsigpart(signal, startlocation, length):
    signal = getsigid( signal )
    if((length + startlocation)*sigtypes[signal].itemsize <= sigbytes[signal]):
        ser.write(b'g' + struct.pack('I',  signal) + struct.pack('I',
                  startlocation) + struct.pack('I',  length * sigtypes[signal].itemsize))
        buffer = ser.read(sigbytes[signal])
        arr = np.ndarray(
            length, dtype=dtypes[signal][1].subdtype[0],  buffer=buffer)
        return arr
    else:
        raise ValueError(
            f'Requested valuues outside of array. Size array: {int(sigbytes[signal]/sigtypes[signal].itemsize)}, requested up to: {(length + startlocation) }.')


def getallsignals():
    ser.write(b'a')
    a = ser.readall()
    arr = np.ndarray(1, dtype=dtypessep,  buffer=a)
    df = pd.DataFrame(arr)
    return df


Ts = getsig('motor.conf.T')


def getFFTdf(df, Ndownsample, j0=int(1/Ts)):
    #    j0 = int(1/Ts) # These amount of samples are ignored
    L = 2047 * Ndownsample
    Naver = int((len(df)-j0)/L)
    # print( 'Naver =' , Naver )
    SIGNAL = np.fft.fft(df.iloc[j0:j0+L*Naver], axis=0)

    f = np.fft.fftfreq(L*Naver, Ts)
    SIGNAL = SIGNAL[f > 0]
    SIGNAL = 2*SIGNAL[Naver-1::Naver]
    f = f[f > 0]
    f = f[Naver-1::Naver]
    SIGNAL = SIGNAL[f < 1/(2*Ts*Ndownsample)]
    f = f[f < 1/(2*Ts*Ndownsample)]
    dfout = pd.DataFrame(SIGNAL)
    dfout.index = f
    dfout.index.name = 'Frequency [Hz]'
    dfout.columns = df.columns
    if 'motor.state1.dist' in dfout:
        dfout = dfout.div(dfout['motor.state1.dist'], axis=0)
    if 'motor.state2.dist' in dfout:  
        dfout = dfout.div(dfout['motor.state2.dist'], axis=0)
    return dfout


def getFFT(signal, Ndownsample, j0=int(1/Ts)):
    #    j0 = int(1/Ts) # These amount of samples are ignored
    L = 2047 * Ndownsample
    Naver = int((len(signal)-j0)/L)
    # print( 'Naver =' , Naver )
    SIGNAL = np.fft.fft(signal[j0:j0+L*Naver], axis=0)
    f = np.fft.fftfreq(L*Naver, Ts)
    SIGNAL = SIGNAL[f > 0]
    SIGNAL = 2*SIGNAL[Naver-1::Naver]
    f = f[f > 0]
    f = f[Naver-1::Naver]
    SIGNAL = SIGNAL[f < 1/(2*Ts*Ndownsample)]
    f = f[f < 1/(2*Ts*Ndownsample)]
    return SIGNAL, f

def prepSP(  p , v , a , j, axis=1):
    t1, t2, t3, jd = make3.make3( p , v , a , j , Ts );
    #double tstart, double t1, double t2, double t3, double p, double v_max, double a_max, double j_max
    if axis == 1:
        ser.write( b'1' + struct.pack('f',  t1) + struct.pack('f',  t2) + struct.pack('f',  t3) + struct.pack('d',  p) + struct.pack('f',  v) + struct.pack('f',  a)  + struct.pack('f',  jd) )
    else:
        ser.write( b'2' + struct.pack('f',  t1) + struct.pack('f',  t2) + struct.pack('f',  t3) + struct.pack('d',  p) + struct.pack('f',  v) + struct.pack('f',  a)  + struct.pack('f',  jd) )
    return t1, t2, t3, jd

def setNotch( axis, index, f0, debth_db, width ):
    if index < 8:
        ser.write( b'N' + struct.pack('I',  axis) + struct.pack('I',  index) + struct.pack('f',  f0) + struct.pack('f',  debth_db) + struct.pack('f',  width) )

def setLowpass( axis, index, f0, damping ):
    if index < 8:
        ser.write( b'L' + struct.pack('I',  axis) + struct.pack('I',  index) + struct.pack('f',  f0) + struct.pack('f',  damping) )


def readall( ):
    a = []
    for signal in signames:
        a.append( getsig(signal))
    df = pd.DataFrame( a , index=signames ).T
    return df

def discrete_notch( f0, debth_db, width):
    w0 = 2 * pi * f0 * Ts;
    if (debth_db < 0):
        alpha = width * np.sin(w0);
        alpha1 = alpha * 10 **(debth_db / 20);
    else: 
        alpha = width * np.sin(w0) * 10 **(-debth_db / 20);
        alpha1 = width * np.sin(w0);
    b0 =   (1 + alpha1) / (1 + alpha);
    b1 =  -2 * np.cos(w0) / (1 + alpha);
    b2 =   (1 - alpha1) / (1 + alpha);
    a0 = 1;
    a1 =  -2 * np.cos(w0) / (1 + alpha);
    a2 =   (1 - alpha) / (1 + alpha);
    
    return ct.TransferFunction( [b0, b1, b2] , [a0, a1, a2]  , float(Ts) )
    
def discrete_lowpass( f0, damping ):
    w0 = 2 * pi * f0 * Ts;
    alpha = np.sin(w0) * damping;
    b0 = 0.5 * (1 - np.cos(w0)) / (1 + alpha) ;
    b1 = 2 * b0;
    b2 = b0 ;
    a0 = 1;
    a1 = (-2 * np.cos(w0)) / (1 + alpha);
    a2 = (1 - alpha) / (1 + alpha);
    return ct.TransferFunction( [b0, b1, b2] , [a0, a1, a2]  , float(Ts) )

def vel( vel = 0 , motor = 0 ):
    if type(vel) == list:
        vel1 = vel[0]
        vel2 = vel[1]
    else:
        vel1 = vel
        vel2 = vel
    if motor == 0 or motor == 1:
        setpar('motor.state1.offsetVel' ,  vel1 )
    if motor == 0 or motor == 2:
        setpar('motor.state2.offsetVel' ,  vel2 )
        
def pos( target = 0 , motor = 0 , vel = 100 , acc = 1000):
    if type(target) == list:
        target1 = target[0]
        target2 = target[1]
    else:
        target1 = target
        target2 = target
    if motor == 0 or motor == 1:
        enc2rad1 = getsig('c1.enc2rad')
        target1 = round(target1/360*2*pi/enc2rad1)*enc2rad1
        delta = -getsig('motor.state1.rmech') + target1
        if delta !=0:
            prepSP( abs(delta) , vel , acc ,500000 , 1)
            setpar('motor.state1.SPdir' , delta>0)
            setpar('motor.state1.spNgo', 1)
    if motor == 0 or motor == 2:
        enc2rad2 = getsig('c2.enc2rad')
        target2 = round(target2/360*2*pi/enc2rad2)*enc2rad2
        delta = -getsig('motor.state2.rmech') + target2
        if delta !=0:
            prepSP( abs(delta) , vel , acc ,500000 , 2)
            setpar('motor.state2.SPdir' , delta>0)
            setpar('motor.state2.spNgo', 1)

def pos_wait( target = 0 , motor = 0 , vel = 100 , acc = 1000):
    pos( target , motor , vel , acc)
    while (getsig('motor.state1.REFstatus') > 0  or getsig('motor.state2.REFstatus') > 0 ):
        bla = 1



def rel( rel = 0 , motor = 0 ):
    if type(rel) == list:
        rel1 = rel[0]
        rel2 = rel[1]
    else:
        rel1 = rel
        rel2 = rel
    if motor == 0 or motor == 1:
        enc2rad1 = getsig('c1.enc2rad')
        delta = round(rel1/360*2*pi/enc2rad1)*enc2rad1
        if delta !=0:
            prepSP( abs(delta) , 100 , 1000 ,500000 , 1)
            setpar('motor.state1.SPdir' , delta>0)
            setpar('motor.state1.spNgo', 1)
    if motor == 0 or motor == 2:
        enc2rad2 = getsig('c2.enc2rad')
        delta = round(rel2/360*2*pi/enc2rad2)*enc2rad2
        if delta !=0:
            prepSP( abs(delta) , 100 , 1000 ,500000 , 2)
            setpar('motor.state2.SPdir' , delta>0)
            setpar('motor.state2.spNgo', 1)
        

def CL( cont = 2):
    setpar('c1.maxerror',0.5)
    setpar('c2.maxerror',0.5)
    
    # CONF1
    setLowpass( 1 , 0, 4000, 0.7 )
    setLowpass( 1 , 1, 0, 0.3 )
    setLowpass( 1 , 2, 0, 0.3 )
    setLowpass( 1 , 3, 0, 0.3 )
    # setNotch( 1 , 1, 590, -20, 0.1 )
    
    setLowpass( 2 , 0, 4000, 0.7 )
    setLowpass( 2 , 1, 0, 0.3 )
    setLowpass( 2 , 2, 0, 0.3 )
    setLowpass( 2 , 3, 0, 0.3 )
    # setNotch( 2 , 1, 590, -20, 0.1 )
    
    if cont == 0:
        BW = 0
        alpha_i = 0 
        alpha_1 = 3
        alpha_2 = 3
    elif cont == 1:
        BW = 10
        alpha_i = 6
        alpha_1 = 3
        alpha_2 = 3
        setLowpass( 1 , 0, 150, 0.6 )
        setLowpass( 2 , 0, 150, 0.6 )     
        setpar('c1.maxerror',pi)
        setpar('c2.maxerror',pi)
    elif cont == 2:
        BW = 50
        alpha_i =6
        alpha_1 = 3
        alpha_2 = 10
    elif cont == 3:
        BW = 200
        alpha_i = 6
        alpha_1 = 3.5
        alpha_2 = 5
        setNotch( 1 , 1, 590, -20, 0.1 )
        setNotch( 2 , 1, 590, -20, 0.1 )
    elif cont == 4:
        BW = 200
        alpha_i = 2.5
        alpha_1 = 2.5
        alpha_2 = 20
    elif cont == 5:
        BW = 250
        alpha_i = 2.5
        alpha_1 = 2.5
        alpha_2 = 20
    else:
        BW = 50
        alpha_i = 6
        alpha_1 = 3
        alpha_2 = 10
        
    J = 3.6e-5
    gain_at_BW = J * (BW*2*pi)**2
    
    if BW > 0:
        if alpha_i > 0:
            Ki = BW*2*pi*Ts/alpha_i
        else:
            Ki = 0
        if alpha_1 > 0:
            Kd = alpha_1/(Ts*BW*2*pi)
        else:
            Kd = 0
        if alpha_2 > 0:
            lowpass_c = 1-np.exp(-BW*2*pi*alpha_2*Ts)
        else:
            lowpass_c =1
        Kp = gain_at_BW * abs(lowpass_c + np.exp(pi*BW*Ts*2j) - 1) / (abs(lowpass_c*((Ki*np.exp(pi*BW*Ts*2j)) / (np.exp(pi*BW*Ts*2j) - 1) + 1) * (Kd - Kd*np.exp(-pi*BW*Ts*2j) + 1) )) 
    else:
        Kp = 0
        Ki = 0
        Kd = 0
        lowpass_c = 1
    
    setpar( 'motor.conf1.Kp_prep' , Kp )
    setpar( 'motor.conf1.Ki_prep' , Ki )
    setpar( 'motor.conf1.Kd_prep' , Kd )
    setpar( 'motor.conf1.lowpass_c_prep' , lowpass_c )
    
    setpar( 'motor.conf1.Command' , 2 ) #Reset error
    
    setpar( 'motor.conf1.Command' , 1 ) #Activate controller
    
    # CONF2
    J = 4e-5
    gain_at_BW = J * (BW*2*pi)**2
    # alpha_i = 6
    # alpha_1 = 3
    # alpha_2 = 3
    
    if BW > 0:
        if alpha_i > 0:
            Ki = BW*2*pi*Ts/alpha_i
        else:
            Ki = 0
        if alpha_1 > 0:
            Kd = alpha_1/(Ts*BW*2*pi)
        else:
            Kd = 0
        if alpha_2 > 0:
            lowpass_c = 1-np.exp(-BW*2*pi*alpha_2*Ts)
        else:
            lowpass_c =1
        Kp = gain_at_BW * abs(lowpass_c + np.exp(pi*BW*Ts*2j) - 1) / (abs(lowpass_c*((Ki*np.exp(pi*BW*Ts*2j)) / (np.exp(pi*BW*Ts*2j) - 1) + 1) * (Kd - Kd*np.exp(-pi*BW*Ts*2j) + 1) )) 
    else:
        Kp = 0
        Ki = 0
        Kd = 0
        lowpass_c = 1
    
    setpar( 'motor.conf2.Kp_prep' , Kp )
    setpar( 'motor.conf2.Ki_prep' , Ki )
    setpar( 'motor.conf2.Kd_prep' , Kd )
    setpar( 'motor.conf2.lowpass_c_prep' , lowpass_c )
    
    setpar( 'motor.conf2.Command' , 1 ) #Activate controller     

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

z = ct.TransferFunction( [1, 0] , [1] , float(Ts))


# %%
setTrace(['s.sens1' , 's.sens2' , 's.sens3' , 's.sens4' , 's.sensBus' , 's.sensBus2' ])
df = trace(0.1)

plt.figure(1)
fftpsd( df['motor.state.sens1'])
plt.figure(2)
fftpsd( df['motor.state.sens2'])
plt.figure(3)
fftpsd( df['motor.state.sens3'])
plt.figure(4)
fftpsd( df['motor.state.sens4'])


# %%
setTrace([ 'motor.state.encoderPos1','motor.state.encoderPos2','motor.state.IndexFound1','motor.state.IndexFound2'])
df = trace(1);

df.plot()

# %%
getsig('motor.state.firsterror')

# %%
setpar('motor.conf1.clipMethod', 0)
setpar('motor.conf1.clipMethod', 1)

# %%
setpar('motor.conf1.ridethewave', 1)
setpar('motor.conf2.ridethewave', 1)
time.sleep(0.5)

setpar('motor.conf1.commutationoffset', 0)
setpar('motor.conf2.commutationoffset', 0)

setpar('motor.state1.Valpha_offset', 1)
setpar('motor.state2.Valpha_offset', 1)

time.sleep(1)

offset1 = getsig('motor.state1.thetaPark_enc')
offset2 = getsig('motor.state2.thetaPark_enc')

setpar('motor.state1.Valpha_offset', 0)
setpar('motor.state2.Valpha_offset', 0)
setpar('motor.conf1.commutationoffset', -offset1)
setpar('motor.conf2.commutationoffset', -offset2)

# Servo motor Wittenstein cyber MSSI 055G
Lq = 250e-6
Ld = 210e-6
R = 0.33
setpar('motor.conf1.Lambda_m', 0.005405)
setpar('motor.conf1.N_pp',  4)
setpar('motor.conf1.Lq', Lq)
setpar('motor.conf1.Ld', Ld)
setpar('motor.state1.R', R)
setpar('motor.conf1.useIlowpass', 0)

f_bw = 1.5e3
f_lp = f_bw*3
f_lp_2nd = f_bw*6
f_lp_2nd_damp = 0.5

setpar('motor.conf1.Kp_iq', Lq * f_bw * 2 * pi)  # Current loop Kp
setpar('motor.conf1.Ki_iq', R/Lq)  # Current loop Ki
setpar('motor.conf1.Kp_id', Ld * f_bw * 2 * pi)  # Current loop Kp
setpar('motor.conf1.Ki_id', R/Ld)  # Current loop Ki
lowpass_c = 1-np.exp(-f_lp*2*pi*Ts)
setpar( 'motor.conf1.lowpass_Vd_c' , lowpass_c )
setpar( 'motor.conf1.lowpass_Vq_c' , lowpass_c )

setpar('motor.conf1.I_max', 15)
setpar('motor.conf1.maxDutyCycle', 0.99)

setpar('motor.conf1.enc_transmission' , 1)

# Servo motor Wittenstein cyber MSSI 055G  CONF2
Lq = 250e-6
Ld = 210e-6
R = 0.33
setpar('motor.conf2.Lambda_m', 0.005405)
setpar('motor.conf2.N_pp',  4)
setpar('motor.conf2.Lq', Lq)
setpar('motor.conf2.Ld', Ld)
setpar('motor.state2.R', R)
setpar('motor.conf2.useIlowpass', 0)

setpar('motor.conf2.Kp_iq', Lq * f_bw * 2 * pi)  # Current loop Kp
setpar('motor.conf2.Ki_iq', R/Lq)  # Current loop Ki
setpar('motor.conf2.Kp_id', Ld * f_bw * 2 * pi)  # Current loop Kp
setpar('motor.conf2.Ki_id', R/Ld)  # Current loop Ki
lowpass_c = 1-np.exp(-f_lp*2*pi*Ts)
setpar( 'motor.conf2.lowpass_Vd_c' , lowpass_c )
setpar( 'motor.conf2.lowpass_Vq_c' , lowpass_c )

setpar('motor.conf2.I_max', 15)
setpar('motor.conf2.maxDutyCycle', 0.99)

setpar('motor.conf2.enc_transmission' , 1)

setLowpass( 1 , 4, f_lp_2nd, f_lp_2nd_damp )
setLowpass( 1 , 5, f_lp_2nd, f_lp_2nd_damp )
setLowpass( 2 , 4, f_lp_2nd, f_lp_2nd_damp )
setLowpass( 2 , 5, f_lp_2nd, f_lp_2nd_damp )


# %% Trampa 160KV
#Rotate by hand first
Ld = 19e-6
Lq = 34e-6
R = 0.035
setpar('motor.conf1.Lambda_m', 0.005405)
setpar('motor.conf1.N_pp',  7)
setpar('motor.conf1.Lq', Lq)
setpar('motor.conf1.Ld', Ld)
setpar('motor.state1.R', R)
setpar('motor.conf.useIlowpass', 0)

setpar('motor.conf1.commutationoffset', 0)

setpar('motor.state1.Valpha_offset', 0.2)
time.sleep(0.5)
Valpha1 = getsig('motor.state1.Valpha_offset')
Ialpha1 = getsig('motor.state1.Ialpha')
Ibeta1 = getsig('motor.state1.Ibeta')
Ia1 = getsig('motor.state1.ia')
bus1 = getsig('motor.state.sensBus_lp')

setpar('motor.state1.Valpha_offset', 0.5)

time.sleep(1)
Valpha2 = getsig('motor.state1.Valpha_offset')
Va = getsig('motor.state1.Va')
Vb = getsig('motor.state1.Vb')
Ialpha2 = getsig('motor.state1.Ialpha')
Ibeta2 = getsig('motor.state1.Ibeta')
bus2 = getsig('motor.state.sensBus')
Ia2 = getsig('motor.state1.ia')

R = (Valpha2 - Valpha1) / (Ialpha2 - Ialpha1)

offset = getsig('motor.state1.thetaPark_enc')

setpar('motor.state1.Valpha_offset', 0)
setpar('motor.conf1.commutationoffset', -offset)

thetaPark = getsig('motor.state1.thetaPark')

# %%
# Trampa 160KV
f_bw = 2e3

setpar('motor.conf1.Kp_iq', Lq * f_bw * 2 * pi)  # Current loop Kp
setpar('motor.conf1.Ki_iq', R/Lq)  # Current loop Ki
setpar('motor.conf1.Kp_id', Ld * f_bw * 2 * pi)  # Current loop Kp
setpar('motor.conf1.Ki_id', R/Ld)  # Current loop Ki

setpar('motor.conf1.I_max', 20)
setpar('motor.conf1.maxDutyCycle', 0.99)


# %% Small inrunner surpass hobby
Ld = 6e-6
Lq = 6e-6
R = 0.053
setpar('motor.conf1.Lambda_m', 0.0006)
setpar('motor.conf1.N_pp',  2)
setpar('motor.conf1.Lq', Lq)
setpar('motor.conf1.Ld', Ld)
setpar('motor.state1.R', R)
setpar('motor.conf.useIlowpass', 0)

f_bw = 1e3

setpar('motor.conf1.Kp_iq', Lq * f_bw * 2 * pi)  # Current loop Kp
setpar('motor.conf1.Ki_iq', R/Lq)  # Current loop Ki
setpar('motor.conf1.Kp_id', Ld * f_bw * 2 * pi)  # Current loop Kp
setpar('motor.conf1.Ki_id', R/Ld)  # Current loop Ki

# setpar('motor.conf1.I_max' , 20)
# setpar('motor.conf1.maxDutyCycle' , 0.99)

# %% Iflight XING-E Pro 2207 1800KV
Ld = 7e-6
Lq = 7e-6
R = 0.06
kv = 1800
N_pp = 7
# fluxlinkage = 60 / (np.sqrt(3) * 2 * np.pi * kv * N_pp) 
fluxlinkage = 0.00046
setpar('motor.conf1.Lambda_m', fluxlinkage)
setpar('motor.conf1.N_pp',  N_pp)
setpar('motor.conf1.Lq', Lq)
setpar('motor.conf1.Ld', Ld)
setpar('motor.state1.R', R)
setpar('motor.conf.useIlowpass', 0)

f_bw = 2e3

setpar('motor.conf1.Kp_iq', Lq * f_bw * 2 * pi)  # Current loop Kp
setpar('motor.conf1.Ki_iq', R/Lq)  # Current loop Ki
setpar('motor.conf1.Kp_id', Ld * f_bw * 2 * pi)  # Current loop Kp
setpar('motor.conf1.Ki_id', R/Ld)  # Current loop Ki

setpar('motor.conf1.I_max' , 20)
setpar('motor.conf1.maxDutyCycle' , 0.99)
setpar('motor.conf1.anglechoice', 1)
# %%

setpar('motor.conf1.anglechoice', 1)

setpar('motor.state1.Iq_offset_SP', 10)

setpar('motor.state1.Iq_offset_SP', 1.5)
time.sleep(0.5)
setpar('motor.state1.Iq_offset_SP', 0)


# %% Enable hfi 1
setpar('s1.Id_offset_SP', 5)
time.sleep(0.5)
setpar('s1.Id_offset_SP', 0)


setpar('s1.hfi_use_lowpass', 1)

setpar('s1.hfi_method', 1)

Ki = 2000*2*pi
hfi_v = 6

setpar('s1.hfi_maxvel', 1e6)
setpar('s1.hfi_gain', Ki)
setpar('s1.hfi_gain_int2', 5*2*pi)
setpar('s1.hfi_V', hfi_v)
setpar('s1.hfi_on', 1)
setpar('c1.anglechoice', 3)




# %%
signals = setTrace(['motor.state1.ia', 'motor.state1.ib', 'motor.state1.ic'])
df = trace(0.1)
df.plot()

# %%
signals = setTrace(['motor.state1.Ialpha', 'motor.state1.Ibeta', 'motor.state.sensBus',
                   'motor.state.curtime', 'motor.state1.Va', 'motor.state1.Vb', 'motor.state1.Vc'])
df = trace(1)
df.plot()

# %%

signals = setTrace(['motor.state1.Id_SP', 'motor.state1.Iq_SP',  'motor.state1.Id_meas', 'motor.state1.Iq_meas',
                   'motor.state.sensBus', 'motor.state.curtime', 'motor.state1.Vq', 'motor.state1.Vd'])
df = trace(1)

df.plot()
# %%

signals = setTrace(['motor.state1.erpm', 'motor.state.sensBus', 'motor.state.curtime',
                   'motor.state1.Vq', 'motor.state1.Vd', 'motor.state1.BEMFa', 'motor.state1.BEMFb'])
df = trace(1)

df.plot()

# %%
setpar('motor.state1.Id_offset_SP', -5)

# %%
signals = setTrace(['motor.state1.erpm',  'motor.state1.Id_SP', 'motor.state1.Iq_SP',  'motor.state1.Id_meas',
                   'motor.state1.Iq_meas', 'motor.state.sensBus', 'motor.state1.Vq', 'motor.state1.Vd'])
df = trace(3)

df.plot()


# %%
signals = setTrace(['motor.state1.erpm',  'motor.state1.Id_SP', 'motor.state1.Iq_SP',  'motor.state1.Id_meas',
                   'motor.state1.Iq_meas', 'motor.state.sensBus', 'motor.state1.Vq', 'motor.state1.Vd'])

setpar('motor.state1.Iq_offset_SP', 0.5)
# %%
setpar('motor.state1.Iq_offset_SP', 6.6)
df = trace(3)
setpar('motor.state1.Iq_offset_SP', 3)
df2 = trace(1)
setpar('motor.state1.Iq_offset_SP', 1)
df3 = trace(1)
setpar('motor.state1.Iq_offset_SP', 0)
df.plot()

# %%
setpar('motor.state1.Iq_offset_SP', 5)

# %%
setpar('motor.state1.Iq_offset_SP', 3)
df2 = trace(1)
setpar('motor.state1.Iq_offset_SP', 1)
df3 = trace(1)
setpar('motor.state1.Iq_offset_SP', 0)
# %%
setpar('motor.state1.muziek_gain', 3)
setpar('motor.state2.muziek_gain', 3)

setpar('motor.state1.muziek_gain', 0)
setpar('motor.state2.muziek_gain', 0)


setpar('motor.state1.offsetVel' ,  70 )
setpar('motor.state2.offsetVel' ,  70 )

setpar('motor.state1.offsetVel' ,  0 )
setpar('motor.state2.offsetVel' ,  0 )


# %%
signals = setTrace(['motor.state1.thetaPark',  's1.delta_id' , 's1.delta_iq' ,
                   'motor.state1.thetaPark_enc', 'motor.state1.Iq_SP', 'motor.state1.Id_SP'])
                   # 'motor.state1.Id_meas','motor.state1.Iq_meas',
                   # 'motor.state1.Vd','motor.state1.Vq','motor.state1.I_bus','motor.state.sensBus'])


df = trace(2)
df.plot()

# %%
signals = setTrace([
                    'motor.state1.ia','motor.state1.ib' , 
                    'motor.state2.ia','motor.state2.ib' ])
                    # 'motor.state.sens1' ,'motor.state.sens2' ,'motor.state.sens3' ,'motor.state.sens4'])
                   # 'motor.state1.Id_meas','motor.state1.Iq_meas',
                   # 'motor.state1.Vd','motor.state1.Vq','motor.state1.I_bus','motor.state.sensBus'])

df = trace(1)
df.plot()



# %%
# signals = setTrace( ['motor1.curtime',
#                      'motor1.state.x' ,
#                      'motor1.state.current' ,
#                      'motor1.mot_conf.banaan_x' ,
#                      'motor2.mot_conf.banaan_x'])


signals = setTrace(signames[0:12])


signals = setTrace(['motor.state.curtime', 'motor.state.sens1',
                   'motor.state.sens2', 'motor.state.sens1_lp', 'motor.state.sens2_lp'])


signals = setTrace(['motor.state1.dist', 'motor.state1.noisebit'])


setpar('motor.state1.SPdir', 1)
signals = setTrace(['motor.state1.rmech'])
setpar('motor.state1.spNgo', 5)
df = trace(0.5)
df.plot()

plt.plot(np.diff(df.index))


setpar('motor.state1.x', [1])
setpar('motor.state2.x', [5])
print(getsig('motor.state1.x'))
print(getsig('motor.state2.x'))


setpar('motor.state1.current', [1, 4, 24, 5])
setpar('motor.state2.current', [2, 4, 24, 5])
print(getsig('motor.state1.current'))
print(getsig('motor.state2.current'))


setparpart('motor.state1.current', [5, 0], 2)
print(getsig('motor.state1.current'))

print(getsigpart('motor.state1.current', 1, 3))

# %% Current loop axis 1  
setpar('motor.conf1.anglechoice', 0)

NdownsamplePRBS = 2
N = 30*NdownsamplePRBS*2047
signals = ['motor.state1.Id_meas', 'motor.state1.Iq_meas',
           'motor.state1.dist', 'motor.state1.Vq', 'motor.state1.Vd']
setTrace(signals )


gain = 10
setpar('motor.state1.Vq_distgain', 1)
setpar('motor.state1.Vd_distgain', 1)
setpar('motor.state1.Iq_distgain', 0)
setpar('motor.state1.Id_distgain', 0)
setpar('motor.state1.mechdistgain', 0)

setpar('motor.conf.NdownsamplePRBS', NdownsamplePRBS)  # Downsampling
setpar('motor.state1.distval', gain)  # disturbance amplitude
setpar('motor.state1.distoff', 0)  # disturbance offset
df = trace(N * Ts)
setpar('motor.state1.distval', 0)  # disturbance amplitude
setpar('motor.state1.distoff', 0)  # disturbance offset
setpar('motor.conf.NdownsamplePRBS', 1)  # Downsampling

dfout = getFFTdf(df, NdownsamplePRBS , 10*2047 )
f = dfout.index.values
Pd = dfout['motor.state1.Id_meas'].values / dfout['motor.state1.Vd'].values
Pq = dfout['motor.state1.Iq_meas'].values / dfout['motor.state1.Vq'].values
Sd = dfout['motor.state1.Vd'].values
Sq = dfout['motor.state1.Vq'].values
# %% Current loop axis 2
setpar('motor.conf2.anglechoice', 0)

NdownsamplePRBS = 2
N = 30*NdownsamplePRBS*2047
signals = ['motor.state2.Id_meas', 'motor.state2.Iq_meas',
           'motor.state2.dist', 'motor.state2.Vq', 'motor.state2.Vd']
setTrace(signals )


gain = 10
setpar('motor.state2.Vq_distgain', 1)
setpar('motor.state2.Vd_distgain', 1)
setpar('motor.state2.Iq_distgain', 0)
setpar('motor.state2.Id_distgain', 0)
setpar('motor.state2.mechdistgain', 0)

setpar('motor.conf.NdownsamplePRBS', NdownsamplePRBS)  # Downsampling
setpar('motor.state2.distval', gain)  # disturbance amplitude
setpar('motor.state2.distoff', 0)  # disturbance offset
df = trace(N * Ts)
setpar('motor.state2.distval', 0)  # disturbance amplitude
setpar('motor.state2.distoff', 0)  # disturbance offset
setpar('motor.conf.NdownsamplePRBS', 1)  # Downsampling

dfout = getFFTdf(df, NdownsamplePRBS , 10*2047 )
f = dfout.index.values
Pd = dfout['motor.state2.Id_meas'].values / dfout['motor.state2.Vd'].values
Pq = dfout['motor.state2.Iq_meas'].values / dfout['motor.state2.Vq'].values
Sd = dfout['motor.state2.Vd'].values
Sq = dfout['motor.state2.Vq'].values

# %% Current loop plots
plt.figure(1)
bode( Pd , f, 'Measured D axis plant')
bode( Pq , f, 'Measured Q axis plant')

plt.figure(2)
bode( 1 / Sd - 1 , f, 'Open loop D')
bode( 1 / Sq - 1 , f, 'Open loop Q')

plt.figure(6)
bode( (1 / Sd - 1)/Pd , f, 'Controller D')
bode( (1 / Sq - 1)/Pq , f, 'Controller Q')


plt.figure(3)
nyquist( 1 / Sq - 1 , f, 'Open loop D')
nyquist( 1 / Sd - 1 , f, 'Open loop Q')

plt.figure(4)
plt.plot(
    f, np.abs(1/(Pd * f * 2 * np.pi)) * 1e6)
plt.grid()
plt.xlim([1e3, 10e3])
plt.ylim([ 100 , 300])
plt.title('Ld [uH]')


plt.figure(5)
plt.plot(
    f, np.abs(1/(Pq * f * 2 * np.pi)) * 1e6)
plt.grid()
plt.xlim([1e3, 10e3])
plt.ylim([ 100 , 300])
plt.title('Lq [uH]')


# %% Open loop identification
NdownsamplePRBS = 10
N = 15*NdownsamplePRBS*2047

signals = ['motor.state1.Id_meas', 'motor.state1.Iq_meas',
           'motor.state1.dist', 'motor.state1.emech' , 'motor.state1.mechcontout' , 'motor.state1.Iq_SP']
setTrace(signals )


gain = 0.05
setpar('motor.state1.Vq_distgain', 0)
setpar('motor.state1.Vd_distgain', 0)
setpar('motor.state1.Iq_distgain', 0)
setpar('motor.state1.Id_distgain', 0)
setpar('motor.state1.mechdistgain', 1)

setpar('motor.conf.NdownsamplePRBS', NdownsamplePRBS)  # Downsampling
setpar('motor.state1.distval', gain)  # disturbance amplitude
setpar('motor.state1.distoff', 0)  # disturbance offset
df = trace(N * Ts)
setpar('motor.state1.distval', 0)  # disturbance amplitude
setpar('motor.state1.distoff', 0)  # disturbance offset
setpar('motor.conf.NdownsamplePRBS', 1)  # Downsampling


dfout = getFFTdf(df, NdownsamplePRBS , 10*2047 )
f = dfout.index.values

plt.figure(1)
bode( -dfout['motor.state1.emech'] , f, 'Measured plant')


plt.figure(2)
bode( 1/(-dfout['motor.state1.emech'] * (2*pi*f)**2) , f, 'Measured inertia')



# %% set Lowpass and Notches 1
setLowpass( 1 , 0, 1200, 0.7 )
setNotch( 1 , 1, 200, 0, 0.1 )
setNotch( 1 , 2, 590, -20, 0.1 )
setNotch( 1 , 3, 700, 0, 0.1 )

# %% Closed loop identification 1
setpar('motor.state1.offsetVel' ,  100 )

NdownsamplePRBS = 10
Naver = 10
Nwait = 5
N = (Nwait+Naver)*NdownsamplePRBS*2047

signals = ['motor.state1.Id_meas', 'motor.state1.Iq_meas',
           'motor.state1.dist', 'motor.state1.emech' , 'motor.state1.mechcontout' , 'motor.state1.Iq_SP']
setTrace(signals )


gain = 0.10
setpar('motor.state1.Vq_distgain', 0)
setpar('motor.state1.Vd_distgain', 0)
setpar('motor.state1.Iq_distgain', 0)
setpar('motor.state1.Id_distgain', 0)
setpar('motor.state1.mechdistgain', 1)

setpar('motor.conf.NdownsamplePRBS', NdownsamplePRBS)  # Downsampling
setpar('motor.state1.distval', gain)  # disturbance amplitude
setpar('motor.state1.distoff', 0)  # disturbance offset

df = trace(N * Ts)
setpar('motor.state1.distval', 0)  # disturbance amplitude
setpar('motor.state1.distoff', 0)  # disturbance offset
setpar('motor.conf.NdownsamplePRBS', 1)  # Downsampling

setpar('motor.state1.offsetVel' ,  0 )

dfout = getFFTdf(df, NdownsamplePRBS , Nwait*NdownsamplePRBS*2047 )
f = dfout.index.values

S = dfout['motor.state1.mechcontout']
PS = -dfout['motor.state1.emech']
P = PS / S 
OL = 1/S - 1

plt.figure(1)
bode( OL , f, 'Measured Open Loop')

plt.figure(2)
nyquist( OL , f, 'Measured Open Loop')

plt.figure(3)
bode( P , f, 'Measured Plant')

plt.figure(4)
bode( 1/(P * (2*pi*f)**2) , f, 'Measured inertia')

plt.figure(5)
bode( OL / P , f, 'Measured C')

# %% set Lowpass and Notches 2
setLowpass( 2 , 0, 1200, 0.7 )
setNotch( 2 , 1, 200, -20, 0.1 )
setNotch( 2 , 2, 590, -20, 0.1 )
setNotch( 2 , 3, 700, -20, 0.1 )

# %% Closed loop identification motor 2
setpar('motor.state2.offsetVel' ,  100 )

NdownsamplePRBS = 10
Naver = 10
Nwait = 5
N = (Nwait+Naver)*NdownsamplePRBS*2047

signals = ['motor.state2.Id_meas', 'motor.state2.Iq_meas',
           'motor.state2.dist', 'motor.state2.emech' , 'motor.state2.mechcontout' , 'motor.state2.Iq_SP']
setTrace(signals )


gain = 0.1
setpar('motor.state2.Vq_distgain', 0)
setpar('motor.state2.Vd_distgain', 0)
setpar('motor.state2.Iq_distgain', 0)
setpar('motor.state2.Id_distgain', 0)
setpar('motor.state2.mechdistgain', 1)

setpar('motor.conf.NdownsamplePRBS', NdownsamplePRBS)  # Downsampling
setpar('motor.state2.distval', gain)  # disturbance amplitude
setpar('motor.state2.distoff', 0)  # disturbance offset
df = trace(N * Ts)
setpar('motor.state2.distval', 0)  # disturbance amplitude
setpar('motor.state2.distoff', 0)  # disturbance offset
setpar('motor.conf.NdownsamplePRBS', 1)  # Downsampling

setpar('motor.state2.offsetVel' ,  0 )

dfout = getFFTdf(df, NdownsamplePRBS , Nwait*NdownsamplePRBS*2047 )
f = dfout.index.values

S = dfout['motor.state2.mechcontout']
PS = -dfout['motor.state2.emech']
P = PS / S 
OL = 1/S - 1

plt.figure(1)
bode( OL , f, 'Measured Open Loop')

plt.figure(2)
nyquist( OL , f, 'Measured Open Loop')

plt.figure(3)
bode( P , f, 'Measured Plant')

plt.figure(4)
bode( 1/(P * (2*pi*f)**2) , f, 'Measured inertia')

plt.figure(5)
bode( OL / P , f, 'Measured C')


# %% Detect Lambda_m
signals = ['motor.state1.BEMFa', 'motor.state1.BEMFb', 'motor.state.sensBus',
           'motor.state1.Iq_meas', 'motor.state1.Id_meas', 'motor.state1.Id_e', 'motor.state1.Iq_e']
setTrace(signals)

# setpar('motor.conf1.Lambda_m', 1)

vmax = 20000  # ERPM

setpar('motor.conf1.anglechoice', 99)
setpar('motor.state1.Id_offset_SP', 2)
setpar('motor.state1.i_vector_acc', 500)
setpar('motor.state1.i_vector_radpers', vmax / 60 * 2*pi)

while abs(getsig('motor.state1.i_vector_radpers_act')) < 0.99 * vmax / 60 * 2*pi:
    a = 1

setpar('motor.state1.Id_offset_SP', 0)


df = trace(1.5)


# lambda_m = np.mean((np.max(data[1][:,0:2] , axis=0) - np.min(data[1][:,0:2] , axis=0))/2)

setpar('motor.state1.i_vector_radpers', 0)
setpar('motor.state1.i_vector_acc', 1e8)

df.filter(regex='BEMF').plot()

# %% Debug HFI
setpar('s1.hfi_method', 1)

Ki = 1000*2*pi
hfi_v = 12

setpar('s1.hfi_maxvel', 1e6)
setpar('s1.hfi_gain', Ki)
setpar('s1.hfi_gain_int2', 5*2*pi)
setpar('s1.hfi_V', hfi_v)
setpar('s1.hfi_on', 1)

setpar('motor.conf1.anglechoice', 101)
setpar('motor.state1.Iq_offset_SP',5)

time.sleep(2)

# setpar('motor.state1.thetaPark', 0)

# vmax = 500  # ERPM
# setpar('motor.conf1.anglechoice', 99)
# setpar('motor.state1.i_vector_acc', 100000)
# setpar('motor.state1.i_vector_radpers', vmax / 60 * 2*pi)

signals = ['motor.state1.BEMFa', 'motor.state1.BEMFb', 'motor.state.sensBus', 'motor.state1.Iq_meas', 'motor.state1.Id_meas',
           'motor.state1.Id_e', 'motor.state1.Iq_e', 's1.delta_id', 's1.delta_iq', 'motor.state1.thetaPark' , 'motor.state1.thetaPark_enc', 'motor.state.is_v7']
setTrace(signals)

df = trace(2)

setpar('s1.hfi_on', 0)
setpar('motor.conf1.anglechoice', 0)
setpar('motor.state1.Iq_offset_SP',0)

# df.filter(regex='delta').rolling(50).mean().plot()
# df.filter(regex='motor.state1.thetaPark_enc').plot()

id = df['s1.delta_id'].rolling(100).mean()
iq = df['s1.delta_iq'].rolling(100).mean()

idmean = df['s1.delta_id'].mean()
iqmean = df['s1.delta_iq'].mean()

print(idmean  )
print( iqmean )
# (df['motor.state1.thetaPark_enc'].rolling(100).mean()/2/np.pi).plot()

# (0.5*np.arctan2(iq, id-0.9)/2/pi+0.5).plot()

# df.plot()

h = plt.figure(2)
plt.plot( (df['motor.state1.thetaPark_enc'].rolling(1).mean()*360/2/np.pi) , df['s1.delta_id'].rolling(100).mean() )
plt.plot((df['motor.state1.thetaPark_enc'].rolling(1).mean()*360/2/np.pi) , df['s1.delta_iq'].rolling(100).mean()  )
# plt.plot( (df['motor.state1.thetaPark_enc'].rolling(100).mean()/2/np.pi) , ( df['s1.delta_id'].rolling(100).mean()  + df['s1.delta_iq'].rolling(100).mean() )  )
plt.gca().xaxis.set_major_locator(plt.MaxNLocator(5, steps=[1,2,4.5,5,9,10]))
plt.xlabel('Rotor angle [deg]')
plt.ylabel('delta current [A]')
plt.legend( ("Parallel" , "Orthogonal") , loc='best') 
plt.xlim( 0, 360)

plt.legend( ("Parallel beta 0 A" , "Orthogonal beta 0 A" , "Parallel beta -5 A" , "Orthogonal beta -5 A" , "Parallel beta 5 A" , "Orthogonal beta 5 A") , loc='best') 

# h = plt.figure(3)
# plt.plot( 360 + np.unwrap((df['motor.state1.thetaPark_enc'].rolling(1).mean()*360/2/np.pi) , 2*np.pi) - np.unwrap((df['motor.state1.thetaPark'].rolling(1).mean()*360/2/np.pi)  , 2*np.pi), df['s1.delta_id'].rolling(100).mean() )
# plt.plot( 360 + np.unwrap((df['motor.state1.thetaPark_enc'].rolling(1).mean()*360/2/np.pi) , 2*np.pi) - np.unwrap((df['motor.state1.thetaPark'].rolling(1).mean()*360/2/np.pi)  , 2*np.pi) , df['s1.delta_iq'].rolling(100).mean()  )
# # plt.plot( 360 + (df['motor.state1.thetaPark_enc'].rolling(1).mean()*360/2/np.pi)  - (df['motor.state1.thetaPark'].rolling(1).mean()*360/2/np.pi)  , df['s1.delta_id'].rolling(100).mean() )
# # plt.plot( 360 + (df['motor.state1.thetaPark_enc'].rolling(1).mean()*360/2/np.pi)  - (df['motor.state1.thetaPark'].rolling(1).mean()*360/2/np.pi)  , df['s1.delta_iq'].rolling(100).mean()  )
# ## plt.plot( (df['motor.state1.thetaPark_enc'].rolling(100).mean()/2/np.pi) , ( df['s1.delta_id'].rolling(100).mean()  + df['s1.delta_iq'].rolling(100).mean() )  )
# plt.gca().xaxis.set_major_locator(plt.MaxNLocator(5, steps=[1,2,4.5,5,9,10]))
# plt.xlabel('Rotor angle [deg]')
# plt.ylabel('delta current [A]')
# plt.legend( ("Parallel" , "Orthogonal") , loc='best') 
# # plt.xlim( -180, 180)
# plt.title('Rotating current vector, stationary rotor')

# offset = hfi_v * (1/Ld + 1/Lq) * Ts * 0.5

# plt.figure()
# plt.plot( (df['motor.state1.thetaPark_enc'].rolling(100).mean()/2/np.pi)   , df['s1.delta_id'].rolling(100).mean() - offset)
# plt.plot( (df['motor.state1.thetaPark_enc'].rolling(100).mean()/2/np.pi)  , df['s1.delta_iq'].rolling(100).mean()  )
# plt.plot( (df['motor.state1.thetaPark_enc'].rolling(100).mean()/2/np.pi)  +0.125/2 , ( df['s1.delta_id'].rolling(100).mean()  + df['s1.delta_iq'].rolling(100).mean() -offset ) / np.sqrt(2) )


# %% Debug HFI rotating vector
vmax = 4000  # ERPM

setpar('motor.conf1.anglechoice', 99)
setpar('motor.state1.i_vector_acc', 10)
setpar('motor.state1.i_vector_radpers', vmax / 60 * 2*pi)

# %% Calibrate HFI offsets
signals = ['motor.state1.BEMFa', 'motor.state1.BEMFb', 'motor.state.sensBus', 'motor.state1.Iq_meas', 'motor.state1.Id_meas',
           'motor.state1.Id_e', 'motor.state1.Iq_e', 's1.delta_id', 's1.delta_iq', 'motor.state1.thetaPark_enc']
setTrace(signals)


setpar('s1.hfi_method', 1)
Ki = 1000*2*pi

setpar('s1.hfi_maxvel', 1e6)
setpar('s1.hfi_gain', Ki)
setpar('s1.hfi_gain_int2', 5*2*pi)
setpar('s1.hfi_V', hfi_v)
setpar('s1.hfi_on', 1)
setpar( 's1.diq_compensation_on' , 0)

setpar('motor.conf1.anglechoice', 100)
setpar('motor.state1.Id_offset_SP', 10)

setpar('motor.state1.thetaPark', 1)
time.sleep(1)
setpar('motor.state1.thetaPark', 0.5)
time.sleep(1)
setpar('motor.state1.thetaPark', 0)
time.sleep(1)


for i in range(361):
    setparpart('s1.diq_compensation', 0.1, i)

did = []
diq = []
did_comp = []
diq_comp = []
THETA = np.linspace(0, 360, 361)
for theta in THETA:
    setpar('motor.state1.thetaPark', theta * 2*pi/360)
    time.sleep(0.05)
    df = trace(0.05)
    did.append(df['s1.delta_id'].mean())
    diq.append(df['s1.delta_iq'].mean())
    setpar( 's1.diq_compensation_on' , 1)
    time.sleep(0.05)
    df = trace(0.05)
    did_comp.append(df['s1.delta_id'].mean())
    diq_comp.append(df['s1.delta_iq'].mean())
    setpar( 's1.diq_compensation_on' , 0)


setpar('motor.state1.Id_offset_SP', 0)

compeffect = 10 * (np.array(diq_comp) - np.array(diq)) 

compneeded = -np.array(diq) / compeffect

plt.figure()
plt.plot( THETA , diq)
plt.plot( compneeded )

plt.plot( THETA , -0.1* np.array(diq) / (np.array(diq_comp) - np.array(diq)) )


plt.plot(THETA, diq - np.array(diq) / (np.array(diq_comp) - np.array(diq)))

(np.array(diq_comp) - np.array(diq))

A = np.fft.fft(did)
B = np.fft.fft(diq)
freq = np.fft.fftfreq(len(A))

B[abs(freq * 180) > 10] = 0
diq_correct = np.real(np.fft.ifft(B))

plt.figure()
plt.plot(freq * 180, abs(A))
plt.plot(freq * 180, abs(B))


plt.figure()
plt.plot(THETA, diq)
plt.plot(THETA, did)
plt.plot(THETA, diq_correct)

Ld_est = hfi_v / np.mean(did) * Ts

# %% Store HFI compensation data
for i in range(len(diq)):
    # setparpart('s1.diq_compensation', diq[i], i)
    # setparpart('s1.diq_compensation', diq_correct[i], i)
    setparpart('s1.diq_compensation', compneeded[i], i)

getsig('s1.diq_compensation')


# %%
setpar( 's1.diq_compensation_on' , 1)
setpar( 's1.diq_compensation_on' , 0)
# %%
setpar('motor.state1.Iq_offset_SP',5)
setpar('motor.state1.Iq_offset_SP', 0)

# %%
signals = ['motor.state1.thetaPark' , 'motor.state1.thetaPark_enc' ]
setTrace(signals)

setpar( 's1.diq_compensation_on' , 0)
df1 = trace( 0.1 )
setpar( 's1.diq_compensation_on' , 1)
df2 = trace( 0.1 )


df1.plot()
df2.plot()

#%%  
setpar( 's1.hfi_useforfeedback' , 1)

# setpar('I_max' , 20)

setpar( 'motor.conf1.Kp' , 0)
setpar( 'motor.state1.rmechoffset' , 0 )
setpar( 'motor.state1.rmechoffset' , -getsig( 'motor.state1.emech' ) )
getsig('motor.state1.emech'  )

#Trampa 185 KV low BW fun:
Kp =0.5
fBW = 20
alpha1 = 3
alpha2 = 3
fInt = 0
fLP = fBW * 7
setpar( 'motor.conf1.maxerror' , 1.5*np.pi )

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

#Good for Trampa 185 KV:
# Kp =10
# fBW = 30
# alpha1 = 3
# alpha2 = 3
# fInt = fBW / 6
# fLP = fBW * 7
# setpar( 'motor.conf1.maxerror' , 0.5 )

# Kp =30
# fBW = 40
# alpha1 = 3
# alpha2 = 3
# fInt = fBW / 8
# fLP = fBW * 7

setpar( 'motor.conf1.fBW' , fBW)
setpar( 'motor.conf1.alpha1' , alpha1)
setpar( 'motor.conf1.alpha2' , alpha2)
setpar( 'motor.conf1.fInt' , fInt)
setpar( 'motor.conf1.fLP' ,  fLP)

ser.write(b'C')

setpar( 'motor.conf1.Kp' , Kp)

# %%
setpar( 'motor.conf1.Kp' , 0)
setpar( 'motor.state1.rmechoffset' , 0 )
setpar( 'motor.state1.rmechoffset' , -getsig( 'motor.state1.emech' ) )
getsig('motor.state1.emech'  )

fBW = 100
setpar( 'motor.conf1.fBW' , fBW)
setpar( 'motor.conf1.alpha1' , 3)
setpar( 'motor.conf1.alpha2' , 4)
setpar( 'motor.conf1.fInt' , fBW / 6)
setpar( 'motor.conf1.fLP' ,  fBW * 8)


ser.write(b'C')

setpar( 'motor.conf1.Kp' , 23.3)

# %%
setpar( 's1.diq_compensation_on' , 1)
setpar( 's1.diq_compensation_on' , 0)

#%%  
Lq = 250e-6
Ld = 210e-6
R = 0.33
# Lq = 0
# Ld = 0
# R = 0
setpar('motor.conf1.Lq', Lq)
setpar('motor.conf1.Ld', Ld)
setpar('motor.state1.R', R)
#%%   Setpoint
setpar('motor.state1.rdelay' , 0)

setpar('motor.state1.Jload' , 3.25e-5)
setpar('motor.state1.velFF' , 0.00006)
setpar('motor.state2.Jload' , 3.25e-5)
setpar('motor.state2.velFF' , 0.00006)
# setpar('motor.state1.Jload' , 0)
# setpar('motor.state1.velFF' , 0)
# setpar('motor.state2.Jload' , 0)
# setpar('motor.state2.velFF' , 0)

# setTrace( ['motor.state.sensBus' , 'motor.state.sensBus2' , 'motor.state1.mechcontout' , 'motor.state1.T_FF_acc', 'motor.state1.T_FF_vel', 'motor.state.sensBus_lp','motor.state1.rmech' , 'motor.state1.vel', 'motor.state1.emech', 'motor.state2.emech', 'motor.state1.Vd', 'motor.state1.Vq' , 'motor.state1.Iq_SP', 'motor.state2.Iq_SP'])
setTrace( ['motor.state1.rmech' , 'motor.state2.rmech' , 'motor.state1.ymech' , 'motor.state2.ymech' ,'motor.state1.vel' , 'motor.state2.vel', 'motor.state1.emech' , 'motor.state2.emech'])
# setTrace( ['motor.state1.rmech' ,  'motor.state1.ymech' , 'motor.state1.emech' , 's1.Iq_SP' , 's1.Iq_meas' , 's1.Iq_e', 's1.Id_SP' , 's1.Id_meas' , 's1.Vd', 's1.Vq'])
# setTrace( ['s1.rmech' ,  's1.ymech' , 's1.emech' ])
# setTrace( ['s2.rmech' ,  's2.ymech' , 's2.emech' ])

p = 360
prepSP( p/360*2*pi , 250 , 6000 ,500000 , 1)
prepSP( p/360*2*pi , 250 , 6000 ,500000 , 2)

setpar('motor.conf2.Command' , 3)
df = trace(0.1)
while (getsig('motor.state2.spNgo') > 0 or getsig('motor.state2.REFstatus') > 0 ):
    bla = 1;
setpar('motor.conf2.Command' , 4)


plt.figure();ax = plt.gca()
df.plot(ax=ax) # draws to fig1 now

# df.plot()
# plt.figure(1)
# (df['motor.state1.rmech']/2/pi*20000).plot()
# (df['motor.state2.rmech']/2/pi*20000).plot()

#%%  
prepSP( 360/360*2*pi , 250 , 6000 ,2500000)
N = 1
setpar('motor.state1.SPdir' , 1)
setpar('motor.state1.spNgo' , N)

while (getsig('motor.state1.spNgo') > 0 or getsig('motor.state1.REFstatus') > 0 ):
    bla = 1;
setpar('motor.state1.SPdir' , 0)
setpar('motor.state1.spNgo' , N)

#%%  
p = 360
prepSP( p/360*2*pi , 250 , 6000 ,2500000 , 1)
prepSP( p/360*2*pi , 250 , 6000 ,2500000 , 2)
N = 2

setpar('motor.state1.SPdir' , 1)
setpar('motor.state2.SPdir' , 1)
setpar('motor.state1.spNgo' , N)
setpar('motor.state2.spNgo' , N)
while (getsig('motor.state2.spNgo') > 0 or getsig('motor.state2.REFstatus') > 0 ):
    bla = 1;
setpar('motor.state1.SPdir' , 0)
setpar('motor.state2.SPdir' , 0)
setpar('motor.state1.spNgo' , N)
setpar('motor.state2.spNgo' , N)


#%%  
p = 2*np.pi/3
v = 100
a = 100*np.pi
j = 100000*np.pi
Nsp = 1
delay = 0
[t1, t2, t3, jd] = prepSP(  p , v , a , j )

for i in range(6):
    setpar('motor.state1.SPdir' , 1)
    setpar('motor.state1.spNgo',Nsp)
    while (getsig('motor.state1.REFstatus') > 0):
        bla = 1;
    time.sleep(delay)

setpar('motor.state1.SPdir' , 0)
setpar('motor.state1.spNgo',6)

#%%  
p = 20*np.pi
v = 200
a = 100*np.pi
j = 100000*np.pi
Nsp = 1

# delay = 0.3
# for p in np.linspace( 2*np.pi/360*1 , 2*np.pi/360*10 , 10):
#     [t1, t2, t3, jd] = prepSP(  p , v , a , j )
#     setpar('motor.state1.SPdir' , 1)
#     setpar('motor.state1.spNgo',Nsp)
#     while (getsig('motor.state1.REFstatus') > 0):
#         bla = 1;
#     time.sleep(delay)
#     setpar('motor.state1.SPdir' , 0)
#     setpar('motor.state1.spNgo',Nsp)
#     while (getsig('motor.state1.REFstatus') > 0):
#         bla = 1;
#     time.sleep(delay)
    
delay = 0.00
for p in np.linspace( 0.1 , 2*pi/10 , 11):
    [t1, t2, t3, jd] = prepSP(  p , v , a , j )
    setpar('motor.state1.SPdir' , 1)
    setpar('motor.state1.spNgo',Nsp)
    while (getsig('motor.state1.REFstatus') > 0):
        bla = 1;
    time.sleep(delay)
    setpar('motor.state1.SPdir' , 0)
    setpar('motor.state1.spNgo',Nsp)
    while (getsig('motor.state1.REFstatus') > 0):
        bla = 1;
    time.sleep(delay)
    
    


#%% CONF1
setLowpass( 1 , 0, 4000, 0.7 )
setLowpass( 1 , 1, 0, 0.3 )
setLowpass( 1 , 2, 0, 0.3 )
setLowpass( 1 , 3, 0, 0.3 )
# setNotch( 1 , 1, 590, -20, 0.1 )

setLowpass( 2 , 0, 4000, 0.7 )
setLowpass( 2 , 1, 0, 0.3 )
setLowpass( 2 , 2, 0, 0.3 )
setLowpass( 2 , 3, 0, 0.3 )
# setNotch( 2 , 1, 590, -20, 0.1 )

BW = 40
alpha_i = 0 
alpha_1 = 3
alpha_2 = 3
setLowpass( 1 , 0, 200, 0.7 )
setLowpass( 2 , 0, 200, 0.7 )


# BW = 50
# alpha_i =6
# alpha_1 = 3
# alpha_2 = 10

# BW = 200
# alpha_i = 6
# alpha_1 = 3.5
# alpha_2 = 5
# setNotch( 1 , 1, 590, -20, 0.1 )
# setNotch( 2 , 1, 590, -20, 0.1 )

# BW = 200
# alpha_i = 2.5
# alpha_1 = 2.5
# alpha_2 = 20

# BW = 250
# alpha_i = 2.5
# alpha_1 = 2.5
# alpha_2 = 20


J = 3.6e-5
gain_at_BW = J * (BW*2*pi)**2

if BW > 0:
    if alpha_i > 0:
        Ki = BW*2*pi*Ts/alpha_i
    else:
        Ki = 0
    if alpha_1 > 0:
        Kd = alpha_1/(Ts*BW*2*pi)
    else:
        Kd = 0
    if alpha_2 > 0:
        lowpass_c = 1-np.exp(-BW*2*pi*alpha_2*Ts)
    else:
        lowpass_c =1
    Kp = gain_at_BW * abs(lowpass_c + np.exp(pi*BW*Ts*2j) - 1) / (abs(lowpass_c*((Ki*np.exp(pi*BW*Ts*2j)) / (np.exp(pi*BW*Ts*2j) - 1) + 1) * (Kd - Kd*np.exp(-pi*BW*Ts*2j) + 1) )) 
else:
    Kp = 0
    Ki = 0
    Kd = 0
    lowpass_c = 1

setpar( 'motor.conf1.Kp_prep' , Kp )
setpar( 'motor.conf1.Ki_prep' , Ki )
setpar( 'motor.conf1.Kd_prep' , Kd )
setpar( 'motor.conf1.lowpass_c_prep' , lowpass_c )

setpar( 'motor.conf1.Command' , 2 ) #Reset error

setpar( 'motor.conf1.Command' , 1 ) #Activate controller

# CONF2

J = 4e-5

gain_at_BW = J * (BW*2*pi)**2
# alpha_i = 6
# alpha_1 = 3
# alpha_2 = 3


if BW > 0:
    if alpha_i > 0:
        Ki = BW*2*pi*Ts/alpha_i
    else:
        Ki = 0
    if alpha_1 > 0:
        Kd = alpha_1/(Ts*BW*2*pi)
    else:
        Kd = 0
    if alpha_2 > 0:
        lowpass_c = 1-np.exp(-BW*2*pi*alpha_2*Ts)
    else:
        lowpass_c =1
    Kp = gain_at_BW * abs(lowpass_c + np.exp(pi*BW*Ts*2j) - 1) / (abs(lowpass_c*((Ki*np.exp(pi*BW*Ts*2j)) / (np.exp(pi*BW*Ts*2j) - 1) + 1) * (Kd - Kd*np.exp(-pi*BW*Ts*2j) + 1) )) 
else:
    Kp = 0
    Ki = 0
    Kd = 0
    lowpass_c = 1

setpar( 'motor.conf2.Kp_prep' , Kp )
setpar( 'motor.conf2.Ki_prep' , Ki )
setpar( 'motor.conf2.Kd_prep' , Kd )
setpar( 'motor.conf2.lowpass_c_prep' , lowpass_c )

setpar( 'motor.conf2.Command' , 1 ) #Activate controller

#%% 
setTrace( ['motor.state.sensBus'  ,'motor.state1.rmech' , 'motor.state1.vel', 'motor.state1.emech',  'motor.state1.ymech', 'motor.state1.Vd', 'motor.state1.Vq' , 'motor.state1.Iq_SP'])
# setTrace( ['motor.state1.emech' ])

p = 360
prepSP( p/360*2*pi , 1 , 10 ,2500000 , 1)

setpar('motor.state1.SPdir' , 1)
setpar('motor.state1.spNgo' , 1)
df = trace(2)
while (getsig('motor.state2.spNgo') > 0 or getsig('motor.state2.REFstatus') > 0 ):
    bla = 1;
setpar('motor.state1.SPdir' , 0)
setpar('motor.state2.SPdir' , 0)
setpar('motor.state1.spNgo' , 1)
setpar('motor.state2.spNgo' , 1)

df.plot()
#%% Controller tuning current loop

Lq = 250e-6
Ld = 210e-6
R = 0.33

f_bw = 3e3
f_lp = f_bw*3
C1 = discrete_lowpass( f_bw*6 , 0.5 )

Kp = Lq * f_bw * 2 * pi 
Ki = R/Lq * Ts

lowpass_c = 1-np.exp(-f_lp*2*pi*Ts)

Cz =  C1 * lowpass_c*z / (z-1+lowpass_c)  * ( 1 + Ki*z/(z-1) ) * Kp;

C = pd.Series( ct.freqresp( Cz , f * 2 *pi ).eval(f * 2 *pi) )
C.index = f
C.index.name = 'Frequency [Hz]'

plt.figure(1)
bode( Pq * C , f )

plt.figure(2)
nyquist( Pq * C , f )

plt.figure(3)
bode( Pq * C / (1 + Pq * C ) , f )
#%% Controller tuning


# BW = 25
# J = 3.6e-5
# gain_at_BW = J * (BW*2*pi)**2
# alpha_i = 6
# alpha_1 = 3
# alpha_2 = 3
# C1 = discrete_lowpass( 120, 0.6 )
# C2 = 1
# # C2 = #discrete_notch( 590, -20, 0.1) 
# C3 = 1
# C4 = 1

# BW = 200
# J = 3.6e-5
# gain_at_BW = J * (BW*2*pi)**2
# alpha_i = 6
# alpha_1 = 3.5
# alpha_2 = 5
# C1 = discrete_lowpass( BW*7, 0.6 )
# C2 = discrete_notch( 590, -20, 0.1) 
# C3 = 1
# C4 = 1

BW = 250
J = 3.6e-5
gain_at_BW = J * (BW*2*pi)**2
alpha_i = 2.5
alpha_1 = 2.5
alpha_2 = 20
C1 = discrete_lowpass( 4000, 0.6 )
C2 = 1
# C2 = #discrete_notch( 590, -20, 0.1) 
C3 = 1
C4 = 1

if BW > 0:
    if alpha_i > 0:
        Ki = BW*2*pi*Ts/alpha_i
    else:
        Ki = 0
    if alpha_1 > 0:
        Kd = alpha_1/(Ts*BW*2*pi)
    else:
        Kd = 0
    if alpha_2 > 0:
        lowpass_c = 1-np.exp(-BW*2*pi*alpha_2*Ts)
    else:
        lowpass_c =1
    Kp = gain_at_BW * abs(lowpass_c + np.exp(pi*BW*Ts*2j) - 1) / (abs(lowpass_c*((Ki*np.exp(pi*BW*Ts*2j)) / (np.exp(pi*BW*Ts*2j) - 1) + 1) * (Kd - Kd*np.exp(-pi*BW*Ts*2j) + 1) )) 
else:
    Kp = 0
    Ki = 0
    Kd = 0
    lowpass_c = 1

Cz =  C4 * C3 * C2 * C1 * lowpass_c*z / (z-1+lowpass_c)  * ( 1 + Ki*z/(z-1) ) *  (1 +  Kd * (z-1)/z ) * Kp;

C = pd.Series( ct.freqresp( Cz , f * 2 *pi ).eval(f * 2 *pi) )
C.index = f
C.index.name = 'Frequency [Hz]'

plt.figure(1)
bode( P * C , f )

plt.figure(2)
nyquist( P * C , f )


#%% 
pos([0,180])
#%% 
vel([100,-100])
#%% 
vel(0)


#%% 
pos_wait([180,0])
#%% 

v = 150
a = 7000
pos_wait([0,180] , vel=v , acc = a)
pos_wait([180,0], vel=v , acc = a)
pos_wait([-90,90], vel=v , acc = a)
pos_wait([-270,-90] , vel=v, acc = a)
pos_wait([0,180] , vel=v, acc = a)
pos_wait([180,0], vel=v, acc = a)
pos_wait([-90,90], vel=v, acc = a)
pos_wait([-270,-90] , vel=v, acc = a)
pos_wait([-180,0] , vel=v, acc = a)
pos_wait([0,-180], vel=v, acc = a)
pos_wait([90,-90], vel=v, acc = a)
pos_wait([-90,-270] , vel=v, acc = a)
pos_wait([180,0] , vel=v, acc = a)

