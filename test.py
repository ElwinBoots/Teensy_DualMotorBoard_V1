import matplotlib.pyplot as plt
import pandas as pd
import serial
import struct
import numpy as np
import time
import os 
os.chdir('C:\GIT\Teensy_DualMotorBoard_V1')
import make3

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


def setTrace(signals, downsample=1) :
    if isinstance(signals, str) or isinstance(signals, int):
        signals = [signals]

    ser.signalsnames = signals
    i = 0
    ser.signals = []
    signalsout = []
    ser.tracebytes = 0;
    for signal in ser.signalsnames:
        if isinstance(signal, str):
            signal = signames.index(signal)
            # signal = ser.signames.index( signal )
        # if not signal in ser.signals:
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


def setpar(signal, value):
    if isinstance(signal, list):
        signal = signal[0]
    if isinstance(signal, str):
        signal = signames.index(signal)
    data = np.array(value, sigtypes[signal]).tobytes()
    if(len(data) == sigbytes[signal]):
        ser.write(b'S' + struct.pack('I',  signal) + data)
    else:
        raise ValueError(
            f'Size in bytes expected: {sigbytes[signal]}, given: {len(data)}.')


def setparpart(signal, value, startlocation=0):
    if isinstance(signal, list):
        signal = signal[0]
    if isinstance(signal, str):
        signal = signames.index(signal)
    data = np.array(value, sigtypes[signal]).tobytes()
    length = len(data)
    if((length + startlocation * sigtypes[signal].itemsize) <= sigbytes[signal]):
        ser.write(b's' + struct.pack('I',  signal) + struct.pack('I',
                  startlocation) + struct.pack('I',  length) + data)
    else:
        raise ValueError(
            f'Max size in bytes expected: {sigbytes[signal]}, given: {len(data) + startlocation * sigtypes[signal].itemsize}.')


def getsig(signal):
    if isinstance(signal, list):
        signal = signal[0]
    if isinstance(signal, str):
        signal = signames.index(signal)
    ser.write(b'G' + struct.pack('I',  signal))
    buffer = ser.read(sigbytes[signal])
    arr = np.ndarray(1, dtype=dtypes[signal][1],  buffer=buffer)
    return arr[0]


def getsigpart(signal, startlocation, length):
    if isinstance(signal, list):
        signal = signal[0]
    if isinstance(signal, str):
        signal = signames.index(signal)
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
    dfout = dfout.div(dfout['motor.state1.dist'], axis=0)
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

def prepSP(  p , v , a , j  ):
    t1, t2, t3, jd = make3.make3( p , v , a , j , Ts );
    #double tstart, double t1, double t2, double t3, double p, double v_max, double a_max, double j_max
    ser.write( b'1' + struct.pack('f',  t1) + struct.pack('f',  t2) + struct.pack('f',  t3) + struct.pack('f',  p) + struct.pack('f',  v) + struct.pack('f',  a)  + struct.pack('f',  jd) )
    return t1, t2, t3, jd

def readall( ):
    a = []
    for signal in signames:
        a.append( getsig(signal))
    df = pd.DataFrame( a , index=signames ).T
    return df


# %%

getsig('motor.state.firsterror')

# %%
setpar('motor.conf1.clipMethod', 0)
setpar('motor.conf1.clipMethod', 1)

# %%
setpar('motor.conf1.ridethewave', 1)
time.sleep(1)

setpar('motor.conf1.commutationoffset', 0)


setpar('motor.state1.Valpha_offset', 0.5)
time.sleep(0.5)
Valpha1 = getsig('motor.state1.Valpha_offset')
Ialpha1 = getsig('motor.state1.Ialpha')
Ibeta1 = getsig('motor.state1.Ibeta')
Ia1 = getsig('motor.state1.ia')
bus1 = getsig('motor.state.sensBus_lp')

setpar('motor.state1.Valpha_offset', 1)

time.sleep(0.5)
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

# %% Servo motor Wittenstein cyber MSSI 055G
Lq = 2.6e-4
Ld = 2.2e-4
R = 0.33
setpar('motor.conf1.Lambda_m', 0.005405)
setpar('motor.conf1.N_pp',  4)
setpar('motor.conf1.Lq', Lq)
setpar('motor.conf1.Ld', Ld)
setpar('motor.state1.R', R)
setpar('motor.conf.useIlowpass', 0)

f_bw = 2e3

setpar('motor.conf1.Kp_iq', Lq * f_bw * 2 * pi)  # Current loop Kp
setpar('motor.conf1.Ki_iq', R/Lq)  # Current loop Ki
setpar('motor.conf1.Kp_id', Ld * f_bw * 2 * pi)  # Current loop Kp
setpar('motor.conf1.Ki_id', R/Ld)  # Current loop Ki

setpar('motor.conf1.I_max', 15)
setpar('motor.conf1.maxDutyCycle', 0.99)

setpar('motor.conf1.enc_transmission' , 1)

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


# %% Enable hfi
setpar('motor.state1.Id_offset_SP', 5)
time.sleep(0.5)
setpar('motor.state1.Id_offset_SP', 0)


setpar('motor.hfi1.hfi_use_lowpass', 1)

setpar('motor.hfi1.hfi_method', 1)

Ki = 1000*2*pi
hfi_v = 3

setpar('motor.hfi1.hfi_maxvel', 1e6)
setpar('motor.hfi1.hfi_gain', Ki)
setpar('motor.hfi1.hfi_gain_int2', 5*2*pi)
setpar('motor.hfi1.hfi_V', hfi_v)
setpar('motor.hfi1.hfi_on', 1)
setpar('motor.conf1.anglechoice', 3)

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
setpar('motor.state1.muziek_gain', 8)

setpar('motor.state1.muziek_gain', 0)





# %%
signals = setTrace(['motor.state1.thetaPark',  'motor.hfi1.delta_id' , 'motor.hfi1.delta_iq' ,
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


setpar('motor.setpoint.SPdir', 1)
signals = setTrace(['motor.state1.rmech'])
setpar('motor.setpoint.spNgo', 5)
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

# %%
setpar('motor.conf1.anglechoice', 0)

NdownsamplePRBS = 1
N = 30*NdownsamplePRBS*2047
signals = ['motor.state1.Id_meas', 'motor.state1.Iq_meas',
           'motor.state1.dist', 'motor.state1.Vq', 'motor.state1.Vd']
setTrace(signals )


gain = 1
setpar('motor.state1.Vq_distgain', 1)
setpar('motor.state1.Vd_distgain', 1)
setpar('motor.state1.Iq_distgain', 0)
setpar('motor.state1.Id_distgain', 0)
setpar('motor.state1.mechdistgain', 0)

setpar('motor.conf1.NdownsamplePRBS', NdownsamplePRBS)  # Downsampling
setpar('motor.state1.distval', gain)  # disturbance amplitude
setpar('motor.state1.distoff', 0)  # disturbance offset
df = trace(N * Ts)
setpar('motor.state1.distval', 0)  # disturbance amplitude
setpar('motor.state1.distoff', 0)  # disturbance offset
setpar('motor.conf1.NdownsamplePRBS', 1)  # Downsampling

# %%

dfout = getFFTdf(df, NdownsamplePRBS , 10*2047 )
f = dfout.index.values

Pd = dfout['motor.state1.Id_meas'].values / dfout['motor.state1.Vd'].values
Pq = dfout['motor.state1.Iq_meas'].values / dfout['motor.state1.Vq'].values

plt.figure(1)
bode( Pd , f, 'Measured D axis plant')
bode( Pq , f, 'Measured Q axis plant')

plt.figure(2)
bode( dfout['motor.state1.Vq'].values  , f, 'Open loop')
bode( dfout['motor.state1.Vd'].values  , f, 'Open loop')

plt.figure(3)
bode( 1 / dfout['motor.state1.Vq'].values - 1 , f, 'Open loop')
bode( 1 / dfout['motor.state1.Vd'].values - 1 , f, 'Open loop')




plt.figure(4)
plt.plot(
    f, np.abs(1/(Pq * f * 2 * np.pi)) * 1e6)
plt.grid(which='major')
plt.xlim([1e3, 10e3])
plt.ylim([ 100 , 300])
plt.title('Lq [uH]')

plt.figure(5)
plt.plot(
    f, np.abs(1/(Pd * f * 2 * np.pi)) * 1e6)
plt.grid(which='major')
plt.xlim([1e3, 10e3])
plt.ylim([ 100 , 300])
plt.title('Ld [uH]')


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

setpar('motor.conf1.NdownsamplePRBS', NdownsamplePRBS)  # Downsampling
setpar('motor.state1.distval', gain)  # disturbance amplitude
setpar('motor.state1.distoff', 0)  # disturbance offset
df = trace(N * Ts)
setpar('motor.state1.distval', 0)  # disturbance amplitude
setpar('motor.state1.distoff', 0)  # disturbance offset
setpar('motor.conf1.NdownsamplePRBS', 1)  # Downsampling


dfout = getFFTdf(df, NdownsamplePRBS , 10*2047 )
f = dfout.index.values

plt.figure(1)
bode( -dfout['motor.state1.emech'] , f, 'Measured D axis plant')


# %% Closed loop identification
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

setpar('motor.conf1.NdownsamplePRBS', NdownsamplePRBS)  # Downsampling
setpar('motor.state1.distval', gain)  # disturbance amplitude
setpar('motor.state1.distoff', 0)  # disturbance offset
df = trace(N * Ts)
setpar('motor.state1.distval', 0)  # disturbance amplitude
setpar('motor.state1.distoff', 0)  # disturbance offset
setpar('motor.conf1.NdownsamplePRBS', 1)  # Downsampling


dfout = getFFTdf(df, NdownsamplePRBS , 10*2047 )
f = dfout.index.values

S = dfout['motor.state1.mechcontout']
plt.figure(1)
bode( 1/S - 1 , f, 'Measured D axis plant')


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
setpar('motor.hfi1.hfi_method', 1)

Ki = 1000*2*pi
hfi_v = 12

setpar('motor.hfi1.hfi_maxvel', 1e6)
setpar('motor.hfi1.hfi_gain', Ki)
setpar('motor.hfi1.hfi_gain_int2', 5*2*pi)
setpar('motor.hfi1.hfi_V', hfi_v)
setpar('motor.hfi1.hfi_on', 1)

setpar('motor.conf1.anglechoice', 101)
setpar('motor.state1.Iq_offset_SP',5)

time.sleep(2)

# setpar('motor.state1.thetaPark', 0)

# vmax = 500  # ERPM
# setpar('motor.conf1.anglechoice', 99)
# setpar('motor.state1.i_vector_acc', 100000)
# setpar('motor.state1.i_vector_radpers', vmax / 60 * 2*pi)

signals = ['motor.state1.BEMFa', 'motor.state1.BEMFb', 'motor.state.sensBus', 'motor.state1.Iq_meas', 'motor.state1.Id_meas',
           'motor.state1.Id_e', 'motor.state1.Iq_e', 'motor.hfi1.delta_id', 'motor.hfi1.delta_iq', 'motor.state1.thetaPark' , 'motor.state1.thetaPark_enc', 'motor.state.is_v7']
setTrace(signals)

df = trace(2)

setpar('motor.hfi1.hfi_on', 0)
setpar('motor.conf1.anglechoice', 0)
setpar('motor.state1.Iq_offset_SP',0)

# df.filter(regex='delta').rolling(50).mean().plot()
# df.filter(regex='motor.state1.thetaPark_enc').plot()

id = df['motor.hfi1.delta_id'].rolling(100).mean()
iq = df['motor.hfi1.delta_iq'].rolling(100).mean()

idmean = df['motor.hfi1.delta_id'].mean()
iqmean = df['motor.hfi1.delta_iq'].mean()

print(idmean  )
print( iqmean )
# (df['motor.state1.thetaPark_enc'].rolling(100).mean()/2/np.pi).plot()

# (0.5*np.arctan2(iq, id-0.9)/2/pi+0.5).plot()

# df.plot()

h = plt.figure(2)
plt.plot( (df['motor.state1.thetaPark_enc'].rolling(1).mean()*360/2/np.pi) , df['motor.hfi1.delta_id'].rolling(100).mean() )
plt.plot((df['motor.state1.thetaPark_enc'].rolling(1).mean()*360/2/np.pi) , df['motor.hfi1.delta_iq'].rolling(100).mean()  )
# plt.plot( (df['motor.state1.thetaPark_enc'].rolling(100).mean()/2/np.pi) , ( df['motor.hfi1.delta_id'].rolling(100).mean()  + df['motor.hfi1.delta_iq'].rolling(100).mean() )  )
plt.gca().xaxis.set_major_locator(plt.MaxNLocator(5, steps=[1,2,4.5,5,9,10]))
plt.xlabel('Rotor angle [deg]')
plt.ylabel('delta current [A]')
plt.legend( ("Parallel" , "Orthogonal") , loc='best') 
plt.xlim( 0, 360)

plt.legend( ("Parallel beta 0 A" , "Orthogonal beta 0 A" , "Parallel beta -5 A" , "Orthogonal beta -5 A" , "Parallel beta 5 A" , "Orthogonal beta 5 A") , loc='best') 

# h = plt.figure(3)
# plt.plot( 360 + np.unwrap((df['motor.state1.thetaPark_enc'].rolling(1).mean()*360/2/np.pi) , 2*np.pi) - np.unwrap((df['motor.state1.thetaPark'].rolling(1).mean()*360/2/np.pi)  , 2*np.pi), df['motor.hfi1.delta_id'].rolling(100).mean() )
# plt.plot( 360 + np.unwrap((df['motor.state1.thetaPark_enc'].rolling(1).mean()*360/2/np.pi) , 2*np.pi) - np.unwrap((df['motor.state1.thetaPark'].rolling(1).mean()*360/2/np.pi)  , 2*np.pi) , df['motor.hfi1.delta_iq'].rolling(100).mean()  )
# # plt.plot( 360 + (df['motor.state1.thetaPark_enc'].rolling(1).mean()*360/2/np.pi)  - (df['motor.state1.thetaPark'].rolling(1).mean()*360/2/np.pi)  , df['motor.hfi1.delta_id'].rolling(100).mean() )
# # plt.plot( 360 + (df['motor.state1.thetaPark_enc'].rolling(1).mean()*360/2/np.pi)  - (df['motor.state1.thetaPark'].rolling(1).mean()*360/2/np.pi)  , df['motor.hfi1.delta_iq'].rolling(100).mean()  )
# ## plt.plot( (df['motor.state1.thetaPark_enc'].rolling(100).mean()/2/np.pi) , ( df['motor.hfi1.delta_id'].rolling(100).mean()  + df['motor.hfi1.delta_iq'].rolling(100).mean() )  )
# plt.gca().xaxis.set_major_locator(plt.MaxNLocator(5, steps=[1,2,4.5,5,9,10]))
# plt.xlabel('Rotor angle [deg]')
# plt.ylabel('delta current [A]')
# plt.legend( ("Parallel" , "Orthogonal") , loc='best') 
# # plt.xlim( -180, 180)
# plt.title('Rotating current vector, stationary rotor')

# offset = hfi_v * (1/Ld + 1/Lq) * Ts * 0.5

# plt.figure()
# plt.plot( (df['motor.state1.thetaPark_enc'].rolling(100).mean()/2/np.pi)   , df['motor.hfi1.delta_id'].rolling(100).mean() - offset)
# plt.plot( (df['motor.state1.thetaPark_enc'].rolling(100).mean()/2/np.pi)  , df['motor.hfi1.delta_iq'].rolling(100).mean()  )
# plt.plot( (df['motor.state1.thetaPark_enc'].rolling(100).mean()/2/np.pi)  +0.125/2 , ( df['motor.hfi1.delta_id'].rolling(100).mean()  + df['motor.hfi1.delta_iq'].rolling(100).mean() -offset ) / np.sqrt(2) )


# %% Debug HFI rotating vector
vmax = 4000  # ERPM

setpar('motor.conf1.anglechoice', 99)
setpar('motor.state1.i_vector_acc', 10)
setpar('motor.state1.i_vector_radpers', vmax / 60 * 2*pi)

# %% Calibrate HFI offsets
signals = ['motor.state1.BEMFa', 'motor.state1.BEMFb', 'motor.state.sensBus', 'motor.state1.Iq_meas', 'motor.state1.Id_meas',
           'motor.state1.Id_e', 'motor.state1.Iq_e', 'motor.hfi1.delta_id', 'motor.hfi1.delta_iq', 'motor.state1.thetaPark_enc']
setTrace(signals)


setpar('motor.hfi1.hfi_method', 1)
Ki = 1000*2*pi

setpar('motor.hfi1.hfi_maxvel', 1e6)
setpar('motor.hfi1.hfi_gain', Ki)
setpar('motor.hfi1.hfi_gain_int2', 5*2*pi)
setpar('motor.hfi1.hfi_V', hfi_v)
setpar('motor.hfi1.hfi_on', 1)
setpar( 'motor.hfi1.diq_compensation_on' , 0)

setpar('motor.conf1.anglechoice', 100)
setpar('motor.state1.Id_offset_SP', 10)

setpar('motor.state1.thetaPark', 1)
time.sleep(1)
setpar('motor.state1.thetaPark', 0.5)
time.sleep(1)
setpar('motor.state1.thetaPark', 0)
time.sleep(1)


for i in range(361):
    setparpart('motor.hfi1.diq_compensation', 0.1, i)

did = []
diq = []
did_comp = []
diq_comp = []
THETA = np.linspace(0, 360, 361)
for theta in THETA:
    setpar('motor.state1.thetaPark', theta * 2*pi/360)
    time.sleep(0.05)
    df = trace(0.05)
    did.append(df['motor.hfi1.delta_id'].mean())
    diq.append(df['motor.hfi1.delta_iq'].mean())
    setpar( 'motor.hfi1.diq_compensation_on' , 1)
    time.sleep(0.05)
    df = trace(0.05)
    did_comp.append(df['motor.hfi1.delta_id'].mean())
    diq_comp.append(df['motor.hfi1.delta_iq'].mean())
    setpar( 'motor.hfi1.diq_compensation_on' , 0)


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
    # setparpart('motor.hfi1.diq_compensation', diq[i], i)
    # setparpart('motor.hfi1.diq_compensation', diq_correct[i], i)
    setparpart('motor.hfi1.diq_compensation', compneeded[i], i)

getsig('motor.hfi1.diq_compensation')


# %%
setpar( 'motor.hfi1.diq_compensation_on' , 1)
setpar( 'motor.hfi1.diq_compensation_on' , 0)
# %%
setpar('motor.state1.Iq_offset_SP',5)
setpar('motor.state1.Iq_offset_SP', 0)

# %%
signals = ['motor.state1.thetaPark' , 'motor.state1.thetaPark_enc' ]
setTrace(signals)

setpar( 'motor.hfi1.diq_compensation_on' , 0)
df1 = trace( 0.1 )
setpar( 'motor.hfi1.diq_compensation_on' , 1)
df2 = trace( 0.1 )


df1.plot()
df2.plot()

#%%  
setpar( 'motor.hfi1.hfi_useforfeedback' , 1)

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
setpar( 'motor.hfi1.diq_compensation_on' , 1)
setpar( 'motor.hfi1.diq_compensation_on' , 0)
#%%  
prepSP( 360/360*2*pi , 50*pi , 250 ,100000)

setpar('motor.setpoint.SPdir' , 1)
setpar('motor.setpoint.spNgo' , 2)
while (getsig('motor.state1.REFstatus') > 0):
    bla = 1;
setpar('motor.setpoint.SPdir' , 0)
setpar('motor.setpoint.spNgo' , 2)

#%%  
p = 2*np.pi/3
v = 100
a = 100*np.pi
j = 100000*np.pi
Nsp = 1
delay = 0
[t1, t2, t3, jd] = prepSP(  p , v , a , j )

for i in range(6):
    setpar('motor.setpoint.SPdir' , 1)
    setpar('motor.setpoint.spNgo',Nsp)
    while (getsig('motor.state1.REFstatus') > 0):
        bla = 1;
    time.sleep(delay)

setpar('motor.setpoint.SPdir' , 0)
setpar('motor.setpoint.spNgo',6)

#%%  
p = 20*np.pi
v = 200
a = 100*np.pi
j = 100000*np.pi
Nsp = 1

# delay = 0.3
# for p in np.linspace( 2*np.pi/360*1 , 2*np.pi/360*10 , 10):
#     [t1, t2, t3, jd] = prepSP(  p , v , a , j )
#     setpar('motor.setpoint.SPdir' , 1)
#     setpar('motor.setpoint.spNgo',Nsp)
#     while (getsig('motor.state1.REFstatus') > 0):
#         bla = 1;
#     time.sleep(delay)
#     setpar('motor.setpoint.SPdir' , 0)
#     setpar('motor.setpoint.spNgo',Nsp)
#     while (getsig('motor.state1.REFstatus') > 0):
#         bla = 1;
#     time.sleep(delay)
    
delay = 0.00
for p in np.linspace( 0.1 , 2*pi/10 , 11):
    [t1, t2, t3, jd] = prepSP(  p , v , a , j )
    setpar('motor.setpoint.SPdir' , 1)
    setpar('motor.setpoint.spNgo',Nsp)
    while (getsig('motor.state1.REFstatus') > 0):
        bla = 1;
    time.sleep(delay)
    setpar('motor.setpoint.SPdir' , 0)
    setpar('motor.setpoint.spNgo',Nsp)
    while (getsig('motor.state1.REFstatus') > 0):
        bla = 1;
    time.sleep(delay)
    
    
#%%  
BW = 100;

J = 2.5e-5

gain_at_BW = J * (BW*2*pi)**2
alpha_i = 6
alpha_1 = 3
alpha_2 = 3


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


setpar( 'motor.conf1.Kp_prep' , Kp )
setpar( 'motor.conf1.Kd_prep' , Kd )
setpar( 'motor.conf1.Ki_prep' , Ki )
setpar( 'motor.conf1.lowpass_c_prep' , lowpass_c )


setpar( 'motor.state1.rmechoffset' , getsig('motor.state1.rmechoffset') -getsig( 'motor.state1.emech' ) )

setpar( 'motor.conf1.Command' , 1 )

#%%  
setpar('motor.conf1.T_max' , 1e8)

NdownsamplePRBS = 5
N = 15*NdownsamplePRBS*2047

signals = ['motor.state1.Id_meas', 'motor.state1.Iq_meas',
           'motor.state1.dist', 'motor.state1.emech' , 'motor.state1.mechcontout' , 'motor.state1.Iq_SP' , 'motor.state1.Kp_out']
setTrace(signals )


gain = 1
setpar('motor.state1.Vq_distgain', 0)
setpar('motor.state1.Vd_distgain', 0)
setpar('motor.state1.Iq_distgain', 0)
setpar('motor.state1.Id_distgain', 0)
setpar('motor.state1.mechdistgain', 0)
setpar('motor.state1.rdistgain', 1)


setpar('motor.conf1.NdownsamplePRBS', NdownsamplePRBS)  # Downsampling
setpar('motor.state1.distval', gain)  # disturbance amplitude
setpar('motor.state1.distoff', 0)  # disturbance offset
df = trace(N * Ts)
setpar('motor.state1.distval', 0)  # disturbance amplitude
setpar('motor.state1.distoff', 0)  # disturbance offset
setpar('motor.conf1.NdownsamplePRBS', 1)  # Downsampling
setpar('motor.state1.rdistgain', 0)


dfout = getFFTdf(df, NdownsamplePRBS , 10*2047 )
f = dfout.index.values


plt.figure(1)
bode( dfout['motor.state1.mechcontout'] , f, 'Measured controller')