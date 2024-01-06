import matplotlib.pyplot as plt
import pandas as pd
import struct
import numpy as np
import time
import control as ct
import TeensyMotorControl as tc
import pyqtgraph as pg

# For graph and tree:
from PyQt5.QtWidgets import QApplication, QGridLayout, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QLineEdit, QSlider, QCompleter
from PyQt5.QtCore import QObject, pyqtSignal, QEvent
from PyQt5.QtCore import Qt
import struct
from pyqtgraph.parametertree import Parameter, ParameterTree
from pyqtgraph.Qt import QtCore, QtWidgets
from pyqtgraph.widgets.PlotWidget import PlotWidget

pi = np.pi
# plt.rcParams["figure.dpi"] = 200 #Use this with 4k screens
plt.rcParams.update({"axes.grid": True})
plt.rcParams.update({"legend.loc": 'upper right'})
pg.setConfigOption('background', 'k')
pg.setConfigOption('foreground', 'w')

m = tc.Motor(  )
motor = tc.MotorVariables( m )

z = ct.TransferFunction( [1, 0] , [1] , float(m.Ts))

# Run code till here, and you will have the m and motor objects to work with 

# %%
Lq = 250e-6
Ld = 210e-6
R = 0.33

m.CL_cur( 0 )
motor.conf1.ridethewave = 1
motor.conf2.ridethewave = 1
time.sleep(0.5)
motor.conf1.commutationoffset = 0
motor.conf2.commutationoffset = 0

motor.state1.Valpha_offset = 1
motor.state2.Valpha_offset = 1

time.sleep(1)

offset1 = motor.state1.thetaPark_enc
offset2 = motor.state2.thetaPark_enc

motor.state1.Valpha_offset = 0
motor.state2.Valpha_offset = 0
motor.conf1.commutationoffset = -offset1
motor.conf2.commutationoffset = -offset2

# Servo motor Wittenstein cyber MSSI 055G
motor.conf1.Lambda_m = 0.01245
motor.conf1.N_pp =  4
motor.conf1.Lq = Lq
motor.conf1.Ld = Ld
motor.state1.R = R

# Servo motor Wittenstein cyber MSSI 055G  CONF2
motor.conf2.Lambda_m = 0.01245
motor.conf2.N_pp =  4
motor.conf2.Lq = Lq
motor.conf2.Ld = Ld
motor.state2.R = R

CL_cur( 1.5e3 )


# %%
m.setTrace([ 'motor.state.encoderPos1','motor.state.encoderPos2','motor.state.IndexFound1','motor.state.IndexFound2','motor.state1.emech'])
df = m.trace(1);


df.plot()

# %% Trampa 160KV
#Rotate by hand first
Ld = 19e-6
Lq = 34e-6
R = 0.035
m.setpar('motor.conf2.Lambda_m', 0.005405)
m.setpar('motor.conf2.N_pp',  7)
m.setpar('motor.conf2.Lq', Lq)
m.setpar('motor.conf2.Ld', Ld)
m.setpar('motor.state2.R', R)

m.setpar('motor.conf2.commutationoffset', 0)

m.setpar('motor.state2.Valpha_offset', 0.5)
time.sleep(0.5)
Valpha1 = m.getsig('motor.state2.Valpha_offset')
Ialpha1 = m.getsig('motor.state2.Ialpha')
Ibeta1 = m.getsig('motor.state2.Ibeta')
Ia1 = m.getsig('motor.state2.ia')
bus1 = m.getsig('motor.state.sensBus_lp')

m.setpar('motor.state2.Valpha_offset', 1)

time.sleep(1)
Valpha2 = m.getsig('motor.state2.Valpha_offset')
Va = m.getsig('motor.state2.Va')
Vb = m.getsig('motor.state2.Vb')
Ialpha2 = m.getsig('motor.state2.Ialpha')
Ibeta2 = m.getsig('motor.state2.Ibeta')
bus2 = m.getsig('motor.state.sensBus')
Ia2 = m.getsig('motor.state2.ia')
m.setpar('motor.state2.Valpha_offset', 0)

R = (Valpha2 - Valpha1) / (Ialpha2 - Ialpha1)

offset = m.getsig('motor.state2.thetaPark_enc')

m.setpar('motor.conf2.commutationoffset', -offset)

thetaPark = m.getsig('motor.state2.thetaPark')

# %%
m.setTrace(['s.sens1' , 's.sens2' , 's.sens3' , 's.sens4' , 's.sensBus' , 's.sensBus2' ])
df = m.trace(0.1)

plt.figure(1)
fftpsd( df['motor.state.sens1'])
plt.figure(2)
fftpsd( df['motor.state.sens2'])
plt.figure(3)
fftpsd( df['motor.state.sens3'])
plt.figure(4)
fftpsd( df['motor.state.sens4'])


# %%
# Trampa 160KV
f_bw = 1e3

m.setpar('motor.conf2.Kp_iq', Lq * f_bw * 2 * pi)  # Current loop Kp
m.setpar('motor.conf2.Ki_iq', R/Lq)  # Current loop Ki
m.setpar('motor.conf2.Kp_id', Ld * f_bw * 2 * pi)  # Current loop Kp
m.setpar('motor.conf2.Ki_id', R/Ld)  # Current loop Ki

m.setpar('motor.conf2.I_max', 14)
m.setpar('motor.conf2.maxDutyCycle', 0.99)

# %% Small inrunner surpass hobby
Ld = 7.5e-6
Lq = 7.5e-6
R = 0.053
kv = 4800 
N_pp = 2
# fluxlinkage = 60 / (np.sqrt(3) * 2 * np.pi * kv * N_pp) 
fluxlinkage = 0.0005743
m.setpar('motor.conf1.Lambda_m', fluxlinkage)
m.setpar('motor.conf1.N_pp',  N_pp)
m.setpar('motor.conf1.Lq', Lq)
m.setpar('motor.conf1.Ld', Ld)
m.setpar('motor.state1.R', R)

f_bw = 3e3
f_lp = f_bw*4

m.setpar('motor.conf1.Kp_iq', Lq * f_bw * 2 * pi)  # Current loop Kp
m.setpar('motor.conf1.Ki_iq', R/Lq)  # Current loop Ki
m.setpar('motor.conf1.Kp_id', Ld * f_bw * 2 * pi)  # Current loop Kp
m.setpar('motor.conf1.Ki_id', R/Ld)  # Current loop Ki

m.setpar('motor.conf1.I_max' , 20)
m.setpar('motor.conf1.maxDutyCycle' , 0.99)
m.setpar('motor.conf1.anglechoice', 1)

lowpass_c = 1-np.exp(-f_lp*2*pi*m.Ts)
m.setpar( 'motor.conf1.lowpass_Vd_c' , lowpass_c )
m.setpar( 'motor.conf1.lowpass_Vq_c' , lowpass_c )

# %% Iflight XING-E Pro 2207 1800KV
Ld = 7e-6
Lq = 7e-6
R = 0.06
kv = 1800
N_pp = 7
# fluxlinkage = 60 / (np.sqrt(3) * 2 * np.pi * kv * N_pp) 
fluxlinkage = 0.00046
m.setpar('motor.conf1.Lambda_m', fluxlinkage)
m.setpar('motor.conf1.N_pp',  N_pp)
m.setpar('motor.conf1.Lq', Lq)
m.setpar('motor.conf1.Ld', Ld)
m.setpar('motor.state1.R', R)

f_bw = 2e3

m.setpar('motor.conf1.Kp_iq', Lq * f_bw * 2 * pi)  # Current loop Kp
m.setpar('motor.conf1.Ki_iq', R/Lq)  # Current loop Ki
m.setpar('motor.conf1.Kp_id', Ld * f_bw * 2 * pi)  # Current loop Kp
m.setpar('motor.conf1.Ki_id', R/Ld)  # Current loop Ki

m.setpar('motor.conf1.I_max' , 20)
m.setpar('motor.conf1.maxDutyCycle' , 0.99)
m.setpar('motor.conf1.anglechoice', 1)


# %% Five33 TinyTurner 1404 Motor - 4533KV
Ld = 10e-6
Lq = 10e-6
R = 0.12
kv = 4533
N_pp = 6
# fluxlinkage = 60 / (np.sqrt(3) * 2 * np.pi * kv * N_pp) 
fluxlinkage = 0.000203 * 0.97
m.setpar('motor.conf1.Lambda_m', fluxlinkage)
m.setpar('motor.conf1.N_pp',  N_pp)
m.setpar('motor.conf1.Lq', Lq)
m.setpar('motor.conf1.Ld', Ld)
m.setpar('motor.state1.R', R)

f_bw = 3e3
f_lp = f_bw*8

m.setpar('motor.conf1.Kp_iq', Lq * f_bw * 2 * pi)  # Current loop Kp
m.setpar('motor.conf1.Ki_iq', R/Lq)  # Current loop Ki
m.setpar('motor.conf1.Kp_id', Ld * f_bw * 2 * pi)  # Current loop Kp
m.setpar('motor.conf1.Ki_id', R/Ld)  # Current loop Ki

m.setpar('motor.conf1.I_max' , 20)
m.setpar('motor.conf1.maxDutyCycle' , 0.99)
m.setpar('motor.conf1.anglechoice', 1)

lowpass_c = 1-np.exp(-f_lp*2*pi*m.Ts)
m.setpar( 'motor.conf1.lowpass_Vd_c' , lowpass_c )
m.setpar( 'motor.conf1.lowpass_Vq_c' , lowpass_c )

# %%

m.setpar('motor.conf1.anglechoice', 1)

m.setpar('motor.state1.Iq_offset_SP', 5)

m.setpar('motor.state1.Iq_offset_SP', 1.5)
time.sleep(0.5)
m.setpar('motor.state1.Iq_offset_SP', 0)


# %% Enable hfi 1
Ld = 19e-6
Lq = 34e-6
R = 0.035
m.setpar('motor.conf1.Lambda_m', 0.005405)
m.setpar('motor.conf1.N_pp',  7)
m.setpar('motor.conf1.Lq', Lq)
m.setpar('motor.conf1.Ld', Ld)
m.setpar('motor.state1.R', R)

m.CL_cur( 2e3 , 1)

m.setpar('s1.Id_offset_SP', 5)
time.sleep(0.5)
m.setpar('s1.Id_offset_SP', 0)

m.setpar('s1.hfi_use_lowpass', 1)
m.setpar('s1.hfi_method', 1)

Ki = 500*2*pi
hfi_v = 3

m.setpar('s1.hfi_maxvel', 1e6)
m.setpar('s1.hfi_gain', Ki)
m.setpar('s1.hfi_gain_int2', 5*2*pi)
m.setpar('s1.hfi_V', hfi_v)
m.setpar('s1.hfi_on', 1)
m.setpar('c1.anglechoice', 3)

#%%  
m.setpar( 's1.hfi_useforfeedback' , 1)

# m.CL( 1, 1, J=0.00021)
m.CL( 7, 1, J=0.00021)
# m.CL( 8, 1, J=0.00021)
# m.CL( 9, 1, J=0.00021)
# m.CL( 10, 1, J=0.00021)

# %% Enable hfi 2
Ld = 19e-6
Lq = 34e-6
R = 0.035
m.setpar('motor.conf2.Lambda_m', 0.005405)
m.setpar('motor.conf2.N_pp',  7)
m.setpar('motor.conf2.Lq', Lq)
m.setpar('motor.conf2.Ld', Ld)
m.setpar('motor.state2.R', R)

m.CL_cur( 2e3 , 2)

m.setpar('s2.Id_offset_SP', 5)
time.sleep(0.5)
m.setpar('s2.Id_offset_SP', 0)


m.setpar('s2.hfi_use_lowpass', 1)
m.setpar('s2.hfi_method', 1)

Ki = 500*2*pi
hfi_v = 3

m.setpar('s2.hfi_maxvel', 1e6)
m.setpar('s2.hfi_gain', Ki)
m.setpar('s2.hfi_gain_int2', 5*2*pi)
m.setpar('s2.hfi_V', hfi_v)
m.setpar('s2.hfi_on', 1)
m.setpar('c2.anglechoice', 3)

#%%  
m.setpar( 's2.hfi_useforfeedback' , 1)

# m.CL( 1, 2, J=0.00025)
m.CL( 7, 2, J=0.00025)
# m.CL( 8, 2, J=0.00025)
# m.CL( 9, 2, J=0.00025)


# %%
signals = m.setTrace(['motor.state1.ia', 'motor.state1.ib', 'motor.state1.ic'])
df = m.trace(0.1)
df.plot()

# %%
signals = m.setTrace(['motor.state1.Ialpha', 'motor.state1.Ibeta', 'motor.state.sensBus',
                   'motor.state.curtime', 'motor.state1.Va', 'motor.state1.Vb', 'motor.state1.Vc'])
df = m.trace(1)
df.plot()

# %%

signals = m.setTrace(['motor.state1.Id_SP', 'motor.state1.Iq_SP',  'motor.state1.Id_meas', 'motor.state1.Iq_meas',
                   'motor.state.sensBus', 'motor.state1.Vq', 'motor.state1.Vd', 'motor.state1.ia', 'motor.state1.ib', 'motor.state1.ic'])
df = m.trace(0.1)

df.plot()
# %%

signals = m.setTrace(['motor.state1.BEMFa', 'motor.state1.BEMFb'])
df = m.trace(1)

df.plot()

# %%
m.setpar('motor.state1.Id_offset_SP', -5)

# %%
signals = m.setTrace(['motor.state1.erpm',  'motor.state1.Id_SP', 'motor.state1.Iq_SP',  'motor.state1.Id_meas',
                   'motor.state1.Iq_meas', 'motor.state.sensBus', 'motor.state1.Vq', 'motor.state1.Vd', 'motor.state1.ia', 'motor.state1.ib', 'motor.state1.ic'])
df = m.trace(3)

df.plot()


# %%
signals = m.setTrace(['motor.state1.erpm',  'motor.state1.Id_SP', 'motor.state1.Iq_SP',  'motor.state1.Id_meas',
                   'motor.state1.Iq_meas', 'motor.state.sensBus', 'motor.state1.Vq', 'motor.state1.Vd','motor.state1.BEMFa', 'motor.state1.BEMFb'])

m.setpar('motor.state1.Iq_offset_SP', 0.5)
# %%
m.setpar('motor.state1.Iq_offset_SP', 4.5)
df = m.trace(3)
m.setpar('motor.state1.Iq_offset_SP', 2)
df2 = m.trace(1)
m.setpar('motor.state1.Iq_offset_SP', 0)
df.plot()

# %%
m.setpar('motor.state1.Iq_offset_SP', 5)

# %%
m.setpar('motor.state1.Iq_offset_SP', 3)
df2 = m.trace(1)
m.setpar('motor.state1.Iq_offset_SP', 1)
df3 = m.trace(1)
m.setpar('motor.state1.Iq_offset_SP', 0)
# %%
m.setpar('motor.state1.muziek_gain', 8)
m.setpar('motor.state2.muziek_gain', 1)

m.setpar('motor.state1.muziek_gain', 0)
m.setpar('motor.state2.muziek_gain', 0)


m.setpar('motor.state1.offsetVel' ,  70 )
m.setpar('motor.state2.offsetVel' ,  70 )

m.setpar('motor.state1.offsetVel' ,  0 )
m.setpar('motor.state2.offsetVel' ,  0 )


# %%
signals = m.setTrace(['motor.state1.thetaPark',  's1.delta_id' , 's1.delta_iq' ,
                   'motor.state1.thetaPark_enc', 'motor.state1.Iq_SP', 'motor.state1.Id_SP'])
                   # 'motor.state1.Id_meas','motor.state1.Iq_meas',
                   # 'motor.state1.Vd','motor.state1.Vq','motor.state1.I_bus','motor.state.sensBus'])


df = m.trace(2)
df.plot()

# %%
signals = m.setTrace([
                    'motor.state1.ia','motor.state1.ib' , 
                    'motor.state2.ia','motor.state2.ib' ])
                    # 'motor.state.sens1' ,'motor.state.sens2' ,'motor.state.sens3' ,'motor.state.sens4'])
                   # 'motor.state1.Id_meas','motor.state1.Iq_meas',
                   # 'motor.state1.Vd','motor.state1.Vq','motor.state1.I_bus','motor.state.sensBus'])

df = m.trace(1)
df.plot()



# %%
# signals = m.setTrace( ['motor1.curtime',
#                      'motor1.state.x' ,
#                      'motor1.state.current' ,
#                      'motor1.mot_conf.banaan_x' ,
#                      'motor2.mot_conf.banaan_x'])


signals = m.setTrace(m.signames[0:12])


signals = m.setTrace(['motor.state.curtime', 'motor.state.sens1',
                   'motor.state.sens2', 'motor.state.sens1_lp', 'motor.state.sens2_lp'])


signals = m.setTrace(['motor.state1.dist', 'motor.state1.noisebit'])


m.setpar('motor.state1.SPdir', 1)
signals = m.setTrace(['motor.state1.rmech'])
m.setpar('motor.state1.spNgo', 5)
df = m.trace(0.5)
df.plot()

plt.plot(np.diff(df.index))


m.setpar('motor.state1.x', [1])
m.setpar('motor.state2.x', [5])
print(m.getsig('motor.state1.x'))
print(m.getsig('motor.state2.x'))


m.setpar('motor.state1.current', [1, 4, 24, 5])
m.setpar('motor.state2.current', [2, 4, 24, 5])
print(m.getsig('motor.state1.current'))
print(m.getsig('motor.state2.current'))


m.setparpart('motor.state1.current', [5, 0], 2)
print(m.getsig('motor.state1.current'))

print(m.getsigpart('motor.state1.current', 1, 3))

# %% Current loop axis 1  
m.setpar('motor.conf1.anglechoice', 0)

NdownsamplePRBS = 1
N = 30*NdownsamplePRBS*2047
signals = ['motor.state1.Id_meas', 'motor.state1.Iq_meas',
           'motor.state1.dist', 'motor.state1.Vq', 'motor.state1.Vd']
m.setTrace(signals )


gain = 0.5
m.setpar('motor.state1.Vq_distgain', 1)
m.setpar('motor.state1.Vd_distgain', 1)
m.setpar('motor.state1.Iq_distgain', 0)
m.setpar('motor.state1.Id_distgain', 0)
m.setpar('motor.state1.mechdistgain', 0)

m.setpar('motor.conf.NdownsamplePRBS', NdownsamplePRBS)  # Downsampling
m.setpar('motor.state1.distval', gain)  # disturbance amplitude
m.setpar('motor.state1.distoff', 0)  # disturbance offset
df = m.trace(N * m.Ts)
m.setpar('motor.state1.distval', 0)  # disturbance amplitude
m.setpar('motor.state1.distoff', 0)  # disturbance offset
m.setpar('motor.conf.NdownsamplePRBS', 1)  # Downsampling

dfout = m.getFFTdf(df, NdownsamplePRBS , 10*2047 )
f = dfout.index.values
Pd = dfout['motor.state1.Id_meas'].values / dfout['motor.state1.Vd'].values
Pq = dfout['motor.state1.Iq_meas'].values / dfout['motor.state1.Vq'].values
Sd = dfout['motor.state1.Vd'].values
Sq = dfout['motor.state1.Vq'].values
# %% Current loop axis 2
m.setpar('motor.conf2.anglechoice', 0)

NdownsamplePRBS = 2
N = 10*NdownsamplePRBS*2047
signals = ['motor.state2.Id_meas', 'motor.state2.Iq_meas',
           'motor.state2.dist', 'motor.state2.Vq', 'motor.state2.Vd']
m.setTrace(signals )


gain = 0.5
m.setpar('motor.state2.Vq_distgain', 1)
m.setpar('motor.state2.Vd_distgain', 1)
m.setpar('motor.state2.Iq_distgain', 0)
m.setpar('motor.state2.Id_distgain', 0)
m.setpar('motor.state2.mechdistgain', 0)

m.setpar('motor.conf.NdownsamplePRBS', NdownsamplePRBS)  # Downsampling
m.setpar('motor.state2.distval', gain)  # disturbance amplitude
m.setpar('motor.state2.distoff', 0)  # disturbance offset
df = m.trace(N * m.Ts)
m.setpar('motor.state2.distval', 0)  # disturbance amplitude
m.setpar('motor.state2.distoff', 0)  # disturbance offset
m.setpar('motor.conf.NdownsamplePRBS', 1)  # Downsampling

dfout = m.getFFTdf(df, NdownsamplePRBS , 10*2047 )
f = dfout.index.values
Pd = dfout['motor.state2.Id_meas'].values / dfout['motor.state2.Vd'].values
Pq = dfout['motor.state2.Iq_meas'].values / dfout['motor.state2.Vq'].values
Sd = dfout['motor.state2.Vd'].values
Sq = dfout['motor.state2.Vq'].values

# %% Current loop plots
plt.figure(1)
m.bode( Pd , f, 'Measured D axis plant')
m.bode( Pq , f, 'Measured Q axis plant')

plt.figure(2)
m.bode( 1 / Sd - 1 , f, 'Open loop D')
m.bode( 1 / Sq - 1 , f, 'Open loop Q')

plt.figure(6)
m.bode( (1 / Sd - 1)/Pd , f, 'Controller D')
m.bode( (1 / Sq - 1)/Pq , f, 'Controller Q')


plt.figure(3)
m.nyquist( 1 / Sd - 1 , f, 'Open loop D')
m.nyquist( 1 / Sq - 1 , f, 'Open loop Q')

plt.figure(4)
plt.plot(
    f, np.abs(1/(Pd * f * 2 * np.pi)) * 1e6)
plt.grid()
plt.xlim([1e3, 10e3])
# plt.ylim([ 100 , 300])
plt.title('Ld [uH]')


plt.figure(5)
plt.plot(
    f, np.abs(1/(Pq * f * 2 * np.pi)) * 1e6)
plt.grid()
plt.xlim([1e3, 10e3])
# plt.ylim([ 100 , 300])
plt.title('Lq [uH]')

plt.figure(6)
m.bode( 1 - Sd , f, 'Closed loop D')
m.bode( 1 - Sq , f, 'Closed loop Q')

# %% Open loop identification
NdownsamplePRBS = 10
N = 15*NdownsamplePRBS*2047

signals = ['motor.state1.Id_meas', 'motor.state1.Iq_meas',
           'motor.state1.dist', 'motor.state1.emech' , 'motor.state1.mechcontout' , 'motor.state1.Iq_SP']
m.setTrace(signals )


gain = 0.05
m.setpar('motor.state1.Vq_distgain', 0)
m.setpar('motor.state1.Vd_distgain', 0)
m.setpar('motor.state1.Iq_distgain', 0)
m.setpar('motor.state1.Id_distgain', 0)
m.setpar('motor.state1.mechdistgain', 1)

m.setpar('motor.conf.NdownsamplePRBS', NdownsamplePRBS)  # Downsampling
m.setpar('motor.state1.distval', gain)  # disturbance amplitude
m.setpar('motor.state1.distoff', 0)  # disturbance offset
df = m.trace(N * m.Ts)
m.setpar('motor.state1.distval', 0)  # disturbance amplitude
m.setpar('motor.state1.distoff', 0)  # disturbance offset
m.setpar('motor.conf.NdownsamplePRBS', 1)  # Downsampling


dfout = getFFTdf(df, NdownsamplePRBS , 10*2047 )
f = dfout.index.values

plt.figure(1)
bode( -dfout['motor.state1.emech'] , f, 'Measured plant')


plt.figure(2)
bode( 1/(-dfout['motor.state1.emech'] * (2*pi*f)**2) , f, 'Measured inertia')


# %% Open loop identification motor 2
NdownsamplePRBS = 10
N = 10*NdownsamplePRBS*2047

signals = ['motor.state2.Id_meas', 'motor.state2.Iq_meas',
           'motor.state2.dist', 'motor.state2.emech' , 'motor.state2.mechcontout' , 'motor.state2.Iq_SP']
m.setTrace(signals )


gain = 0.2
m.setpar('motor.state2.Vq_distgain', 0)
m.setpar('motor.state2.Vd_distgain', 0)
m.setpar('motor.state2.Iq_distgain', 0)
m.setpar('motor.state2.Id_distgain', 0)
m.setpar('motor.state2.mechdistgain', 1)

m.setpar('motor.conf.NdownsamplePRBS', NdownsamplePRBS)  # Downsampling
m.setpar('motor.state2.distval', gain)  # disturbance amplitude
m.setpar('motor.state2.distoff', 0)  # disturbance offset
df = m.trace(N * m.Ts)
m.setpar('motor.state2.distval', 0)  # disturbance amplitude
m.setpar('motor.state2.distoff', 0)  # disturbance offset
m.setpar('motor.conf.NdownsamplePRBS', 1)  # Downsampling


dfout = getFFTdf(df, NdownsamplePRBS , 10*2047 )
f = dfout.index.values

plt.figure(1)
bode( -dfout['motor.state2.emech'] , f, 'Measured plant')


plt.figure(2)
bode( 1/(-dfout['motor.state2.emech'] * (2*pi*f)**2) , f, 'Measured inertia')


# %% set Lowpass and Notches 1
setLowpass( 1 , 0, 1200, 0.7 )
setNotch( 1 , 1, 200, 0, 0.1 )
setNotch( 1 , 2, 590, -20, 0.1 )
setNotch( 1 , 3, 700, 0, 0.1 )

# %% Closed loop identification 1
m.setpar('motor.state1.offsetVel' ,  250 )

NdownsamplePRBS = 20
Naver = 20
Nwait = 5
N = (Nwait+Naver)*NdownsamplePRBS*2047

signals = ['motor.state1.Id_meas', 'motor.state1.Iq_meas',
           'motor.state1.dist', 'motor.state1.emech' , 'motor.state1.mechcontout' , 'motor.state1.Iq_SP']
m.setTrace(signals )


gain = 0.5
m.setpar('motor.state1.Vq_distgain', 0)
m.setpar('motor.state1.Vd_distgain', 0)
m.setpar('motor.state1.Iq_distgain', 0)
m.setpar('motor.state1.Id_distgain', 0)
m.setpar('motor.state1.mechdistgain', 1)

m.setpar('motor.conf.NdownsamplePRBS', NdownsamplePRBS)  # Downsampling
m.setpar('motor.state1.distval', gain)  # disturbance amplitude
m.setpar('motor.state1.distoff', 0)  # disturbance offset

df = m.trace(N * m.Ts)
m.setpar('motor.state1.distval', 0)  # disturbance amplitude
m.setpar('motor.state1.distoff', 0)  # disturbance offset
m.setpar('motor.conf.NdownsamplePRBS', 1)  # Downsampling

m.setpar('motor.state1.offsetVel' ,  0 )

dfout = m.getFFTdf(df, NdownsamplePRBS , Nwait*NdownsamplePRBS*2047 )
f = dfout.index.values

S = dfout['motor.state1.mechcontout']
PS = -dfout['motor.state1.emech']
P = PS / S 
OL = 1/S - 1

plt.figure(1)
m.bode( OL , f, 'Measured Open Loop')

plt.figure(2)
m.nyquist( OL , f, 'Measured Open Loop')

plt.figure(3)
m.bode( P , f, 'Measured Plant')

plt.figure(4)
m.bode( 1/(P * (2*pi*f)**2) , f, 'Measured inertia')

plt.figure(5)
m.bode( OL / P , f, 'Measured C')

# %% set Lowpass and Notches 2
setLowpass( 2 , 0, 1200, 0.7 )
setNotch( 2 , 1, 200, -20, 0.1 )
setNotch( 2 , 2, 590, -20, 0.1 )
setNotch( 2 , 3, 700, -20, 0.1 )

# %% Closed loop identification motor 2
m.setpar('motor.state2.offsetVel' ,  100 )

NdownsamplePRBS = 10
Naver = 10
Nwait = 5
N = (Nwait+Naver)*NdownsamplePRBS*2047

signals = ['motor.state2.Id_meas', 'motor.state2.Iq_meas',
           'motor.state2.dist', 'motor.state2.emech' , 'motor.state2.mechcontout' , 'motor.state2.Iq_SP']
m.setTrace(signals )


gain = 0.2
m.setpar('motor.state2.Vq_distgain', 0)
m.setpar('motor.state2.Vd_distgain', 0)
m.setpar('motor.state2.Iq_distgain', 0)
m.setpar('motor.state2.Id_distgain', 0)
m.setpar('motor.state2.mechdistgain', 1)

m.setpar('motor.conf.NdownsamplePRBS', NdownsamplePRBS)  # Downsampling
m.setpar('motor.state2.distval', gain)  # disturbance amplitude
m.setpar('motor.state2.distoff', 0)  # disturbance offset
df = m.trace(N * m.Ts)
m.setpar('motor.state2.distval', 0)  # disturbance amplitude
m.setpar('motor.state2.distoff', 0)  # disturbance offset
m.setpar('motor.conf.NdownsamplePRBS', 1)  # Downsampling

m.setpar('motor.state2.offsetVel' ,  0 )

dfout = m.getFFTdf(df, NdownsamplePRBS , Nwait*NdownsamplePRBS*2047 )
f = dfout.index.values

S = dfout['motor.state2.mechcontout']
PS = -dfout['motor.state2.emech']
P = PS / S 
OL = 1/S - 1

plt.figure(1)
m.bode( OL , f, 'Measured Open Loop')

plt.figure(2)
m.nyquist( OL , f, 'Measured Open Loop')

plt.figure(3)
m.bode( P , f, 'Measured Plant')

plt.figure(4)
m.bode( 1/(P * (2*pi*f)**2) , f, 'Measured inertia')

plt.figure(5)
m.bode( OL / P , f, 'Measured C')


# %% Detect Lambda_m
signals = ['motor.state1.BEMFa', 'motor.state1.BEMFb', 'motor.state.sensBus',
           'motor.state1.Iq_meas', 'motor.state1.Id_meas', 'motor.state1.Id_e', 'motor.state1.Iq_e']
m.setTrace(signals)

# m.setpar('motor.conf1.Lambda_m', 1)

vmax = 20000  # ERPM

m.setpar('motor.conf1.anglechoice', 99)
m.setpar('motor.state1.Id_offset_SP', 2)
m.setpar('motor.state1.i_vector_acc', 500)
m.setpar('motor.state1.i_vector_radpers', vmax / 60 * 2*pi)

while abs(m.getsig('motor.state1.i_vector_radpers_act')) < 0.99 * vmax / 60 * 2*pi:
    a = 1

m.setpar('motor.state1.Id_offset_SP', 0)


df = m.trace(1.5)


# lambda_m = np.mean((np.max(data[1][:,0:2] , axis=0) - np.min(data[1][:,0:2] , axis=0))/2)

m.setpar('motor.state1.i_vector_radpers', 0)
m.setpar('motor.state1.i_vector_acc', 1e8)

df.filter(regex='BEMF').plot()

# %% Debug HFI
m.setpar('s1.hfi_method', 1)

Ki = 1000*2*pi
hfi_v = 12

m.setpar('s1.hfi_maxvel', 1e6)
m.setpar('s1.hfi_gain', Ki)
m.setpar('s1.hfi_gain_int2', 5*2*pi)
m.setpar('s1.hfi_V', hfi_v)
m.setpar('s1.hfi_on', 1)

m.setpar('motor.conf1.anglechoice', 101)
m.setpar('motor.state1.Iq_offset_SP',5)

time.sleep(2)

# m.setpar('motor.state1.thetaPark', 0)

# vmax = 500  # ERPM
# m.setpar('motor.conf1.anglechoice', 99)
# m.setpar('motor.state1.i_vector_acc', 100000)
# m.setpar('motor.state1.i_vector_radpers', vmax / 60 * 2*pi)

signals = ['motor.state1.BEMFa', 'motor.state1.BEMFb', 'motor.state.sensBus', 'motor.state1.Iq_meas', 'motor.state1.Id_meas',
           'motor.state1.Id_e', 'motor.state1.Iq_e', 's1.delta_id', 's1.delta_iq', 'motor.state1.thetaPark' , 'motor.state1.thetaPark_enc', 'motor.state.is_v7']
m.setTrace(signals)

df = m.trace(2)

m.setpar('s1.hfi_on', 0)
m.setpar('motor.conf1.anglechoice', 0)
m.setpar('motor.state1.Iq_offset_SP',0)

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

# offset = hfi_v * (1/Ld + 1/Lq) * m.Ts * 0.5

# plt.figure()
# plt.plot( (df['motor.state1.thetaPark_enc'].rolling(100).mean()/2/np.pi)   , df['s1.delta_id'].rolling(100).mean() - offset)
# plt.plot( (df['motor.state1.thetaPark_enc'].rolling(100).mean()/2/np.pi)  , df['s1.delta_iq'].rolling(100).mean()  )
# plt.plot( (df['motor.state1.thetaPark_enc'].rolling(100).mean()/2/np.pi)  +0.125/2 , ( df['s1.delta_id'].rolling(100).mean()  + df['s1.delta_iq'].rolling(100).mean() -offset ) / np.sqrt(2) )


# %% Debug HFI rotating vector
vmax = 4000  # ERPM

m.setpar('motor.conf1.anglechoice', 99)
m.setpar('motor.state1.i_vector_acc', 10)
m.setpar('motor.state1.i_vector_radpers', vmax / 60 * 2*pi)

# %% Calibrate HFI offsets
signals = ['motor.state1.BEMFa', 'motor.state1.BEMFb', 'motor.state.sensBus', 'motor.state1.Iq_meas', 'motor.state1.Id_meas',
           'motor.state1.Id_e', 'motor.state1.Iq_e', 's1.delta_id', 's1.delta_iq', 'motor.state1.thetaPark_enc']
m.setTrace(signals)


m.setpar('s1.hfi_method', 1)
Ki = 1000*2*pi

m.setpar('s1.hfi_maxvel', 1e6)
m.setpar('s1.hfi_gain', Ki)
m.setpar('s1.hfi_gain_int2', 5*2*pi)
m.setpar('s1.hfi_V', hfi_v)
m.setpar('s1.hfi_on', 1)
m.setpar( 's1.diq_compensation_on' , 0)

m.setpar('motor.conf1.anglechoice', 100)
m.setpar('motor.state1.Id_offset_SP', 10)

m.setpar('motor.state1.thetaPark', 1)
time.sleep(1)
m.setpar('motor.state1.thetaPark', 0.5)
time.sleep(1)
m.setpar('motor.state1.thetaPark', 0)
time.sleep(1)


for i in range(361):
    m.setparpart('s1.diq_compensation', 0.1, i)

did = []
diq = []
did_comp = []
diq_comp = []
THETA = np.linspace(0, 360, 361)
for theta in THETA:
    m.setpar('motor.state1.thetaPark', theta * 2*pi/360)
    time.sleep(0.05)
    df = m.trace(0.05)
    did.append(df['s1.delta_id'].mean())
    diq.append(df['s1.delta_iq'].mean())
    m.setpar( 's1.diq_compensation_on' , 1)
    time.sleep(0.05)
    df = m.trace(0.05)
    did_comp.append(df['s1.delta_id'].mean())
    diq_comp.append(df['s1.delta_iq'].mean())
    m.setpar( 's1.diq_compensation_on' , 0)


m.setpar('motor.state1.Id_offset_SP', 0)

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

Ld_est = hfi_v / np.mean(did) * m.Ts

# %% Store HFI compensation data
for i in range(len(diq)):
    # m.setparpart('s1.diq_compensation', diq[i], i)
    # m.setparpart('s1.diq_compensation', diq_correct[i], i)
    m.setparpart('s1.diq_compensation', compneeded[i], i)

m.getsig('s1.diq_compensation')


# %%
m.setpar( 's1.diq_compensation_on' , 1)
m.setpar( 's1.diq_compensation_on' , 0)
# %%
m.setpar('s1.Iq_offset_SP',5)
m.setpar('s1.Iq_offset_SP', 0)
# %%
m.setpar('s2.Iq_offset_SP',5)
m.setpar('s2.Iq_offset_SP', 0)
# %%
signals = ['motor.state1.thetaPark' , 'motor.state1.thetaPark_enc' ]
m.setTrace(signals)

m.setpar( 's1.diq_compensation_on' , 0)
df1 = m.trace( 0.1 )
m.setpar( 's1.diq_compensation_on' , 1)
df2 = m.trace( 0.1 )


df1.plot()
df2.plot()

#%%  
m.setpar( 's2.hfi_useforfeedback' , 1)


CL( 1, 2, J=0.000316)

# m.setpar('I_max' , 20)

# m.setpar( 'motor.conf2.Kp' , 0)
# m.setpar( 'motor.state2.rmechoffset' , 0 )
# m.setpar( 'motor.state2.rmechoffset' , -m.getsig( 'motor.state2.emech' ) )
# m.getsig('motor.state2.emech'  )




#Trampa 185 KV low BW fun:
# Kp =0.5
# fBW = 20
# alpha1 = 3
# alpha2 = 3
# fInt = 0
# fLP = fBW * 7
# m.setpar( 'motor.conf2.maxerror' , 1.5*np.pi )

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
# m.setpar( 'motor.conf1.maxerror' , 0.5 )

# Kp =30
# fBW = 40
# alpha1 = 3
# alpha2 = 3
# fInt = fBW / 8
# fLP = fBW * 7

# m.setpar( 'motor.conf2.fBW' , fBW)
# m.setpar( 'motor.conf2.alpha1' , alpha1)
# m.setpar( 'motor.conf2.alpha2' , alpha2)
# m.setpar( 'motor.conf2.fInt' , fInt)
# m.setpar( 'motor.conf2.fLP' ,  fLP)

# m.ser.write(b'C')

# m.setpar( 'motor.conf2.Kp' , Kp)

# %%
m.setpar( 'motor.conf1.Kp' , 0)
m.setpar( 'motor.state1.rmechoffset' , 0 )
m.setpar( 'motor.state1.rmechoffset' , -m.getsig( 'motor.state1.emech' ) )
m.getsig('motor.state1.emech'  )

fBW = 100
m.setpar( 'motor.conf1.fBW' , fBW)
m.setpar( 'motor.conf1.alpha1' , 3)
m.setpar( 'motor.conf1.alpha2' , 4)
m.setpar( 'motor.conf1.fInt' , fBW / 6)
m.setpar( 'motor.conf1.fLP' ,  fBW * 8)


m.ser.write(b'C')

m.setpar( 'motor.conf1.Kp' , 23.3)

# %%
m.setpar( 's1.diq_compensation_on' , 1)
m.setpar( 's1.diq_compensation_on' , 0)

#%%  
Lq = 250e-6
Ld = 210e-6
R = 0.33
# Lq = 0
# Ld = 0
# R = 0
m.setpar('motor.conf1.Lq', Lq)
m.setpar('motor.conf1.Ld', Ld)
m.setpar('motor.state1.R', R)
#%%   Setpoint
downsample = 10
m.setpar('motor.state1.rdelay' , 0)

m.setpar('motor.state1.Jload' , 7.2e-5)
m.setpar('motor.state1.velFF' , 0.0002)
m.setpar('motor.state2.Jload' , 7.2e-5)
m.setpar('motor.state2.velFF' , 0.0002)
# m.setpar('motor.state1.Jload' , 0)
# m.setpar('motor.state1.velFF' , 0)
# m.setpar('motor.state2.Jload' , 0)
# m.setpar('motor.state2.velFF' , 0)
# 
m.setTrace( ['motor.state.sensBus' , 'motor.state.sensBus2' , 'motor.state1.mechcontout' , 'motor.state1.T_FF_acc', 'motor.state1.T_FF_vel', 'motor.state.sensBus_lp','motor.state1.rmech' , 'motor.state1.vel', 'motor.state1.emech', 'motor.state2.emech', 'motor.state1.Vd', 'motor.state1.Vq' , 's1.VqFF' ,'motor.state1.Iq_SP', 'motor.state2.Iq_SP'] , downsample)
# m.setTrace( ['motor.state.sensBus' , 'motor.state.sensBus2' , 'motor.state2.mechcontout' , 's2.T_FF_acc', 's2.T_FF_vel', 'motor.state.sensBus_lp','s2.rmech' , 's2.vel','s2.ymech' , 's2.emech', 's2.Vd', 's2.Vq' , 's2.Iq_SP', 's2.Id_meas' , 's2.Iq_meas', 's1.Id_meas' , 's1.Iq_meas'])
# m.setTrace( ['motor.state1.rmech' , 'motor.state2.rmech' , 'motor.state1.ymech' , 'motor.state2.ymech' ,'motor.state1.vel' , 'motor.state2.vel', 'motor.state1.emech' , 'motor.state2.emech'])
# m.setTrace( ['motor.state1.rmech' ,  'motor.state1.ymech' , 'motor.state1.emech' , 's2.thetaPark' ,'s1.Iq_SP' , 's1.Iq_meas' , 's1.Iq_e', 's1.Id_meas' , 's2.Iq_SP' , 's2.Iq_meas' , 's2.Iq_e', 's2.Id_meas' ,  's1.Vd', 's1.Vq',   's.sens1', 's.sens2','s.sens3', 's.sens4', 's2.ia', 's2.ib'])
# m.setTrace( ['s1.rmech' ,  's1.ymech' , 's1.emech' ])
# m.setTrace( ['s2.rmech' ,  's2.ymech' , 's2.emech' ])

p = 360
# prepSP( p/360*2*pi , 15 , 800 ,1500000 , 1)
# prepSP( p/360*2*pi , 15 , 800 ,1500000 , 2)
prepSP( p/360*2*pi , 150 , 8000 ,1500000 , 1)
prepSP( p/360*2*pi , 150 , 8000 ,1500000 , 2)

m.setpar('motor.conf2.Command' , 3)
df = m.trace(0.4)
while (m.getsig('motor.state2.spNgo') > 0 or m.getsig('motor.state2.REFstatus') > 0 ):
    bla = 1;
m.setpar('motor.conf2.Command' , 4)


plt.figure();ax = plt.gca()
df.plot(ax=ax) # draws to fig1 now

plt.figure(2)
(df['motor.state1.emech']/2/pi*20000).plot()
# (df['motor.state2.rmech']/2/pi*20000).plot()

# plt.figure()
# df['motor.state1.Vq'].plot()
# df['motor.state1.VqFF'].plot()
# (df['motor.state1.Iq_SP']*R).plot()


#%%  
prepSP( 360/360*2*pi , 250 , 6000 ,2500000)
N = 1
m.setpar('motor.state1.SPdir' , 1)
m.setpar('motor.state1.spNgo' , N)

while (m.getsig('motor.state1.spNgo') > 0 or m.getsig('motor.state1.REFstatus') > 0 ):
    bla = 1;
m.setpar('motor.state1.SPdir' , 0)
m.setpar('motor.state1.spNgo' , N)

#%%  
p = 360
prepSP( p/360*2*pi , 250 , 6000 ,2500000 , 1)
prepSP( p/360*2*pi , 250 , 6000 ,2500000 , 2)
N = 2

m.setpar('motor.state1.SPdir' , 1)
m.setpar('motor.state2.SPdir' , 1)
m.setpar('motor.state1.spNgo' , N)
m.setpar('motor.state2.spNgo' , N)
while (m.getsig('motor.state2.spNgo') > 0 or m.getsig('motor.state2.REFstatus') > 0 ):
    bla = 1;
m.setpar('motor.state1.SPdir' , 0)
m.setpar('motor.state2.SPdir' , 0)
m.setpar('motor.state1.spNgo' , N)
m.setpar('motor.state2.spNgo' , N)


#%%  
p = 2*np.pi/3
v = 100
a = 100*np.pi
j = 100000*np.pi
Nsp = 1
delay = 0
[t1, t2, t3, jd] = prepSP(  p , v , a , j )

for i in range(6):
    m.setpar('motor.state1.SPdir' , 1)
    m.setpar('motor.state1.spNgo',Nsp)
    while (m.getsig('motor.state1.REFstatus') > 0):
        bla = 1;
    time.sleep(delay)

m.setpar('motor.state1.SPdir' , 0)
m.setpar('motor.state1.spNgo',6)

#%%  
p = 20*np.pi
v = 200
a = 100*np.pi
j = 100000*np.pi
Nsp = 1

# delay = 0.3
# for p in np.linspace( 2*np.pi/360*1 , 2*np.pi/360*10 , 10):
#     [t1, t2, t3, jd] = prepSP(  p , v , a , j )
#     m.setpar('motor.state1.SPdir' , 1)
#     m.setpar('motor.state1.spNgo',Nsp)
#     while (m.getsig('motor.state1.REFstatus') > 0):
#         bla = 1;
#     time.sleep(delay)
#     m.setpar('motor.state1.SPdir' , 0)
#     m.setpar('motor.state1.spNgo',Nsp)
#     while (m.getsig('motor.state1.REFstatus') > 0):
#         bla = 1;
#     time.sleep(delay)
    
delay = 0.00
for p in np.linspace( 0.1 , 2*pi/10 , 11):
    [t1, t2, t3, jd] = prepSP(  p , v , a , j )
    m.setpar('motor.state1.SPdir' , 1)
    m.setpar('motor.state1.spNgo',Nsp)
    while (m.getsig('motor.state1.REFstatus') > 0):
        bla = 1;
    time.sleep(delay)
    m.setpar('motor.state1.SPdir' , 0)
    m.setpar('motor.state1.spNgo',Nsp)
    while (m.getsig('motor.state1.REFstatus') > 0):
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
        Ki = BW*2*pi*m.Ts/alpha_i
    else:
        Ki = 0
    if alpha_1 > 0:
        Kd = alpha_1/(m.Ts*BW*2*pi)
    else:
        Kd = 0
    if alpha_2 > 0:
        lowpass_c = 1-np.exp(-BW*2*pi*alpha_2*m.Ts)
    else:
        lowpass_c =1
    Kp = gain_at_BW * abs(lowpass_c + np.exp(pi*BW*m.Ts*2j) - 1) / (abs(lowpass_c*((Ki*np.exp(pi*BW*m.Ts*2j)) / (np.exp(pi*BW*m.Ts*2j) - 1) + 1) * (Kd - Kd*np.exp(-pi*BW*m.Ts*2j) + 1) )) 
else:
    Kp = 0
    Ki = 0
    Kd = 0
    lowpass_c = 1

m.setpar( 'motor.conf1.Kp_prep' , Kp )
m.setpar( 'motor.conf1.Ki_prep' , Ki )
m.setpar( 'motor.conf1.Kd_prep' , Kd )
m.setpar( 'motor.conf1.lowpass_c_prep' , lowpass_c )

m.setpar( 'motor.conf1.Command' , 2 ) #Reset error

m.setpar( 'motor.conf1.Command' , 1 ) #Activate controller

# CONF2

J = 4e-5

gain_at_BW = J * (BW*2*pi)**2
# alpha_i = 6
# alpha_1 = 3
# alpha_2 = 3


if BW > 0:
    if alpha_i > 0:
        Ki = BW*2*pi*m.Ts/alpha_i
    else:
        Ki = 0
    if alpha_1 > 0:
        Kd = alpha_1/(m.Ts*BW*2*pi)
    else:
        Kd = 0
    if alpha_2 > 0:
        lowpass_c = 1-np.exp(-BW*2*pi*alpha_2*m.Ts)
    else:
        lowpass_c =1
    Kp = gain_at_BW * abs(lowpass_c + np.exp(pi*BW*m.Ts*2j) - 1) / (abs(lowpass_c*((Ki*np.exp(pi*BW*m.Ts*2j)) / (np.exp(pi*BW*m.Ts*2j) - 1) + 1) * (Kd - Kd*np.exp(-pi*BW*m.Ts*2j) + 1) )) 
else:
    Kp = 0
    Ki = 0
    Kd = 0
    lowpass_c = 1

m.setpar( 'motor.conf2.Kp_prep' , Kp )
m.setpar( 'motor.conf2.Ki_prep' , Ki )
m.setpar( 'motor.conf2.Kd_prep' , Kd )
m.setpar( 'motor.conf2.lowpass_c_prep' , lowpass_c )

m.setpar( 'motor.conf2.Command' , 1 ) #Activate controller

#%% 
m.setTrace( ['motor.state.sensBus'  ,'motor.state1.rmech' , 'motor.state1.vel', 'motor.state1.emech',  'motor.state1.ymech', 'motor.state1.Vd', 'motor.state1.Vq' , 'motor.state1.Iq_SP'])
# m.setTrace( ['motor.state1.emech' ])

p = 360
prepSP( p/360*2*pi , 1 , 10 ,2500000 , 1)

m.setpar('motor.state1.SPdir' , 1)
m.setpar('motor.state1.spNgo' , 1)
df = m.trace(2)
while (m.getsig('motor.state2.spNgo') > 0 or m.getsig('motor.state2.REFstatus') > 0 ):
    bla = 1;
m.setpar('motor.state1.SPdir' , 0)
m.setpar('motor.state2.SPdir' , 0)
m.setpar('motor.state1.spNgo' , 1)
m.setpar('motor.state2.spNgo' , 1)

df.plot()
#%% Controller tuning current loop

Lq = 250e-6
Ld = 210e-6
R = 0.33

f_bw = 3e3
f_lp = f_bw*3
C1 = discrete_lowpass( f_bw*6 , 0.5 )

Kp = Lq * f_bw * 2 * pi 
Ki = R/Lq * m.Ts

lowpass_c = 1-np.exp(-f_lp*2*pi*m.Ts)

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

# BW = 250
# J = 7.5e-5
# gain_at_BW = J * (BW*2*pi)**2
# alpha_i = 2.5
# alpha_1 = 2.5
# alpha_2 = 7
# C1 = discrete_lowpass( 4000, 0.4 )
# C2 = discrete_lowpass( 8000, 0.4 )
# # C2 = #discrete_notch( 590, -20, 0.1) 
# C3 = 1
# C4 = 1

# BW = 150
# J = 7.5e-5
# gain_at_BW = J * (BW*2*pi)**2
# alpha_i = 0
# alpha_1 = 1000
# alpha_2 = 3
# C1 = discrete_lowpass( 4000, 0.6 )
# C2 = 1
# # C2 = #discrete_notch( 590, -20, 0.1) 
# C3 = 1
# C4 = 1

BW = 40
J = 0.000211348903
gain_at_BW = J * (BW*2*pi)**2
alpha_i = 6
alpha_1 = 3
alpha_2 = 3
C1 = m.discrete_lowpass( 240, 0.6 )
C2 = 1
# C2 = #discrete_notch( 590, -20, 0.1) 
C3 = 1
C4 = 1



if BW > 0:
    if alpha_i > 0:
        Ki = BW*2*pi*m.Ts/alpha_i
    else:
        Ki = 0
    if alpha_1 > 0:
        Kd = alpha_1/(m.Ts*BW*2*pi)
    else:
        Kd = 0
    if alpha_2 > 0:
        lowpass_c = 1-np.exp(-BW*2*pi*alpha_2*m.Ts)
    else:
        lowpass_c =1
    Kp = gain_at_BW * abs(lowpass_c + np.exp(pi*BW*m.Ts*2j) - 1) / (abs(lowpass_c*((Ki*np.exp(pi*BW*m.Ts*2j)) / (np.exp(pi*BW*m.Ts*2j) - 1) + 1) * (Kd - Kd*np.exp(-pi*BW*m.Ts*2j) + 1) )) 
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
m.bode( P * C , f )

plt.figure(2)
m.nyquist( P * C , f )


#%% 
pos([0,180])
#%% 
vel([100,-100])
#%% 
vel(0)
#%% 
s1 = m.getsig('s.sens1_calib' )
s2 = m.getsig('s.sens2_calib' )
s3 = m.getsig('s.sens3_calib' )
s4 = m.getsig('s.sens4_calib' )

#%% 


# m.getsig('s.sens1_calib')
# m.getsig('s.sens1_calib')
# m.getsig('s.sens1_calib')m.getsig('s.sens1_calib')
m.setpar('s.sens1_calib'  , 1.65 )
m.setpar('s.sens2_calib'  , 1.65 )
m.setpar('s.sens3_calib'  , 1.65 )
m.setpar('s.sens4_calib'  , 1.65 )



#%% 
m.pos_wait([180,0])
#%% 

v = 150
a = 5000

m.pos_wait([0,180] , vel=v , acc = a)
m.pos_wait([180,0], vel=v , acc = a)
m.pos_wait([-90,90], vel=v , acc = a)
m.pos_wait([-270,-90] , vel=v, acc = a)
m.pos_wait([0,180] , vel=v, acc = a)
m.pos_wait([180,0], vel=v, acc = a)
m.pos_wait([-90,90], vel=v, acc = a)
m.pos_wait([-270,-90] , vel=v, acc = a)
m.pos_wait([-180,0] , vel=v, acc = a)
m.pos_wait([0,-180], vel=v, acc = a)
m.pos_wait([90,-90], vel=v, acc = a)
m.pos_wait([-90,-270] , vel=v, acc = a)
m.pos_wait([180,0] , vel=v, acc = a)



#%% 165 kv
v = 100
a = 1000

m.pos_wait( [ 360 , 360 ] , vel=v , acc = a)
m.pos_wait( [ 0 , 0 ] , vel=v , acc = a)


#%% Relative setpoints with trace
# time.sleep(10)

v = 200
a = 2000
# a = 2000 #Only motor 1

motor.state1.Jload = 0.000195

m.setpar('motor.state1.velFF' , 0.00055)
m.setpar('motor.state2.velFF' , 0.00055)

m.setTrace([ 'motor.state1.rmech' , 'motor.state1.ymech' , 'motor.state1.emech' , 'motor.state1.Vd', 'motor.state1.Vq', 'motor.state1.Iq_SP','motor.state2.Iq_SP','motor.state1.Iq_meas', 'motor.state1.Id_meas', 'motor.state1.hfi_abs_pos', 'motor.state1.hfi_curangleest','motor.state1.mechcontout' ,'motor.state2.mechcontout' ,'motor.state1.T_FF_acc' ,'motor.state1.T_FF_vel','motor.state2.T_FF_acc' ,'motor.state2.T_FF_vel'  ,'motor.state.sensBus' ])

m.tracebg(  )

# time.sleep(0.1)

m.rel( [ 360 , 360 ] , vel=v , acc = a)
time.sleep(0.1)
m.rel( [ -360 , -360] , vel=v , acc = a)
time.sleep(0.1)
m.rel( [ 360 , 360 ] , vel=v , acc = a)
time.sleep(0.1)
m.rel( [ -360 , -360 ] , vel=v , acc = a)
time.sleep(0.1)
m.rel( [ 360 , 360 ] , vel=v , acc = a)
time.sleep(0.1)
m.rel( [ -360 , -360] , vel=v , acc = a)
time.sleep(0.1)
m.rel( [ 360 , 360 ] , vel=v , acc = a)
time.sleep(0.1)
m.rel( [ -360 , -360 ] , vel=v , acc = a)
time.sleep(0.1)
m.rel( [ 360 , 360 ] , vel=v , acc = a)
time.sleep(0.1)
m.rel( [ -360 , -360] , vel=v , acc = a)
time.sleep(0.1)
m.rel( [ 360 , 360 ] , vel=v , acc = a)
time.sleep(0.1)
m.rel( [ -360 , -360 ] , vel=v , acc = a)
time.sleep(0.1)

# time.sleep(0.1)

df = m.stoptracegetdata()

# df.filter(regex='sens').plot()


df.filter(regex='state1..me|state1.me').plot()


# df.filter(regex='emech').plot()

# df.filter(regex='T_FF').plot()

# df.filter(regex='state1.Iq').plot()
# df.filter(regex='state1.Id').plot()



# df['motor.state1.mechcontout'].plot()
# (df['motor.state1.T_FF_acc']+df['motor.state1.T_FF_vel']).plot()
# (df['motor.state2.T_FF_acc']+df['motor.state2.T_FF_vel']).plot()


# df.filter(regex='Iq_SP').plot()


#%% Open loop vector rotation
Ld = 19e-6
Lq = 34e-6
R = 0.035
m.setpar('motor.conf1.Lambda_m', 0.005405)
m.setpar('motor.conf1.N_pp',  7)
m.setpar('motor.conf1.Lq', Lq)
m.setpar('motor.conf1.Ld', Ld)
m.setpar('motor.state1.R', R)

m.CL_cur( 1e3 , 1)

m.setpar('motor.conf1.anglechoice', 99)
m.setpar('motor.state1.Id_offset_SP', 5)
m.setpar('motor.state1.i_vector_acc', 1e8)
m.setpar('motor.state1.i_vector_radpers', 2*pi)



m.setTrace([ 'motor.state1.Valpha' , 'motor.state1.Vbeta' ,  'motor.state1.Va',  'motor.state1.Vb' ,  'motor.state1.Vc' ,  'motor.state1.Ialpha' , 'motor.state1.Ibeta' ,  'motor.state1.ia'  ,  'motor.state1.ib' ,  'motor.state1.ic'  ])


df = m.trace( 1 )


df.plot()
