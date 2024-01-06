# -*- coding: utf-8 -*-
"""
Created on Sun Nov  5 13:12:34 2023

@author: elwin
"""

import TeensyMotorControl as tc
import time


m = tc.Motor(  )
motor = tc.MotorVariables( m )






m.setTrace([ 'motor.state1.thetawave' , 'motor.state.curtime' , 'motor.state1.emech' , 'motor.state1.rmech' , 'motor.state1.ymech' , 'motor.state1.Vd', 'motor.state1.Vq', 'motor.state1.Iq_meas', 'motor.state1.Id_meas', 'motor.state1.hfi_abs_pos', 'motor.state1.hfi_curangleest'])

m.tracebg( )

for i in range(int(2e4)):
  motor.state1.thetawave = i
for i in range(int(2e4),0,-1):
  motor.state1.thetawave = i
for i in range(int(2e4)):
  motor.state1.thetawave = i
for i in range(int(2e4),0,-1):
  motor.state1.thetawave = i
  
df = m.stoptracegetdata()

df['motor.state1.thetawave'].plot()




df['motor.state.curtime'].diff().plot()



df['motor.state1.ymech'].plot()
df['motor.state1.Vd'].plot()
df['motor.state1.Vq'].plot()
df['motor.state1.Id_meas'].plot()
df['motor.state1.Iq_meas'].plot()

df['motor.state1.rmech'].plot()
df['motor.state1.ymech'].plot()
df['motor.state1.hfi_abs_pos'].plot()




t = time.time()
m.setTrace(range(50))
df = m.trace(1)
elapsed = time.time() - t
print(elapsed)
print(df)






t = time.time()
m.tracebg( downsample=10000)
time.sleep(1)
df = m.stoptracegetdata()




m.stoptrace()

df = m.gettracedata()
elapsed = time.time() - t
print(elapsed)
print(df)



