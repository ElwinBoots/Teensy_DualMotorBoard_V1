# -*- coding: utf-8 -*-
"""
Created on Sun Nov  5 13:12:34 2023

@author: elwin
"""


import motorclass
import time


m = motorclass.Motor(  )
motor = motorclass.MotorVariables( m )


m.setTrace(range(20))



m.tracebg( 300 )

for i in range(int(2e4)):
  motor.conf1.advancefactor = i
for i in range(int(2e4),0,-1):
  motor.conf1.advancefactor = i
for i in range(int(2e4)):
  motor.conf1.advancefactor = i
for i in range(int(2e4),0,-1):
  motor.conf1.advancefactor = i
  
df = m.stoptracegetdata()

df['motor.conf1.advancefactor'].plot()


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



