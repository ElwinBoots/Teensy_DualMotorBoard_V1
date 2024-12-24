# -*- coding: utf-8 -*-
"""
Created on Mon Feb  5 21:27:00 2024

@author: elwin
"""


m.setpar('motor.conf1.Lambda_m', 1)

# m.setTrace( ['motor.state1.BEMFa','motor.state1.BEMFb'])
m.setTrace( ['motor.state2.BEMFa','motor.state2.BEMFb'])

df = m.trace(2);
df.plot()