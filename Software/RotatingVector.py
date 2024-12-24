# -*- coding: utf-8 -*-
"""
Created on Sat Jan 20 09:34:43 2024

@author: elwin
"""
import numpy as np
import TeensyMotorControl as tc
m = tc.Motor(  )
motor = tc.MotorVariables( m )
pi = np.pi

# %%
m.setpar('motor.conf1.anglechoice', 99)
m.setpar('motor.state1.i_vector_acc', 1e8)

motor.state1.Iq_offset_SP = 5

m.setpar('motor.state1.i_vector_acc', 1e8)
m.setpar('motor.state1.i_vector_radpers', 2*pi/3)
# %%
m.setpar('motor.state1.i_vector_acc', 2)
m.setpar('motor.state1.i_vector_radpers', 200)

# %%
m.setpar('motor.state1.i_vector_acc', 1e8)
m.setpar('motor.state1.i_vector_radpers', 0)
motor.state1.Iq_offset_SP = 0
