# -*- coding: utf-8 -*-
"""
Created on Sun Nov  5 10:40:59 2023

@author: elwin
"""




import motorclass
m = motorclass.Motor(  )
motor = motorclass.MotorVariables( m )

import threading
import time

def serialEvent( m ):
    while True:
      while m.readinbackground is True:
        if m.ser.in_waiting > 0:
          m.data.append(m.ser.readall())
      time.sleep(0.1)
    return


m.readinbackground = True
m.data = list()

t = threading.Thread(target=serialEvent , args=(m,))
t.start()


#%%


import struct




i = 2
m.ser.write(b'b' + struct.pack('I', i))



# m.ser.reset_input_buffer()
# m.ser.write(b'b' + struct.pack('I', i))
# buffer = m.ser.readall()


m.ser.inWaiting()

m.readinbackground = True

m.readinbackground = False



