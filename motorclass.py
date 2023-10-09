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


import sys, serial
from serial.tools import list_ports

# Teensy USB serial microcontroller program id data:
VENDOR_ID = "16C0"
PRODUCT_ID = "0483"

def getTeensyPort( SERIAL_NUMBER=''  ):
	for port in list(list_ports.comports()):
		if port[2].startswith("USB VID:PID=%s:%s SER=%s"%(VENDOR_ID, PRODUCT_ID, SERIAL_NUMBER)):
			SER = port[2].split('SER=')[-1]
			return port[0], SER
	if SERIAL_NUMBER=='':
		raise ConnectionError('No Teensy found')
	else:
		raise ConnectionError(f'No Teensy found with a serial numbers starting with {SERIAL_NUMBER}')
  
class Motor:
  def __init__( self , SERIAL_NUMBER='' ):
    self.com, self.serialnumber = getTeensyPort(SERIAL_NUMBER)
    self.ser = serial.Serial( self.com , timeout=0.1)  # open serial port
    if self.ser.isOpen():
      print(f'Connected to Teensy on {self.com} with serial number: {self.serialnumber}.')
    # self.ser.flush()
    # time.sleep(0.1)
    # Get signal names, lengths and types
    self.ser.write(b'T')
    buffer = self.ser.readlines()

    types = [np.dtype('int8'), np.dtype('uint8'), np.dtype('int16'), np.dtype('uint16'), np.dtype(
        'int32'), np.dtype('uint32'), np.dtype('int64'), np.dtype('uint64'), np.dtype('float32'), np.dtype('float64')]
    self.signames = []
    for signame in buffer[::3]:
        self.signames.append(signame.strip().decode("utf-8"))
    self.siglengths = []
    for siglength in buffer[2::3]:
        self.siglengths.append(int(siglength.strip().decode("utf-8")))
    self.sigtypes = []
    self.sigbytes = []
    i = 0
    for sigtype in buffer[1::3]:
        self.sigtypes.append(types[int(sigtype.strip().decode("utf-8"))])
        self.sigbytes.append(self.sigtypes[-1].itemsize * self.siglengths[i])
        i += 1
    self.dtypes = []
    for i in range(len(self.sigtypes)):
        if self.siglengths[i] > 1:
            self.dtypes.append((self.signames[i],  self.siglengths[i] * self.sigtypes[i]))
        else:
            self.dtypes.append((self.signames[i],  self.sigtypes[i]))
    self.dtypessep = []
    for i in range(len(self.sigtypes)):
        for j in range(self.siglengths[i]):
            if self.siglengths[i] > 1:
                self.dtypessep.append((self.signames[i] + f'{[j]}',  self.sigtypes[i]))
            else:
                self.dtypessep.append((self.signames[i],  self.sigtypes[i]))
    self.dtypessep2 = []
    for i in range(len(self.sigtypes)):
        if self.siglengths[i] > 1:
            a = []
            for j in range(self.siglengths[i]):
                a.append((self.signames[i] + f'{[j]}',  self.sigtypes[i]))
            self.dtypessep2.append(a)
        else:
            self.dtypessep2.append((self.signames[i],  self.sigtypes[i]))
    self.Ts = self.getsig('motor.conf.T')
    self.setTrace(0)
    
  def getsigid( self , signal ):
      if isinstance(signal, list):
          signal = signal[0]
      if isinstance(signal, str):
          if signal[0] == 's':
              signal = 'motor.state' + signal[1:]
          if signal[0] == 'c':
              signal = 'motor.conf' + signal[1:]
          signal = self.signames.index(signal)
      return signal
  
  def setpar( self , signal, value):
      signal = self.getsigid( signal )
      data = np.array(value, self.sigtypes[signal]).tobytes()
      if(len(data) == self.sigbytes[signal]):
          self.ser.write(b'S' + struct.pack('I',  signal) + data)
          arr = np.ndarray(1, dtype=self.dtypes[signal][1],  buffer=data)
      else:
          raise ValueError(
              f'Size in bytes expected: {self.sigbytes[signal]}, given: {len(data)}.')
      return arr[0]
  
  def setparpart( self , signal, value, startlocation=0):
      signal = self.getsigid( signal )
      data = np.array(value, self.sigtypes[signal]).tobytes()
      length = len(data)
      if((length + startlocation * self.sigtypes[signal].itemsize) <= self.sigbytes[signal]):
          self.ser.write(b's' + struct.pack('I',  signal) + struct.pack('I',
                    startlocation) + struct.pack('I',  length) + data)
      else:
          raise ValueError(
              f'Max size in bytes expected: {self.sigbytes[signal]}, given: {len(data) + startlocation * self.sigtypes[signal].itemsize}.')
  
  
  def getsig( self, signal):
      signal = self.getsigid( signal )
      if self.sigbytes[ signal ] < 1500: #Value not tuned
        self.ser.write(b'G' + struct.pack('I',  signal))
        buffer = self.ser.read(self.sigbytes[signal])
        arr = np.ndarray(1, dtype=self.dtypes[signal][1],  buffer=buffer)
        return arr[0]
      else:
        # buffer = []
        # length = 100
        # startlocation = 0
        # for i in range(0, int(np.ceil(m.siglengths[signal]/ length))):
        #   if startlocation + length > m.siglengths[signal]:
        #     length = m.siglengths[signal] - startlocation
        #   buffer.append(self.getsigpart(signal, startlocation, length))
        #   startlocation = startlocation + length
        # arr = np.ndarray(1, dtype=self.dtypes[signal][1],  buffer=buffer)
        return 0

  def getsigpart( self, signal, startlocation, length):
      signal = self.getsigid( signal )
      if((length + startlocation)*self.sigtypes[signal].itemsize <= self.sigbytes[signal]):
          self.ser.write(b'g' + struct.pack('I',  signal) + struct.pack('I',
                    startlocation) + struct.pack('I',  length * self.sigtypes[signal].itemsize))
          buffer = self.ser.read(self.sigbytes[signal])
          arr = np.ndarray(
              length, dtype=self.dtypes[signal][1].subdtype[0],  buffer=buffer)
          return arr
      else:
          raise ValueError(
              f'Requested values outside of array. Size array: {int(self.sigbytes[signal]/self.sigtypes[signal].itemsize)}, requested up to: {(length + startlocation) }.')

  def getallsig( self , maxbytes = 8 ):
      a = []
      names = []
      for i in range(len(self.signames)):
          if (self.sigbytes[i] <= maxbytes):
              names.append( self.signames[i] )
              a.append( self.getsig(self.signames[i]))
      df = pd.DataFrame( a , index=names ).T
      return df

  def setTrace( self , signals, downsample=1):
      maxsize = 50
      if isinstance(signals, str) or isinstance(signals, int):
          signals = [signals]
      if len(signals)>maxsize:
          raise ValueError(
              f'Max size {maxsize}, given: {len(signals)}.')
      self.tracesignalsnames = signals
      i = 0
      self.tracesignals = []
      signalsout = []
      self.tracebytes = 0;
      for signal in self.tracesignalsnames:
          signal = self.getsigid( signal )
          self.tracesignals.append(signal)
          signalsout.append(self.signames[signal])
          # ser.tracesigtypes[i] = ser.sigtypes[signal]
          self.ser.write(b't' + struct.pack('I', signal) + struct.pack('I', i))
          self.tracebytes += self.sigbytes[signal]
          i += 1
      if i < 30: #If not max number of signals, append null pointer to show end of pointers
        self.ser.write(b't' + struct.pack('I', 499) + struct.pack('I', i))      # Hardcoded high value, to be improved
      self.setpar('motor.conf.Ndownsample' , int( downsample ))
      return signalsout
  
  
  def trace( self , t, outtype='df'):
      Ts_downsample = self.Ts * self.getsig('motor.conf.Ndownsample')
      i = int(np.ceil(t/Ts_downsample).astype('int') + 1)
      self.ser.reset_input_buffer()
      self.ser.write(b'b' + struct.pack('I', i))
      buffer = self.ser.readall()
      if outtype == 'df':
          dtypestrace = []
          for isignal in self.tracesignals:
              if(isinstance(self.dtypessep2[isignal], list)):
                  for dtype in self.dtypessep2[isignal]:
                      dtypestrace.append(dtype)
              else:
                  dtypestrace.append(self.dtypessep2[isignal])
          arr = np.ndarray(i, dtype=dtypestrace,  buffer=buffer)
          df = pd.DataFrame(arr)
          df.index = np.linspace( 0 , len(df)-1 , len(df)) * Ts_downsample
          df.index.name = 'Time [s]'
          return df
      elif outtype == 'arr':
          dtypestrace = [self.dtypes[j] for j in self.tracesignals]
          arr = np.ndarray(i, dtype=dtypestrace,  buffer=buffer)
          return arr

  def disconnect(self):
      print('Closing connection')
      self.ser.close()

  def connect(self ):
    self.ser.open()

  def __del__(self):
    self.disconnect()
     
    
class MotorVariables:
  def __init__( self , mot ):
    self._mot = mot
    self.conf = SubVariable( mot , 'motor.conf.')
    self.conf1 = SubVariable( mot , 'motor.conf1.')
    self.conf2 = SubVariable( mot , 'motor.conf2.')
    self.state = SubVariable( mot , 'motor.state.')
    self.state1 = SubVariable( mot , 'motor.state1.')
    self.state2 = SubVariable( mot , 'motor.state2.')
    for signame in self._mot.signames:
      exec( 'self.' + signame.split('.',1)[-1] )

  def __repr__(self):
    print( self.conf )
    print( self.conf1 )
    print( self.conf2 )
    print( self.state )
    print( self.state1 )
    print( self.state2 )
    print(f'Teensy on {self._mot.com}, serial number: {self._mot.serialnumber}.')
    return ""
  
  def __str__(self):
    self.__repr__()
    return ""


class SubVariable:
  def __init__( self , mot , location):
    self.__dict__.update(locals())
   
  def __repr__(self):
    keys = list(self.__dict__.keys())
    keys_sub_set = sorted([x[1:] for x in keys if '_' in x[0]])
    length = len(max(keys_sub_set, key = len) )
    for key in keys_sub_set:
      print( self.location + key + ' '*(length-len(key)) + ' = ' + str(self.__getattr__(key))  )
    return ""
  def __str__(self):
    self.__repr__()
    return ""

  def __getattr__(self, name: str):
    self.__dict__[f"_{name}"] = self.mot.getsig( self.location + name )
    return self.__dict__[f"_{name}"]

  def __setattr__(self, name, value):
    self.__dict__[f"_{name}"] = self.mot.setpar( self.location + name , value)
     
  def __dir__(self):
    keys = list(self.__dict__.keys())
    keys_removed = [x[1:] for x in keys if '_' in x[0]]    
    return sorted(keys_removed)


m = Motor(  )
motor = MotorVariables( m )



