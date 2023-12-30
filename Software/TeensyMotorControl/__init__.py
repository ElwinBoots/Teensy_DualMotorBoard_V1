import pandas as pd
import serial
from serial.tools import list_ports
import struct
import numpy as np
import time
import threading
import matplotlib.pyplot as plt
import make3 
import control as ct
import time

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
    self._readinbackground = False
    self._data = list()

    self._com, self._serialnumber = getTeensyPort(SERIAL_NUMBER)
    self.ser = serial.Serial( self._com , timeout=0.04)  # open serial port
    if self.ser.isOpen():
      print(f'Connected to Teensy on {self._com} with serial number: {self._serialnumber}.')
    # Get signal names, lengths and types

    self.ser.write(b'T')
    buffer = []
    while len(buffer) == 0:
      buffer = self.ser.readlines()

    types = [np.dtype('int8'), np.dtype('uint8'), np.dtype('int16'), np.dtype('uint16'), np.dtype(
        'int32'), np.dtype('uint32'), np.dtype('int64'), np.dtype('uint64'), np.dtype('float32'), np.dtype('float64')]
    self.signames = []
    for signame in buffer[::3]:
        self.signames.append(signame.strip().decode("utf-8"))
    self._siglengths = []
    for siglength in buffer[2::3]:
        self._siglengths.append(int(siglength.strip().decode("utf-8")))
    self._sigtypes = []
    self._sigbytes = []
    i = 0
    for sigtype in buffer[1::3]:
        self._sigtypes.append(types[int(sigtype.strip().decode("utf-8"))])
        self._sigbytes.append(self._sigtypes[-1].itemsize * self._siglengths[i])
        i += 1
    self._dtypes = []
    for i in range(len(self._sigtypes)):
        if self._siglengths[i] > 1:
            self._dtypes.append((self.signames[i],  self._siglengths[i] * self._sigtypes[i]))
        else:
            self._dtypes.append((self.signames[i],  self._sigtypes[i]))
    self._dtypessep = []
    for i in range(len(self._sigtypes)):
        for j in range(self._siglengths[i]):
            if self._siglengths[i] > 1:
                self._dtypessep.append((self.signames[i] + f'{[j]}',  self._sigtypes[i]))
            else:
                self._dtypessep.append((self.signames[i],  self._sigtypes[i]))
    self._dtypessep2 = []
    for i in range(len(self._sigtypes)):
        if self._siglengths[i] > 1:
            a = []
            for j in range(self._siglengths[i]):
                a.append((self.signames[i] + f'{[j]}',  self._sigtypes[i]))
            self._dtypessep2.append(a)
        else:
            self._dtypessep2.append((self.signames[i],  self._sigtypes[i]))
    self.Ts = self.getsig('motor.conf.Ts') / 1e6 #Note: more accurate than getting motor.conf.T
    self.fs = self.getsig('motor.conf.fs')
    self.setTrace(0)
    
    t = threading.Thread(target=self._serialEvent )
    t.start()
    
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
      data = np.array(value, self._sigtypes[signal]).tobytes()
      if(len(data) == self._sigbytes[signal]):
          self.ser.write(b'S' + struct.pack('I',  signal) + data)
          arr = np.ndarray(1, dtype=self._dtypes[signal][1],  buffer=data)
      else:
          raise ValueError(
              f'Size in bytes expected: {self._sigbytes[signal]}, given: {len(data)}.')
      return arr[0]
  
  def setparpart( self , signal, value, startlocation=0):
      signal = self.getsigid( signal )
      data = np.array(value, self._sigtypes[signal]).tobytes()
      length = len(data)
      if((length + startlocation * self._sigtypes[signal].itemsize) <= self._sigbytes[signal]):
          self.ser.write(b's' + struct.pack('I',  signal) + struct.pack('I',
                    startlocation) + struct.pack('I',  length) + data)
      else:
          raise ValueError(
              f'Max size in bytes expected: {self._sigbytes[signal]}, given: {len(data) + startlocation * self._sigtypes[signal].itemsize}.')
  
  
  def getsig( self, signal):
      self._readinbackground = False
      signal = self.getsigid( signal )
      if self._sigbytes[ signal ] < 1500: #Value not tuned
        self.ser.write(b'G' + struct.pack('I',  signal))
        buffer = self.ser.read(self._sigbytes[signal])
        arr = np.ndarray(1, dtype=self._dtypes[signal][1],  buffer=buffer)
        return arr[0]
      else:
        # buffer = []
        # length = 100
        # startlocation = 0
        # for i in range(0, int(np.ceil(self.siglengths[signal]/ length))):
        #   if startlocation + length > self.siglengths[signal]:
        #     length = self.siglengths[signal] - startlocation
        #   buffer.append(self.getsigpart(signal, startlocation, length))
        #   startlocation = startlocation + length
        # arr = np.ndarray(1, dtype=self._dtypes[signal][1],  buffer=buffer)
        return 0

  def getsigpart( self, signal, startlocation, length):
      signal = self.getsigid( signal )
      if((length + startlocation)*self._sigtypes[signal].itemsize <= self._sigbytes[signal]):
          self.ser.write(b'g' + struct.pack('I',  signal) + struct.pack('I',
                    startlocation) + struct.pack('I',  length * self._sigtypes[signal].itemsize))
          buffer = self.ser.read(self._sigbytes[signal])
          arr = np.ndarray(
              length, dtype=self._dtypes[signal][1].subdtype[0],  buffer=buffer)
          return arr
      else:
          raise ValueError(
              f'Requested values outside of array. Size array: {int(self._sigbytes[signal]/self._sigtypes[signal].itemsize)}, requested up to: {(length + startlocation) }.')

  def getallsig( self , maxbytes = 8 ):
      a = []
      names = []
      for i in range(len(self.signames)):
          if (self._sigbytes[i] <= maxbytes):
              names.append( self.signames[i] )
              a.append( self.getsig(self.signames[i]))
      df = pd.DataFrame( a , index=names ).T
      return df

  def setTrace( self , signals):
      maxsize = 50
      if isinstance(signals, str) or isinstance(signals, int):
          signals = [signals]
      if len(signals)>maxsize:
          raise ValueError(
              f'Max size {maxsize}, given: {len(signals)}.')
      self._tracesignalsnames = signals
      i = 0
      self._tracesignals = []
      signalsout = []
      self._tracebytes = 0;
      for signal in self._tracesignalsnames:
          signal = self.getsigid( signal )
          self._tracesignals.append(signal)
          signalsout.append(self.signames[signal])
          # ser.tracesigtypes[i] = ser.sigtypes[signal]
          self.ser.write(b't' + struct.pack('I', signal) + struct.pack('I', i))
          self._tracebytes += self._sigbytes[signal]
          i += 1
      if i < 30: #If not max number of signals, append null pointer to show end of pointers
        self.ser.write(b't' + struct.pack('I', 499) + struct.pack('I', i))      # Hardcoded high value, to be improved
      return signalsout
  

  def trace( self , t, downsample=1, outtype='df' ):
      self._readinbackground = False
      self.setpar('motor.conf.Ndownsample' , int( downsample ))
      self._fs_downsample = self.fs / downsample
      self.ser.timeout = max(1/self._fs_downsample * 1.2 , 0.04 ) #Note: changing timeout sometimes causes a glitch on the motor when running 60 kHz. Don't know why
      i = int(np.ceil(t * self._fs_downsample).astype('int') + 1)
      # self.ser.reset_input_buffer()
      self.ser.write(b'b' + struct.pack('I', i))
      buffer = self.ser.readall()
      if outtype == 'df':
          dtypestrace = []
          for isignal in self._tracesignals:
              if(isinstance(self._dtypessep2[isignal], list)):
                  for dtype in self._dtypessep2[isignal]:
                      dtypestrace.append(dtype)
              else:
                  dtypestrace.append(self._dtypessep2[isignal])
          arr = np.ndarray(i, dtype=dtypestrace,  buffer=buffer)
          df = pd.DataFrame(arr)
          df.index = np.linspace( 0 , len(df)-1 , len(df)) / self._fs_downsample
          df.index.name = 'Time [s]'
          return df
      elif outtype == 'arr':
          dtypestrace = [self._dtypes[j] for j in self._tracesignals]
          arr = np.ndarray(i, dtype=dtypestrace,  buffer=buffer)
          return arr

  def tracebg( self , t=-1 , downsample=1):
      self.setpar('motor.conf.Ndownsample' , int( downsample ))
      
      self._fs_downsample = self.fs / downsample
      self.ser.timeout = max(1/self._fs_downsample * 1.2 , 0.04 ) #Note: changing timeout sometimes causes a glitch on the motor when running 60 kHz. Don't know why
      i = int(np.ceil(t * self._fs_downsample).astype('int') + 1)
      if t == -1:
          i = int(2**32-1)
      else:
        i = int(np.ceil(t * self._fs_downsample).astype('int') + 1)
      self._readinbackground = True
      self.ser.write(b'b' + struct.pack('I', i))

  def stoptrace( self ):
      self.ser.write(b'b' + struct.pack('I', 0))

  def stoptracegetdata( self ):
      self.stoptrace()
      while len(self._data) == 0:
        pass
      return self.gettracedata()
      
  def gettracedata( self , outtype='df'):
      if len(self._data) == 0:
        print('No data available')
        return
      buffer = self._data.pop()
      i = int(len(buffer) / self._tracebytes)
      if outtype == 'df':
          dtypestrace = []
          for isignal in self._tracesignals:
              if(isinstance(self._dtypessep2[isignal], list)):
                  for dtype in self._dtypessep2[isignal]:
                      dtypestrace.append(dtype)
              else:
                  dtypestrace.append(self._dtypessep2[isignal])
          arr = np.ndarray(i, dtype=dtypestrace,  buffer=buffer)
          df = pd.DataFrame(arr)
          df.index = np.linspace( 0 , len(df)-1 , len(df)) / self._fs_downsample
          df.index.name = 'Time [s]'
          return df
      elif outtype == 'arr':
          dtypestrace = [self._dtypes[j] for j in self._tracesignals]
          arr = np.ndarray(i, dtype=dtypestrace,  buffer=buffer)
          return arr

  def _serialEvent( self ):
      while True:
        while self._readinbackground is True:
          if self.ser.in_waiting > 0:
            self._data.append(self.ser.readall())
            print( 'New data available' )
          time.sleep(0.001)
        time.sleep(0.001)
      return

  def disconnect(self):
      print('Closing connection')
      self._readinbackground = False
      self.ser.close()

  def connect(self ):
    self.ser.open()

  def __del__(self):
    self.disconnect()
    
  # Tools
  def bode(  self, H, f, name='Data', title='bode plot'):
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
  
  
  def nyquist(  self, H ,f  , name = '' , title = 'nyquist plot'):
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
  
  
  def getFFTdf(  self, df, Ndownsample, j0=60000):
      #    j0 = int(1/self.Ts) # These amount of samples are ignored
      L = 2047 * Ndownsample
      Naver = int((len(df)-j0)/L)
      # print( 'Naver =' , Naver )
      SIGNAL = np.fft.fft(df.iloc[j0:j0+L*Naver], axis=0)
  
      f = np.fft.fftfreq(L*Naver, self.Ts)
      SIGNAL = SIGNAL[f > 0]
      SIGNAL = 2*SIGNAL[Naver-1::Naver]
      f = f[f > 0]
      f = f[Naver-1::Naver]
      SIGNAL = SIGNAL[f < 1/(2*self.Ts*Ndownsample)]
      f = f[f < 1/(2*self.Ts*Ndownsample)]
      dfout = pd.DataFrame(SIGNAL)
      dfout.index = f
      dfout.index.name = 'Frequency [Hz]'
      dfout.columns = df.columns
      if 'motor.state1.dist' in dfout:
          dfout = dfout.div(dfout['motor.state1.dist'], axis=0)
      if 'motor.state2.dist' in dfout:  
          dfout = dfout.div(dfout['motor.state2.dist'], axis=0)
      return dfout
  
  
  def getFFT(  self, signal, Ndownsample, j0=60000):
      #    j0 = int(1/self.Ts) # These amount of samples are ignored
      L = 2047 * Ndownsample
      Naver = int((len(signal)-j0)/L)
      # print( 'Naver =' , Naver )
      SIGNAL = np.fft.fft(signal[j0:j0+L*Naver], axis=0)
      f = np.fft.fftfreq(L*Naver, self.Ts)
      SIGNAL = SIGNAL[f > 0]
      SIGNAL = 2*SIGNAL[Naver-1::Naver]
      f = f[f > 0]
      f = f[Naver-1::Naver]
      SIGNAL = SIGNAL[f < 1/(2*self.Ts*Ndownsample)]
      f = f[f < 1/(2*self.Ts*Ndownsample)]
      return SIGNAL, f
  
  def prepSP(  self,  p , v , a , j, axis=1):
      t1, t2, t3, jd = make3.make3( p , v , a , j , self.Ts );
      #double self.Tstart, double t1, double t2, double t3, double p, double v_max, double a_max, double j_max
      if axis == 1:
          self.ser.write( b'1' + struct.pack('f',  t1) + struct.pack('f',  t2) + struct.pack('f',  t3) + struct.pack('d',  p) + struct.pack('f',  v) + struct.pack('f',  a)  + struct.pack('f',  jd) )
      else:
          self.ser.write( b'2' + struct.pack('f',  t1) + struct.pack('f',  t2) + struct.pack('f',  t3) + struct.pack('d',  p) + struct.pack('f',  v) + struct.pack('f',  a)  + struct.pack('f',  jd) )
      return t1, t2, t3, jd
  
  def setNotch( self, axis, index, f0, debth_db, width ):
      if index < 8:
          self.ser.write( b'N' + struct.pack('I',  axis) + struct.pack('I',  index) + struct.pack('f',  f0) + struct.pack('f',  debth_db) + struct.pack('f',  width) )
  
  def setLowpass( self, axis, index, f0, damping ):
      if index < 8:
          self.ser.write( b'L' + struct.pack('I',  axis) + struct.pack('I',  index) + struct.pack('f',  f0) + struct.pack('f',  damping) )
  
  
  def discrete_notch(  self, f0, debth_db, width):
      w0 = 2 * np.pi * f0 * self.Ts;
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
      
      return ct.TransferFunction( [b0, b1, b2] , [a0, a1, a2]  , float(self.Ts) )
      
  def discrete_lowpass(  self, f0, damping ):
      w0 = 2 * np.pi * f0 * self.Ts;
      alpha = np.sin(w0) * damping;
      b0 = 0.5 * (1 - np.cos(w0)) / (1 + alpha) ;
      b1 = 2 * b0;
      b2 = b0 ;
      a0 = 1;
      a1 = (-2 * np.cos(w0)) / (1 + alpha);
      a2 = (1 - alpha) / (1 + alpha);
      return ct.TransferFunction( [b0, b1, b2] , [a0, a1, a2]  , float(self.Ts) )
  
  def vel( self, vel = 0 , motor = 0 ):
      if type(vel) == list:
          vel1 = vel[0]
          vel2 = vel[1]
      else:
          vel1 = vel
          vel2 = vel
      if motor == 0 or motor == 1:
          self.setpar('motor.state1.offsetVel' ,  vel1 )
      if motor == 0 or motor == 2:
          self.setpar('motor.state2.offsetVel' ,  vel2 )
          
  def pos( self, target = 0 , motor = 0 , vel = 100 , acc = 1000):
      if type(target) == list:
          target1 = target[0]
          target2 = target[1]
      else:
          target1 = target
          target2 = target
      if motor == 0 or motor == 1:
          enc2rad1 = self.getsig('c1.enc2rad')
          target1 = round(target1/360*2*np.pi/enc2rad1)*enc2rad1
          delta = -self.getsig('motor.state1.rmech') + target1
          if delta !=0:
              self.prepSP( abs(delta) , vel , acc ,500000 , 1)
              self.setpar('motor.state1.SPdir' , delta>0)
              self.setpar('motor.state1.spNgo', 1)
      if motor == 0 or motor == 2:
          enc2rad2 = self.getsig('c2.enc2rad')
          target2 = round(target2/360*2*np.pi/enc2rad2)*enc2rad2
          delta = -self.getsig('motor.state2.rmech') + target2
          if delta !=0:
              self.prepSP( abs(delta) , vel , acc ,500000 , 2)
              self.setpar('motor.state2.SPdir' , delta>0)
              self.setpar('motor.state2.spNgo', 1)
  
  def pos_wait( self, target = 0 , motor = 0 , vel = 100 , acc = 1000):
      self.pos( target , motor , vel , acc)
      while (self.getsig('motor.state1.REFstatus') > 0  or self.getsig('motor.state2.REFstatus') > 0 ):
        time.sleep(1e-3)
  
  def rel( self, rel = 0 , motor = 0 ):
      if type(rel) == list:
          rel1 = rel[0]
          rel2 = rel[1]
      else:
          rel1 = rel
          rel2 = rel
      if motor == 0 or motor == 1:
          enc2rad1 = self.getsig('c1.enc2rad')
          delta = round(rel1/360*2*np.pi/enc2rad1)*enc2rad1
          if delta !=0:
              self.prepSP( abs(delta) , 100 , 1000 ,500000 , 1)
              self.setpar('motor.state1.SPdir' , delta>0)
              self.setpar('motor.state1.spNgo', 1)
      if motor == 0 or motor == 2:
          enc2rad2 = self.getsig('c2.enc2rad')
          delta = round(rel2/360*2*np.pi/enc2rad2)*enc2rad2
          if delta !=0:
              self.prepSP( abs(delta) , 100 , 1000 ,500000 , 2)
              self.setpar('motor.state2.SPdir' , delta>0)
              self.setpar('motor.state2.spNgo', 1)
          
  
  def CL_cur( self, f_bw = 2e3 , axis = 0 ):
      f_lp = f_bw*4
      f_lp_2nd = f_bw*8
      f_lp_2nd_damp = 0.5
      
      if axis == 1 or axis == 0:
        Lq = self.getsig('motor.conf1.Lq')
        Ld = self.getsig('motor.conf1.Ld')
        R = self.getsig('motor.state1.R')

        self.setpar('motor.conf1.Kp_iq', Lq * f_bw * 2 * np.pi)  # Current loop Kp
        self.setpar('motor.conf1.Ki_iq', R/Lq)  # Current loop Ki
        self.setpar('motor.conf1.Kp_id', Ld * f_bw * 2 * np.pi)  # Current loop Kp
        self.setpar('motor.conf1.Ki_id', R/Ld)  # Current loop Ki
        lowpass_c = 1-np.exp(-f_lp*2*np.pi*self.Ts)
        self.setpar( 'motor.conf1.lowpass_Vd_c' , lowpass_c )
        self.setpar( 'motor.conf1.lowpass_Vq_c' , lowpass_c )
        
        self.setpar('motor.conf1.I_max', 15)
        self.setpar('motor.conf1.maxDutyCycle', 0.99)
        
        self.setpar('motor.conf1.enc_transmission' , 1)
      
        self.setLowpass( 1 , 4, f_lp_2nd, f_lp_2nd_damp )
        self.setLowpass( 1 , 5, f_lp_2nd, f_lp_2nd_damp )
      if axis == 2 or axis == 0:
        Lq = self.getsig('motor.conf2.Lq')
        Ld = self.getsig('motor.conf2.Ld')
        R = self.getsig('motor.state2.R')
        self.setpar('motor.conf2.Kp_iq', Lq * f_bw * 2 * np.pi)  # Current loop Kp
        self.setpar('motor.conf2.Ki_iq', R/Lq)  # Current loop Ki
        self.setpar('motor.conf2.Kp_id', Ld * f_bw * 2 * np.pi)  # Current loop Kp
        self.setpar('motor.conf2.Ki_id', R/Ld)  # Current loop Ki
        lowpass_c = 1-np.exp(-f_lp*2*np.pi*self.Ts)
        self.setpar( 'motor.conf2.lowpass_Vd_c' , lowpass_c )
        self.setpar( 'motor.conf2.lowpass_Vq_c' , lowpass_c )
        
        self.setpar('motor.conf2.I_max', 15)
        self.setpar('motor.conf2.maxDutyCycle', 0.99)
        
        self.setpar('motor.conf2.enc_transmission' , 1)
        
        self.setLowpass( 2 , 4, f_lp_2nd, f_lp_2nd_damp )
        self.setLowpass( 2 , 5, f_lp_2nd, f_lp_2nd_damp )
  
  def CL( self, cont = 2 , axis = 0 , J = 7.5e-5):
    
      if axis == 1 or axis == 0:
        self.setpar('c1.maxerror',0.5)
        # CONF1
        self.setLowpass( 1 , 0, 4000, 0.7 )
        self.setLowpass( 1 , 1, 0, 0.3 )
        self.setLowpass( 1 , 2, 0, 0.3 )
        self.setLowpass( 1 , 3, 0, 0.3 )
        # setNotch( 1 , 1, 590, -20, 0.1 )
      
      if axis == 2 or axis == 0:
        self.setpar('c2.maxerror',0.5)
        # CONF2
        self.setLowpass( 2 , 0, 4000, 0.7 )
        self.setLowpass( 2 , 1, 0, 0.3 )
        self.setLowpass( 2 , 2, 0, 0.3 )
        self.setLowpass( 2 , 3, 0, 0.3 )
        # setNotch( 2 , 1, 590, -20, 0.1 )
      
      if cont == 0:
          BW = 0
          alpha_i = 0 
          alpha_1 = 3
          alpha_2 = 3
      elif cont == 1:
          BW = 10
          alpha_i = 0
          alpha_1 = 3
          alpha_2 = 3
          if axis == 1 or axis == 0:
            self.setpar('c1.maxerror',np.pi)
            self.setLowpass( 1 , 0, 150, 0.6 )
          if axis == 2 or axis == 0:
            self.setpar('c2.maxerror',np.pi)
            self.setLowpass( 2 , 0, 150, 0.6 )  
  
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
          if axis == 1 or axis == 0:
            self.setNotch( 1 , 1, 590, -20, 0.1 )
          if axis == 2 or axis == 0:
            self.setNotch( 2 , 1, 590, -20, 0.1 )
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
          
  
      elif cont == 6:
          BW = 250
          alpha_i = 2.5
          alpha_1 = 2.5
          alpha_2 = 7
          if axis == 1 or axis == 0:
            self.setLowpass( 1 , 1, 8000, 0.4 )
          if axis == 2 or axis == 0:
            self.setLowpass( 2 , 1, 8000, 0.4 )
            
      elif cont == 7:
          BW = 20
          alpha_i = 6
          alpha_1 = 3
          alpha_2 = 3
          if axis == 1 or axis == 0:
            self.setLowpass( 1 , 0, 150, 0.6 )
          if axis == 2 or axis == 0:
            self.setLowpass( 2 , 0, 150, 0.6 )  
          
      else:
          BW = 50
          alpha_i = 6
          alpha_1 = 3
          alpha_2 = 10
          
      
      gain_at_BW = J * (BW*2*np.pi)**2
      
      if BW > 0:
          if alpha_i > 0:
              Ki = BW*2*np.pi*self.Ts/alpha_i
          else:
              Ki = 0
          if alpha_1 > 0:
              Kd = alpha_1/(self.Ts*BW*2*np.pi)
          else:
              Kd = 0
          if alpha_2 > 0:
              lowpass_c = 1-np.exp(-BW*2*np.pi*alpha_2*self.Ts)
          else:
              lowpass_c =1
          Kp = gain_at_BW * abs(lowpass_c + np.exp(np.pi*BW*self.Ts*2j) - 1) / (abs(lowpass_c*((Ki*np.exp(np.pi*BW*self.Ts*2j)) / (np.exp(np.pi*BW*self.Ts*2j) - 1) + 1) * (Kd - Kd*np.exp(-np.pi*BW*self.Ts*2j) + 1) )) 
      else:
          Kp = 0
          Ki = 0
          Kd = 0
          lowpass_c = 1
    
      self.setpar( 'motor.conf1.Command' , 2 ) #Reset error
  
      if axis == 1 or axis == 0:
        self.setpar( 'motor.conf1.Kp_prep' , Kp )
        self.setpar( 'motor.conf1.Ki_prep' , Ki )
        self.setpar( 'motor.conf1.Kd_prep' , Kd )
        self.setpar( 'motor.conf1.lowpass_c_prep' , lowpass_c )
        
        
        self.setpar( 'motor.conf1.Command' , 1 ) #Activate controller
        self.setpar('motor.state1.Jload' , J)
  
      # CONF2
      #J = 3.25e-5
      gain_at_BW = J * (BW*2*np.pi)**2
      # alpha_i = 6
      # alpha_1 = 3
      # alpha_2 = 3
      
      if BW > 0:
          if alpha_i > 0:
              Ki = BW*2*np.pi*self.Ts/alpha_i
          else:
              Ki = 0
          if alpha_1 > 0:
              Kd = alpha_1/(self.Ts*BW*2*np.pi)
          else:
              Kd = 0
          if alpha_2 > 0:
              lowpass_c = 1-np.exp(-BW*2*np.pi*alpha_2*self.Ts)
          else:
              lowpass_c =1
          Kp = gain_at_BW * abs(lowpass_c + np.exp(np.pi*BW*self.Ts*2j) - 1) / (abs(lowpass_c*((Ki*np.exp(np.pi*BW*self.Ts*2j)) / (np.exp(np.pi*BW*self.Ts*2j) - 1) + 1) * (Kd - Kd*np.exp(-np.pi*BW*self.Ts*2j) + 1) )) 
      else:
          Kp = 0
          Ki = 0
          Kd = 0
          lowpass_c = 1
          
      if axis == 2 or axis == 0:
        self.setpar( 'motor.conf2.Kp_prep' , Kp )
        self.setpar( 'motor.conf2.Ki_prep' , Ki )
        self.setpar( 'motor.conf2.Kd_prep' , Kd )
        self.setpar( 'motor.conf2.lowpass_c_prep' , lowpass_c )
        
        self.setpar( 'motor.conf2.Command' , 1 ) #Activate controller     
        self.setpar('motor.state2.Jload' , J)
      
  def fftpsd( self, signal , name = '' , Ts=0 ):
      if Ts == 0:
        Ts = self.Ts
      L = len(signal)
      SIGNAL = np.fft.fft( signal , axis = 0) / L
      f  = np.fft.fftfreq(L, self.Ts)
      SIGNAL = 2 * SIGNAL[f>0] 
      f = f[f>0]   
  
      ax1 = plt.subplot(2,1,1)
      plt.loglog( f , abs(SIGNAL)**2 /self.Ts )
      plt.grid( 1 , 'both' , 'both')
      plt.ylabel('PSD [-²/Hz]')
      # plt.title(title)
      plt.legend()
      ax2 = plt.subplot(2,1,2, sharex=ax1)
      plt.semilogx( f , np.cumsum(abs(SIGNAL)**2 /self.Ts) *self.Ts )
      plt.grid( 1 , 'both')
      plt.xlabel('Frequency [Hz]')
      plt.ylabel('CPS [-²]')
      plt.tight_layout()
      plt.show()   
      return SIGNAL, f
    
    
    
    
    
    
    
     
    
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
    print(f'Teensy on {self._mot._com}, serial number: {self._mot._serialnumber}.')
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




if __name__ == "__main__":
  m = Motor(  )
  motor = MotorVariables( m )


