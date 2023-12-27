# -*- coding: utf-8 -*-
"""
Created on Sat Oct  7 14:51:16 2023

@author: elwin
"""

Fplayback = round(1/m.Ts)
N_samples_per_write = 100
bufsize = 10000
musicgain = 4

import scipy

#%%
(Fs , data) = scipy.io.wavfile.read("AC⧸DC - Thunderstruck (Official Video).wav")

stream = data[200*Fs:300*Fs]
stream = scipy.signal.resample(stream , int(len(stream)/Fs*Fplayback) ) 
#%%
(Fs , data) = scipy.io.wavfile.read("#Stereo： Left and Right Stereo Sound Test.wav")

stream = data[6*Fs:]
stream = scipy.signal.resample(stream , int(len(stream)/Fs*Fplayback) ) 
#%%
(Fs , data) = scipy.io.wavfile.read("8D Audio Bass Test !!.wav")

stream = data
stream = scipy.signal.resample(stream , int(len(stream)/Fs*Fplayback) ) 
#%%
(Fs , data) = scipy.io.wavfile.read("Martin Garrix - Animals (Official Video).wav")

stream = data[77*Fs:300*Fs]
stream = scipy.signal.resample(stream , int(len(stream)/Fs*Fplayback) ) 
#%%
(Fs , data) = scipy.io.wavfile.read("Jedi Mind Tricks Presents： Army Of The Pharaohs - ＂Dump The Clip＂ [Official Audio].wav")

stream = data[8*Fs:]
stream = scipy.signal.resample(stream , int(len(stream)/Fs*Fplayback) ) 
#%%
(Fs , data) = scipy.io.wavfile.read("Eminem, Dr. Dre - Forgot About Dre (Explicit) (Official Music Video) ft. Hittman.wav")

stream = data
stream = scipy.signal.resample(stream , int(len(stream)/Fs*Fplayback) ) 
#%%
(Fs , data) = scipy.io.wavfile.read("Lights On ▶ FNAF SECURITY BREACH SONG.wav")

stream = data
stream = scipy.signal.resample(stream , int(len(stream)/Fs*Fplayback) ) 





#%%
stream = stream / np.max(np.abs(stream))
L = len(stream)

m.setpar('s1.runstream' , 0)
m.setpar('s1.curbuffer' , 0)
m.setpar('s2.runstream' , 0)
m.setpar('s2.curbuffer' , 0)

for i in range( int(bufsize /N_samples_per_write) ):
   m.setparpart( 's1.streambuffer' ,  stream[ i*N_samples_per_write : (i+1)*N_samples_per_write , 1] , startlocation = np.mod(i*N_samples_per_write , bufsize ) )
   m.setparpart( 's2.streambuffer' ,  stream[ i*N_samples_per_write : (i+1)*N_samples_per_write , 0] , startlocation = np.mod(i*N_samples_per_write , bufsize ) )

m.setpar('s1.runstream' , 1)
m.setpar('s2.runstream' , 1)
m.setpar('s1.buffergain' , musicgain)
m.setpar('s2.buffergain' , musicgain)

# vel([-50 , 50])
try:
    end = int( (L-bufsize) /N_samples_per_write)
    for i in range( end ):
        # if i == int(end/6):
        #     vel([50 , -50])
        # if i == int(end/3):
        #     vel([-50 , 50])
        # if i == int(end/2):
        #     vel([0 , 0])
        startloc = np.mod(i*N_samples_per_write , bufsize )
        while (startloc  <= m.getsig('s1.curbuffer') <= startloc + N_samples_per_write):
            pass
        m.setparpart( 's1.streambuffer' ,  stream[ i*N_samples_per_write + bufsize: (i+1)*N_samples_per_write + bufsize , 1] , startlocation = startloc )
        m.setparpart( 's2.streambuffer' ,  stream[ i*N_samples_per_write + bufsize: (i+1)*N_samples_per_write + bufsize , 0] , startlocation = startloc )
        print((i+1)*N_samples_per_write + bufsize)
except KeyboardInterrupt:
    time.sleep(0.1)   
    m.ser.flushInput()  
    print('Canceled')
      
  
m.setpar('s1.buffergain' , 0)
m.setpar('s2.buffergain' , 0)
m.setpar('s1.runstream' , 0)
m.setpar('s2.runstream' , 0)
