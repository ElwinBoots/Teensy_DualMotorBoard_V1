#%%
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication, QGridLayout, QHBoxLayout, QWidget, QPushButton, QLineEdit, QSlider
from PyQt5.QtCore import QObject, pyqtSignal

pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')

app = QApplication([])
win = pg.GraphicsLayoutWidget()
layout = QGridLayout()

plot1 = pg.plot()
plot2 = pg.plot()
plot3 = pg.plot()

plot1.showGrid(x = True, y = True, alpha = 0.3)                                        
plot2.showGrid(x = True, y = True, alpha = 0.3)                                        
plot3.showGrid(x = True, y = True, alpha = 0.3)                                        

plot2.setXLink(plot1)
plot3.setXLink(plot1)

maxdata = 1000

plot1.plotItem.getViewBox().setMouseMode(pg.ViewBox.RectMode)
plot2.plotItem.getViewBox().setMouseMode(pg.ViewBox.RectMode)
plot3.plotItem.getViewBox().setMouseMode(pg.ViewBox.RectMode)
plot1.setXRange(0,maxdata)
plot2.setXRange(0,maxdata)
plot3.setXRange(0,maxdata)

layout.addWidget(plot1, 0, 0)
layout.addWidget(plot2, 1, 0)
layout.addWidget(plot3, 2, 0)

col = [[0, 114.4320, 189.6960],
[217.6000,  83.2000, 25.0880],
[237.8240, 177.6640, 32.0000],
[126.4640,  47.1040, 142.3360],
[119.2960, 172.5440, 48.1280],
[77.0560, 190.7200, 238.8480],
[162.5600,  19.9680, 47.1040],
[0, 114.4320, 189.6960]]

width = 1
curve1a = plot1.plot(pen=pg.mkPen(color=col[0], width=width))
curve1b = plot1.plot(pen=pg.mkPen(color=col[1], width=width))
curve1c = plot1.plot(pen=pg.mkPen(color=col[2], width=width))
curve1d = plot1.plot(pen=pg.mkPen(color=col[3], width=width))
curve2a = plot2.plot(pen=pg.mkPen(color=col[0], width=width))
curve2b = plot2.plot(pen=pg.mkPen(color=col[1], width=width))
curve3a = plot3.plot(pen=pg.mkPen(color=col[0], width=width))
curve3b = plot3.plot(pen=pg.mkPen(color=col[1], width=width))
curve3c = plot3.plot(pen=pg.mkPen(color=col[2], width=width))

win.resize( 1000, 700)
win.setLayout(layout)
win.show()
# win.showFullScreen()

import threading
from collections import deque

y1a = deque()
y1b = deque()
y1c = deque()
y1d = deque()
y2a = deque()
y2b = deque()
y3a = deque()
y3b = deque()
y3c = deque()

def update(data1 , data2 , data3, data4 , data5 , data6 , data7 , data8 , data9 ):
    y1a.extend( [data1] )
    y1b.extend( [data2] )
    y1c.extend( [data3] )
    y1d.extend( [data4] )
    y2a.extend( [data5] )
    y2b.extend( [data6] )
    y3a.extend( [data7] )
    y3b.extend( [data8] )
    y3c.extend( [data9] )
    while len(y1a) > maxdata:
        y1a.popleft() #remove oldest
        y1b.popleft() #remove oldest
        y1c.popleft() #remove oldest
        y1d.popleft() #remove oldest
        y2a.popleft() #remove oldest
        y2b.popleft() #remove oldest
        y3a.popleft() #remove oldest
        y3b.popleft() #remove oldest
        y3c.popleft() #remove oldest
    curve1a.setData( y=y1a)
    curve1b.setData( y=y1b)
    curve1c.setData( y=y1c)
    curve1d.setData( y=y1d)
    curve2a.setData( y=y2a)
    curve2b.setData( y=y2b)
    curve3a.setData( y=y3a)
    curve3b.setData( y=y3b)
    curve3c.setData( y=y3c)
    return

class Thread(pg.QtCore.QThread):
    def startdata(self, signals):
        signals = setTrace(signals)
        global dtypestrace, buffer
        dtypestrace = [dtypes[j] for j in ser.signals]
        buffer = bytearray(int(ser.tracebytes ))
        # setpar('motor.conf.Ndownsample' , int( 1/Ts ))
        self.stopdata()
        setpar('motor.conf.Ndownsample' , int( 0.01/Ts ))
        ser.write(b'b' + struct.pack('I',  int(2**32-1)))
    
    def stopdata(self):
        ser.write(b'b' + struct.pack('I',  int(0)))
        bla = ser.readall()
        setpar('motor.conf.Ndownsample' , int( 1 ))
    
    # newData = pg.QtCore.Signal(object)
    newData = pg.QtCore.Signal(float , float , float , float , float , float, float , float, float)
    def run(self):
        while not win.isHidden():
            while ser.in_waiting < len(buffer):
                bla = 1
            ser.readinto(buffer)
            arr = np.ndarray(1, dtype=dtypestrace,  buffer=buffer)
            # self.newData.emit( self.arr[0][0] , self.arr[0][1] , self.arr[0][2] , self.arr[0][3]  , self.arr[0][4] , self.arr[0][5]  ) # <- Here you emit a signal!
            self.newData.emit(arr[0][0],arr[0][1],arr[0][2],arr[0][3],arr[0][4],arr[0][5],arr[0][6],arr[0][7],arr[0][8] )
            # self.newData.emit( self.arr[0] ) # <- Here you emit a signal!
            # print( self.arr[0][0] )
        self.stopdata()
            
thread = Thread()
thread.newData.connect(update)
thread.start()
thread.startdata( [ 'motor.state1.Id_SP', 'motor.state1.Iq_SP', 'motor.state1.Id_meas', 'motor.state1.Iq_meas', 'motor.state1.encoderPos1', 'motor.state1.encoderPos2','motor.state1.Vd','motor.state1.Vq','motor.state1.maxVolt'] )
# thread.startdata( [ 'motor.state1.Id_SP', 'motor.state1.Iq_SP', 'motor.state1.Id_meas', 'motor.state1.Iq_meas', 'motor.state1.thethaPark', 'motor.state1.encoderPos2','motor.state1.Vd','motor.state1.Vq','motor.state1.maxVolt'] )

#%%
thread.stopdata()


#%%
ser.write(b'b' + struct.pack('I',  int(0)))
bla = ser.readall()
setpar('motor.conf.Ndownsample' , int( 1 ))


#%% Working backup
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication, QGridLayout, QHBoxLayout, QWidget, QPushButton, QLineEdit, QSlider
from PyQt5.QtCore import QObject, pyqtSignal

app = QApplication([])
win = pg.GraphicsLayoutWidget()
layout = QGridLayout()

plot1 = pg.plot()
plot2 = pg.plot()
plot3 = pg.plot()

layout.addWidget(plot1, 0, 0)
layout.addWidget(plot2, 1, 0)
layout.addWidget(plot3, 2, 0)

curve1a = plot1.plot()
curve1b = plot1.plot(pen=(1,3))
curve2a = plot2.plot()
curve3a = plot3.plot()
curve3b = plot3.plot(pen=(1,3))

win.resize( 1000, 700)
win.setLayout(layout)
win.show()
# win.showFullScreen()

import threading
from collections import deque

y1 = deque()
y1a = deque()
y2 = deque()
y3 = deque()
y4 = deque()
T = deque()

signals = setTrace([ 'motor.state.curtime' , 'motor.state1.Ialpha', 'motor.state1.Ibeta', 'motor.state1.encoderPos1','motor.state.sensBus',
                   'motor.state1.Va', 'motor.state1.Vb', 'motor.state1.Vc'])
dtypestrace = [dtypes[j] for j in ser.signals]
buffer = bytearray(int(ser.tracebytes ))

# def addData_callbackFunc( value):
#     y2.extend( [value] ); 
#     while len(y2) > 5000:
#         y2.popleft() #remove oldest
#     return


# def addData_callbackFunc2( value):
#     y1.extend( [value] ); 
#     while len(y1) > 5000:
#         y1.popleft() #remove oldest
#     # curve1a.setData( y=y1)
#     return


# class Communicate(QObject):
#     data_signal = pyqtSignal(float)
#     data_signal2 = pyqtSignal(float)
# ''' End Class '''


# def dataSendLoop(addData_callbackFunc , addData_callbackFunc2):
#     signals = setTrace([ 'motor.state.curtime' , 'motor.state1.Ialpha', 'motor.state1.Ibeta', 'motor.state1.encoderPos1','motor.state.sensBus',
#                        'motor.state1.Va', 'motor.state1.Vb', 'motor.state1.Vc'])
#     dtypestrace = [dtypes[j] for j in ser.signals]
#     buffer = bytearray(int(ser.tracebytes ))
#     # setpar('motor.conf.Ndownsample' , int( 0.01/Ts ))
#     setpar('motor.conf.Ndownsample' , int( 0.001/Ts ))
#     ser.write(b'b' + struct.pack('I',  int(2**32-1)))

#     # Setup the signal-slot mechanism.
#     mySrc = Communicate()
#     mySrc.data_signal.connect(addData_callbackFunc)
#     mySrc.data_signal2.connect(addData_callbackFunc2)

#     while not win.isHidden():
#         while ser.in_waiting < len(buffer):
#             bla = 1
#         ser.readinto(buffer)
#         arr = np.ndarray(1, dtype=dtypestrace,  buffer=buffer)
#         mySrc.data_signal.emit( arr[0][3] ) # <- Here you emit a signal!
#         mySrc.data_signal2.emit( arr[0][4] ) # <- Here you emit a signal!
#     ser.write(b'b' + struct.pack('I',  int(0)))
#     bla = ser.readall()
#     setpar('motor.conf.Ndownsample' , int( 1 ))

# myDataLoop = threading.Thread(name = 'myDataLoop', target = dataSendLoop, daemon = True, args = (addData_callbackFunc,addData_callbackFunc2 ))
# myDataLoop.start()



def update(data):
    y2.extend( [data] ); 
    while len(y2) > 5000:
        y2.popleft() #remove oldest
    # plot1.plot(y2, clear=True)
    curve2a.setData( y=y2)
    return

class Thread(pg.QtCore.QThread):
    def startdata(self, signals):
        signals = setTrace(signals)
        dtypestrace = [dtypes[j] for j in ser.signals]
        buffer = bytearray(int(ser.tracebytes ))
        # setpar('motor.conf.Ndownsample' , int( 0.01/Ts ))
        setpar('motor.conf.Ndownsample' , int( 0.001/Ts ))
        ser.write(b'b' + struct.pack('I',  int(2**32-1)))
    
    def stopdata(self):
        ser.write(b'b' + struct.pack('I',  int(0)))
        bla = ser.readall()
        setpar('motor.conf.Ndownsample' , int( 1 ))
    
    newData = pg.QtCore.Signal(object)
    def run(self):
        while not win.isHidden():
            while ser.in_waiting < len(buffer):
                bla = 1
            ser.readinto(buffer)
            self.arr = np.ndarray(1, dtype=dtypestrace,  buffer=buffer)
            self.newData.emit( self.arr[0][3] ) # <- Here you emit a signal!
        self.stopdata()
            
            
thread = Thread()
thread.newData.connect(update)
thread.start()
thread.startdata( [ 'motor.state.curtime' , 'motor.state1.Ialpha', 'motor.state1.Ibeta', 'motor.state1.encoderPos1','motor.state.sensBus','motor.state1.Va', 'motor.state1.Vb', 'motor.state1.Vc'] )





#%%
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication, QGridLayout, QHBoxLayout, QWidget, QPushButton, QLineEdit, QSlider
from PyQt5.QtCore import QObject, pyqtSignal

app = QApplication([])
win = pg.GraphicsLayoutWidget()
layout = QGridLayout()

plot1 = pg.plot()
plot2 = pg.plot()
plot3 = pg.plot()

layout.addWidget(plot1, 0, 0)
layout.addWidget(plot2, 1, 0)
layout.addWidget(plot3, 2, 0)

curve1a = plot1.plot()
curve1b = plot1.plot(pen=(1,3))
curve2a = plot2.plot()
curve3a = plot3.plot()
curve3b = plot3.plot(pen=(1,3))

win.resize( 1000, 700)
win.setLayout(layout)
win.show()
# win.showFullScreen()

import threading
from collections import deque

y1 = deque()
y1a = deque()
y2 = deque()
y3 = deque()
y4 = deque()
T = deque()

signals = setTrace([ 'motor.state.curtime' , 'motor.state1.Ialpha', 'motor.state1.Ibeta', 'motor.state1.encoderPos1','motor.state.sensBus',
                   'motor.state1.Va', 'motor.state1.Vb', 'motor.state1.Vc'])
dtypestrace = [dtypes[j] for j in ser.signals]
buffer = bytearray(int(ser.tracebytes ))

def addData_callbackFunc( value):
    y2.extend( [value] ); 
    while len(y2) > 5000:
        y2.popleft() #remove oldest
    return


def addData_callbackFunc2( value):
    y1.extend( [value] ); 
    while len(y1) > 5000:
        y1.popleft() #remove oldest
    # curve1a.setData( y=y1)
    return


class Communicate(QObject):
    data_signal = pyqtSignal(float)
    data_signal2 = pyqtSignal(float)
''' End Class '''


def dataSendLoop(addData_callbackFunc , addData_callbackFunc2):
    signals = setTrace([ 'motor.state.curtime' , 'motor.state1.Ialpha', 'motor.state1.Ibeta', 'motor.state1.encoderPos1','motor.state.sensBus',
                       'motor.state1.Va', 'motor.state1.Vb', 'motor.state1.Vc'])
    dtypestrace = [dtypes[j] for j in ser.signals]
    buffer = bytearray(int(ser.tracebytes ))
    setpar('motor.conf.Ndownsample' , int( 0.01/Ts ))
    # setpar('motor.conf.Ndownsample' , int( 0.001/Ts ))
    ser.write(b'b' + struct.pack('I',  int(2**32-1)))

    # Setup the signal-slot mechanism.
    mySrc = Communicate()
    mySrc.data_signal.connect(addData_callbackFunc)
    mySrc.data_signal2.connect(addData_callbackFunc2)

    while not win.isHidden():
        while ser.in_waiting < len(buffer):
            bla = 1
        ser.readinto(buffer)
        arr = np.ndarray(1, dtype=dtypestrace,  buffer=buffer)
        mySrc.data_signal.emit( arr[0][3] ) # <- Here you emit a signal!
        mySrc.data_signal2.emit( arr[0][4] ) # <- Here you emit a signal!
    ser.write(b'b' + struct.pack('I',  int(0)))
    bla = ser.readall()
    setpar('motor.conf.Ndownsample' , int( 1 ))

myDataLoop = threading.Thread(name = 'myDataLoop', target = dataSendLoop, daemon = True, args = (addData_callbackFunc,addData_callbackFunc2 ))
myDataLoop.start()

for i in range(1000):
    curve2a.setData( y=y2)
    time.sleep(0.01)

    
# ser.write(b'b' + struct.pack('I',  int(0)))
# bla = ser.readall()
# setpar('motor.conf.Ndownsample' , int( 1 ))

