
import pyqtgraph as pg
from collections import deque
from pyqtgraph.Qt import QtGui, QtCore, QtWidgets

# tic = time.time()
df = readall()
# toc = time.time()
# print(toc-tic, 'sec Elapsed')
# tic = time.time()
# df2 = getallsignals()
# toc = time.time()
# print(toc-tic, 'sec Elapsed')

app = QApplication([])

win2 = pg.GraphicsLayoutWidget()

layout2 = QGridLayout()

increasebutton = QPushButton("Restart controller")

setcont = QPushButton("Set controller 1")
setcont2 = QPushButton("Set controller 2")
Kp = QLineEdit(str( df['motor.conf1.Kp'][0] ))
fBW = QLineEdit(str( df['motor.conf1.fBW'][0] ))
alpha1 = QLineEdit(str( df['motor.conf1.alpha1'][0] ))
alpha2 = QLineEdit(str( df['motor.conf1.alpha2'][0] ))
fInt = QLineEdit(str( df['motor.conf1.fInt'][0] ))
fLP = QLineEdit(str( df['motor.conf1.fLP'][0] ))
Kp2 = QLineEdit(str( df['motor.conf2.Kp'][0] ))
fBW2 = QLineEdit(str( df['motor.conf2.fBW'][0] ))
alpha1_2 = QLineEdit(str( df['motor.conf2.alpha1'][0] ))
alpha2_2 = QLineEdit(str( df['motor.conf2.alpha2'][0] ))
fInt2 = QLineEdit(str( df['motor.conf2.fInt'][0] ))
fLP2 = QLineEdit(str( df['motor.conf2.fLP'][0] ))


layout2.addWidget( setcont ,1,0)
layout2.addWidget( setcont2 ,1,1)
layout2.addWidget(Kp, 2, 0)
layout2.addWidget(fBW, 3, 0)
layout2.addWidget( alpha1 , 4,0)
layout2.addWidget( alpha2 , 5,0)
layout2.addWidget( fInt , 6,0)
layout2.addWidget( fLP , 7,0)

layout2.addWidget(Kp2, 2, 1)
layout2.addWidget(fBW2, 3, 1)
layout2.addWidget( alpha1_2 , 4,1)
layout2.addWidget( alpha2_2 , 5,1)
layout2.addWidget( fInt2 , 6,1)
layout2.addWidget( fLP2 , 7,1)

def setcont_clicked():
    setpar( 'Kp' , float(Kp.text()))
    setpar( 'fBW' , float(fBW.text()))
    setpar( 'alpha1' , float(alpha1.text()))
    setpar( 'alpha2' , float(alpha2.text()))
    setpar( 'fInt' , float(fInt.text()))
    setpar( 'fLP' , float(fLP.text()))
    ser.write( b'C' + struct.pack('I',  0) ) 
    ser.write( b'o' + struct.pack('f',  0) ) # Restart controller


def setcont2_clicked():
    setpar( 'Kp2' , float(Kp2.text()))
    setpar( 'fBW2' , float(fBW2.text()))
    setpar( 'alpha1_2' , float(alpha1_2.text()))
    setpar( 'alpha2_2' , float(alpha2_2.text()))
    setpar( 'fInt2' , float(fInt2.text()))
    setpar( 'fLP2' , float(fLP2.text()))
    ser.write( b'C' + struct.pack('I',  0) ) 
    ser.write( b'o' + struct.pack('f',  0) ) # Restart controller


setcont.clicked.connect(setcont_clicked)
setcont2.clicked.connect(setcont2_clicked)

win2.resize( 300, 1)
win2.setLayout(layout2)
win2.show()

#%%
import pyqtgraph as pg
from collections import deque
from pyqtgraph.Qt import QtGui, QtCore, QtWidgets
from PyQt5.QtWidgets import QApplication, QGridLayout, QHBoxLayout, QWidget, QPushButton, QLineEdit, QSlider
# from PyQt5.QtCore import *
# from PyQt5.QtGui import *


p = 2*2*pi
v = 30*2*pi
a = 1000*2*pi
j = 1000000*2*pi
[t1, t2, t3, jd] = prepSP(  p , v , a , j )

# signals = [ 'tA', 'tB', 'tC']
# setTrace( signals )

y1 = deque()
y1a = deque()
y2 = deque()
y3 = deque()
y4 = deque()
T = deque()

app = QApplication([])
win = pg.GraphicsLayoutWidget()
layout = QGridLayout()


increasebutton = QPushButton("Restart controller")
setcont = QPushButton("Set controller")
decreasebutton = QPushButton("Start Setpoint")
decreasebutton2 = QPushButton("Start Setpoint2")
setspbutton = QPushButton("Set Setpoint")
setpointdist1 = QLineEdit(str(p/2/pi))
setpointdist2 = QLineEdit(str(v/2/pi))
setpointdist3 = QLineEdit(str(a/2/pi))
setpointdist4 = QLineEdit(str(j/2/pi))
nsetpoint = QSlider(1)
nsetpoint.setRange(1, 100)
haptic = QPushButton("Switch haptic mode")
cont_setting = QSlider(1)
cont_setting.setRange(1, 4)
cont_setting.setPageStep(1)

layout.addWidget(increasebutton, 0,0)
layout.addWidget( cont_setting ,1,0)
layout.addWidget(setcont , 2,0)
# layout.addWidget( nsetpoint, 3,0)
# layout.addWidget(decreasebutton, 4, 0)
# layout.addWidget(decreasebutton2, 5, 0)
# layout.addWidget(setspbutton, 6, 0)
# layout.addWidget( setpointdist1 , 7,0)
# layout.addWidget( setpointdist2 , 8,0)
# layout.addWidget( setpointdist3 , 9,0)
# layout.addWidget( setpointdist4 , 10,0)
# layout.addWidget( haptic , 11,0)

hapticon = 0

def on_increasebutton_clicked():
    ser.write( b'o' + struct.pack('f',  0) ) # Restart controller

def on_decreasebutton_clicked():
    setpar('SPdir' , 1)
    setpar('spNgo' , int( nsetpoint.value() ))

def on_decreasebutton_clicked2():
    setpar('SPdir' , 0)
    setpar('spNgo' , int( nsetpoint.value() ))
    
def setcont_clicked():
    ser.write( b'C' + struct.pack('I',  int( cont_setting.value() ) )    )  
    
def on_setsp_clicked():
    p = float(setpointdist1.text())*2*pi
    v = float(setpointdist2.text())*2*pi
    a = float(setpointdist3.text())*2*pi
    j = float(setpointdist4.text())*2*pi
    [t1, t2, t3, jd] = prepSP(  p , v , a , j )

def haptic_clicked():
    global hapticon
    if hapticon ==0:
        hapticon = 1
    else:        
        hapticon = 0
    setpar('haptic' , hapticon)
    ser.write( b'o' + struct.pack('f',  0) ) # Restart controller

increasebutton.clicked.connect(on_increasebutton_clicked)
decreasebutton.clicked.connect(on_decreasebutton_clicked)
decreasebutton2.clicked.connect(on_decreasebutton_clicked2)
setspbutton.clicked.connect(on_setsp_clicked)
setcont.clicked.connect(setcont_clicked)
haptic.clicked.connect(haptic_clicked)

plot1 = pg.plot()
plot2 = pg.plot()
plot3 = pg.plot()

layout.addWidget(plot1, 0, 1)
layout.addWidget(plot2, 1, 1)
layout.addWidget(plot3, 2, 1)

curve1 = plot1.plot()
curve1a = plot1.plot(pen=(1,3))
curve2 = plot2.plot()
curve3 = plot3.plot()
curve4 = plot3.plot(pen=(1,3))

win.resize( 1000, 700)
win.setLayout(layout)
win.show()

#%%


signals = setTrace([ 'motor.state.curtime' , 'motor.state1.Ialpha', 'motor.state1.Ibeta', 'motor.state1.encoderPos1','motor.state.sensBus',
                   'motor.state1.Va', 'motor.state1.Vb', 'motor.state1.Vc'])
dtypestrace = [dtypes[j] for j in ser.signals]

# setpar('motor.conf.Ndownsample' , int( 0.002/Ts ))
# setpar('motor.conf.Ndownsample' , int( 0.02/Ts ))
# ser.write(b'b' + struct.pack('I',  int(1e6)))
# ser.write(b'b' + struct.pack('I',  int(1)))



buffer = bytearray(int(ser.tracebytes ))



import threading


def addData_callbackFunc( value):
    # print("Add data: " + str(value))
    # myFig.addData(value)
    y2.extend( [value] ); 
    # T.extend( [] )
    while len(y2) > 5000:
        y2.popleft() #remove oldest
        # T.popleft()

    curve2.setData( y=y2)
    return


def addData_callbackFunc2( value):
    # print("Add data: " + str(value))
    # myFig.addData(value)
    y1.extend( [value] ); 
    # T.extend( [] )
    while len(y1) > 5000:
        y1.popleft() #remove oldest
        # T.popleft()

    curve1.setData( y=y1)
    return




class Communicate(QObject):
    data_signal = pyqtSignal(float)
    data_signal2 = pyqtSignal(float)
''' End Class '''


def dataSendLoop(addData_callbackFunc , addData_callbackFunc2):
    # Setup the signal-slot mechanism.
    mySrc = Communicate()
    mySrc.data_signal.connect(addData_callbackFunc)
    mySrc.data_signal2.connect(addData_callbackFunc2)

    # Simulate some data
    n = np.linspace(0, 499, 500)
    y = 50 + 25*(np.sin(n / 8.3)) + 10*(np.sin(n / 7.5)) - 5*(np.sin(n / 1.5))
    # y = y/1e6
    i = 0

    while(True):
        # if(i > 499):
        #     i = 0
        # time.sleep(1/20)
        while ser.in_waiting < len(buffer):
            bla = 1
        ser.readinto(buffer)
        arr = np.ndarray(1, dtype=dtypestrace,  buffer=buffer)
        mySrc.data_signal.emit( arr[0][3] ) # <- Here you emit a signal!
        mySrc.data_signal2.emit( arr[0][4] ) # <- Here you emit a signal!
        i += 1



signals = setTrace([ 'motor.state.curtime' , 'motor.state1.Ialpha', 'motor.state1.Ibeta', 'motor.state1.encoderPos1','motor.state.sensBus',
                   'motor.state1.Va', 'motor.state1.Vb', 'motor.state1.Vc'])
dtypestrace = [dtypes[j] for j in ser.signals]

buffer = bytearray(int(ser.tracebytes ))

setpar('motor.conf.Ndownsample' , int( 0.01/Ts ))
# setpar('motor.conf.Ndownsample' , int( 1/Ts ))
ser.write(b'b' + struct.pack('I',  int(2**32-1)))

myDataLoop = threading.Thread(name = 'myDataLoop', target = dataSendLoop, daemon = True, args = (addData_callbackFunc,addData_callbackFunc2 ))
myDataLoop.start()



    
# ser.write(b'b' + struct.pack('I',  int(0)))
# bla = ser.readall()
# setpar('motor.conf.Ndownsample' , int( 1 ))

    
#%%




while not win.isHidden():
    while ser.in_waiting < len(buffer):
        app.processEvents() 
        # print( ser.in_waiting ) 
    ser.readinto(buffer)
        
    arr = np.ndarray(1, dtype=dtypestrace,  buffer=buffer)

    T.extend( [arr['motor.state.curtime'][0] / 1e6 ])
    y1.extend( [arr[0][1]]  ); 
    y1a.extend( [arr[0][2]]) ; 
    y2.extend(  [arr[0][3]]); 
    y3.extend(  [arr[0][4]]) ; 
    y4.extend(  [arr[0][5]]) ; 
    
    while len(y1) > 500:
        y1.popleft() #remove oldest
        y1a.popleft() #remove oldest
        y2.popleft() #remove oldest
        y3.popleft()
        y4.popleft()
        T.popleft()
    curve1.setData( x=T , y=y1)
    curve1a.setData( x=T , y=y1a)
    curve2.setData( x=T , y=y2)
    curve3.setData( x=T , y=y3)
    curve4.setData( x=T , y=y4)
    app.processEvents()  


ser.write(b'b' + struct.pack('I',  int(0)))
bla = ser.readall()
setpar('motor.conf.Ndownsample' , int( 1 ))





#%%
###################################################################
#                                                                 #
#                    PLOT A LIVE GRAPH (PyQt5)                    #
#                  -----------------------------                  #
#            EMBED A MATPLOTLIB ANIMATION INSIDE YOUR             #
#            OWN GUI!                                             #
#                                                                 #
###################################################################

import sys
import os
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import functools
import numpy as np
import random as rd
import matplotlib
matplotlib.use("Qt5Agg")
from matplotlib.figure import Figure
from matplotlib.animation import TimedAnimation
from matplotlib.lines import Line2D
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import time
import threading

signals = setTrace([ 'motor.state.curtime' , 'motor.state1.Ialpha', 'motor.state1.Ibeta', 'motor.state1.encoderPos1','motor.state.sensBus',
                   'motor.state1.Va', 'motor.state1.Vb', 'motor.state1.Vc'])
dtypestrace = [dtypes[j] for j in ser.signals]

buffer = bytearray(int(ser.tracebytes ))

setpar('motor.conf.Ndownsample' , int( 0.01/Ts ))
ser.write(b'b' + struct.pack('I',  int(2**32-1)))

class CustomMainWindow(QMainWindow):
    def __init__(self):
        super(CustomMainWindow, self).__init__()
        # Define the geometry of the main window
        self.setGeometry(300, 300, 800, 400)
        self.setWindowTitle("my first window")
        # Create FRAME_A
        self.FRAME_A = QFrame(self)
        self.FRAME_A.setStyleSheet("QWidget { background-color: %s }" % QColor(210,210,235,255).name())
        self.LAYOUT_A = QGridLayout()
        self.FRAME_A.setLayout(self.LAYOUT_A)
        self.setCentralWidget(self.FRAME_A)
        # Place the zoom button
        self.zoomBtn = QPushButton(text = 'zoom')
        self.zoomBtn.setFixedSize(100, 50)
        self.zoomBtn.clicked.connect(self.zoomBtnAction)
        self.LAYOUT_A.addWidget(self.zoomBtn, *(0,0))
        # Place the matplotlib figure
        self.myFig = CustomFigCanvas()
        self.LAYOUT_A.addWidget(self.myFig, *(0,1))
        self.myFig2 = CustomFigCanvas()
        self.LAYOUT_A.addWidget(self.myFig2, *(1,1))
        # Add the callbackfunc to ..
        self.myDataLoop = threading.Thread(name = 'myDataLoop', target = dataSendLoop, daemon = True, args = (self.addData_callbackFunc,self.addData_callbackFunc2 ))
        self.myDataLoop.start()
        self.show()
        return

    def zoomBtnAction(self):
        print("zoom in")
        # self.myFig.zoomIn(5)
        self.myFig.close()
        self.myFig2.close()
        self.close()
        return

    def addData_callbackFunc(self, value):
        # print("Add data: " + str(value))
        self.myFig.addData(value)
        return

    def addData_callbackFunc2(self, value):
        # print("Add data: " + str(value))
        self.myFig2.addData(value)
        return

''' End Class '''


class CustomFigCanvas( FigureCanvas, TimedAnimation):
    def __init__(self):
        self.addedData = []
        print(matplotlib.__version__)
        # The data
        self.xlim = 1000
        self.n = np.linspace(0, self.xlim - 1, self.xlim)
        a = []
        b = []
        a.append(2.0)
        a.append(4.0)
        a.append(2.0)
        b.append(4.0)
        b.append(3.0)
        b.append(4.0)
        self.y = (self.n * 0.0) + 50
        # The window
        self.fig = Figure(figsize=(5,5), dpi=100)
        self.ax1 = self.fig.add_subplot(111)
        # self.ax1 settings
        self.ax1.set_xlabel('time')
        self.ax1.set_ylabel('raw data')
        self.line1 = Line2D([], [], color='blue' )
        self.line1_head = Line2D([], [], color='red', marker='o', markeredgecolor='r')
        self.ax1.add_line(self.line1)
        self.ax1.add_line(self.line1_head)
        self.ax1.set_xlim(0, self.xlim - 1)
        FigureCanvas.__init__(self, self.fig)
        
        # TimedAnimation.__init__(self, self.fig, interval = 50, blit = True)
        TimedAnimation.__init__(self, self.fig, interval = 7, blit = True)
        return

    def new_frame_seq(self):
        return iter(range(self.n.size))

    def _init_draw(self):
        lines = [self.line1, self.line1_head]
        for l in lines:
            l.set_data([], [])
        return

    def addData(self, value):
        self.addedData.append(value)
        return

    def zoomIn(self, value):
        bottom = self.ax1.get_ylim()[0]
        top = self.ax1.get_ylim()[1]
        bottom += value
        top -= value
        self.ax1.set_ylim(bottom,top)
        self.draw()
        return

    def close(self):
        self.FigureCanvas.close()
        return

    def _step(self, *args):
        # Extends the _step() method for the TimedAnimation class.
        try:
            TimedAnimation._step(self, *args)
        except Exception as e:
            self.abc += 1
            print(str(self.abc))
            TimedAnimation._stop(self)
            pass
        return

    def _draw_frame(self, framedata):
        margin = 2
        while(len(self.addedData) > 0):
            self.y = np.roll(self.y, -1)
            self.y[-1] = self.addedData[0]
            del(self.addedData[0])

        self.line1.set_data(self.n[ 0 : self.n.size - margin ], self.y[ 0 : self.n.size - margin ])
        self.line1_head.set_data(self.n[-1 - margin], self.y[-1 - margin])
        
        self.miny = min(self.y)
        self.maxy = max(self.y)
        if self.miny == self.maxy:
            self.ax1.set_ylim( self.miny -0.1 , self.maxy +0.1)
            self.draw()
        elif( self.ax1.get_ylim()[0] != self.miny or self.ax1.get_ylim()[1] != self.maxy ):
            self.ax1.set_ylim( self.miny , self.maxy )
            # self.ax1.relim(visible_only=True)
            self.draw()
        self._drawn_artists = [self.line1,  self.line1_head]

        return

''' End Class '''


# You need to setup a signal slot mechanism, to
# send data to your GUI in a thread-safe way.
# Believe me, if you don't do this right, things
# go very very wrong..
class Communicate(QObject):
    data_signal = pyqtSignal(float)
    data_signal2 = pyqtSignal(float)

''' End Class '''



def dataSendLoop(addData_callbackFunc , addData_callbackFunc2):
    # Setup the signal-slot mechanism.
    mySrc = Communicate()
    mySrc.data_signal.connect(addData_callbackFunc)
    mySrc.data_signal2.connect(addData_callbackFunc2)

    # Simulate some data
    n = np.linspace(0, 499, 500)
    y = 50 + 25*(np.sin(n / 8.3)) + 10*(np.sin(n / 7.5)) - 5*(np.sin(n / 1.5))
    # y = y/1e6
    i = 0

    while(True):
        # if(i > 499):
        #     i = 0
        # time.sleep(1/20)
        while ser.in_waiting < len(buffer):
            bla = 1
        ser.readinto(buffer)
        arr = np.ndarray(1, dtype=dtypestrace,  buffer=buffer)
        mySrc.data_signal.emit( arr[0][3] ) # <- Here you emit a signal!
        mySrc.data_signal2.emit( arr[0][4] ) # <- Here you emit a signal!
        i += 1

    ###
###

if __name__== '__main__':
    app = QApplication(sys.argv)
    QApplication.setStyle(QStyleFactory.create('Plastique'))
    myGUI = CustomMainWindow()
    sys.exit(app.exec_())