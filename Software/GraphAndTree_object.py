#%%
# import pyqtgraph as pg
# from PyQt5.QtWidgets import QApplication, QGridLayout, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QLineEdit, QSlider, QCompleter
# from PyQt5.QtCore import QObject, pyqtSignal, QEvent
# from PyQt5.QtCore import Qt
# import struct

# from pyqtgraph.parametertree import Parameter, ParameterTree
# from pyqtgraph.Qt import QtCore, QtWidgets
# from pyqtgraph.widgets.PlotWidget import PlotWidget
# import numpy as np

# pg.setConfigOption('background', 'k')
# pg.setConfigOption('foreground', 'w')

# import TeensyMotorControl as tc

# m = tc.Motor(  )
# motor = tc.MotorVariables( m )

#%%
maxdata = 2001
Tsample = 0.01

class CustomViewBox(pg.ViewBox):
    def __init__(self, *args, **kwds):
        kwds['enableMenu'] = False
        pg.ViewBox.__init__(self, *args, **kwds)
        self.setMouseMode(self.RectMode)
        
    ## reimplement right-click to zoom out
    def mouseClickEvent(self, ev):
        if ev.button() == QtCore.Qt.MouseButton.RightButton:
            self.setXRange(0,maxdata*Tsample)
            self.enableAutoRange( pg.ViewBox.YAxis )

    ## reimplement right-click to zoom out
    def mouseDoubleClickEvent(self, ev):
        self.setXRange(0,maxdata*Tsample)
        self.enableAutoRange( pg.ViewBox.YAxis )
    
    ## reimplement mouseDragEvent to disable continuous axis zoom
    def mouseDragEvent(self, ev, axis=None):
        if axis is not None and ev.button() == QtCore.Qt.MouseButton.RightButton:
            ev.ignore()
        else:
            pg.ViewBox.mouseDragEvent(self, ev, axis=axis)

app = QApplication([])
win = pg.GraphicsLayoutWidget(title='Teensy Motor Control')
layout = QVBoxLayout()

vb1 = CustomViewBox()
plot1 = PlotWidget(viewBox=vb1, enableMenu=False)
vb2 = CustomViewBox()
plot2 = PlotWidget(viewBox=vb2, enableMenu=False)
vb3 = CustomViewBox()
plot3 = PlotWidget(viewBox=vb3, enableMenu=False)

plot1.hideButtons()
plot2.hideButtons()
plot3.hideButtons()

# plot1 = PlotWidget()
# plot2 = PlotWidget()
# plot3 = PlotWidget()

plot1.showGrid(x = True, y = True, alpha = 0.3)                                        
plot2.showGrid(x = True, y = True, alpha = 0.3)                                        
plot3.showGrid(x = True, y = True, alpha = 0.3)                                        

plot1.setXRange(0,maxdata*Tsample)
plot2.setXLink(plot1)
plot3.setXLink(plot1)
plot1.plotItem.getViewBox().setMouseMode(pg.ViewBox.RectMode)
plot2.plotItem.getViewBox().setMouseMode(pg.ViewBox.RectMode)
plot3.plotItem.getViewBox().setMouseMode(pg.ViewBox.RectMode)
# plot1.setXRange(0,maxdata)
# plot2.setXRange(0,maxdata)
# plot3.setXRange(0,maxdata)

layout.addWidget(plot1)
layout.addWidget(plot2)
layout.addWidget(plot3)

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


curve1a.setData ( x = np.linspace(0, (maxdata-1) * Tsample , maxdata) )
curve1b.setData ( x = np.linspace(0, (maxdata-1) * Tsample , maxdata) )
curve1c.setData ( x = np.linspace(0, (maxdata-1) * Tsample , maxdata) )
# curve1a.setData ( x = np.linspace(0, (maxdata-1) * Tsample , maxdata) )
# curve1a.setData ( x = np.linspace(0, (maxdata-1) * Tsample , maxdata) )


tracerunning = True

# win.showFullScreen()


import threading
from collections import deque

y1a = deque(maxlen=maxdata)
y1b = deque(maxlen=maxdata)
y1c = deque(maxlen=maxdata)
y1d = deque(maxlen=maxdata)
y2a = deque(maxlen=maxdata)
y2b = deque(maxlen=maxdata)
y3a = deque(maxlen=maxdata)
y3b = deque(maxlen=maxdata)
y3c = deque(maxlen=maxdata)

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
    curve1a.setData( x = np.linspace(0, (len(y1a)-1) * Tsample , len(y1a)) , y=y1a)
    curve1b.setData( x = np.linspace(0, (len(y1b)-1) * Tsample , len(y1b)) , y=y1b)
    curve1c.setData( x = np.linspace(0, (len(y1c)-1) * Tsample , len(y1c)) , y=y1c)
    curve1d.setData( x = np.linspace(0, (len(y1d)-1) * Tsample , len(y1d)) , y=y1d)
    curve2a.setData( x = np.linspace(0, (len(y2a)-1) * Tsample , len(y2a)) ,y=y2a)
    curve2b.setData( x = np.linspace(0, (len(y2b)-1) * Tsample , len(y2b)) ,y=y2b)
    curve3a.setData( x = np.linspace(0, (len(y3a)-1) * Tsample , len(y3a)) ,y=y3a)
    curve3b.setData( x = np.linspace(0, (len(y3b)-1) * Tsample , len(y3b)) ,y=y3b)
    curve3c.setData( x = np.linspace(0, (len(y3c)-1) * Tsample , len(y3c)) ,y=y3c)
    return

class Thread(pg.QtCore.QThread):
    def __init__(self):
      super(pg.QtCore.QThread, self).__init__()
      self.ready = False
  
    def startdata(self, signals):
        signals = m.setTrace(signals)
        self.dtypestrace = [m._dtypes[j] for j in m._tracesignals]
        self.buffer = bytearray(int(m._tracebytes ))
        # setpar('motor.conf.Ndownsample' , int( 1/Ts ))
        self.stopdata()
        m.setpar('motor.conf.Ndownsample' , int( Tsample/m.Ts ))
        m.ser.write(b'b' + struct.pack('I',  int(2**32-1)))
        self.ready = True
    
    def stopdata(self):
        m.ser.write(b'b' + struct.pack('I',  int(0)))
        m.ser.flushInput()
    
    def resume(self):
        m.ser.write(b'b' + struct.pack('I',  int(2**32-1)))  

    # newData = pg.QtCore.Signal(object)
    newData = pg.QtCore.Signal(float , float , float , float , float , float, float , float, float)
    def run(self):
        while not self.ready:
            pass
        while not win.isHidden():
            while m.ser.in_waiting < len(self.buffer) or not tracerunning:
                pass
            m.ser.readinto(self.buffer)
            arr = np.ndarray(1, dtype=self.dtypestrace,  buffer=self.buffer)
            # self.newData.emit( self.arr[0][0] , self.arr[0][1] , self.arr[0][2] , self.arr[0][3]  , self.arr[0][4] , self.arr[0][5]  ) # <- Here you emit a signal!
            self.newData.emit(arr[0][0],arr[0][1],arr[0][2],arr[0][3],arr[0][4],arr[0][5],arr[0][6],arr[0][7],arr[0][8] )
            # self.newData.emit( self.arr[0] ) # <- Here you emit a signal!
            # print( self.arr[0][0] )
        self.stopdata()
            



maxbytes = 8
df = m.getallsig( maxbytes )

# params = list()
# for i in np.argsort( signames):
#     if sigtypes[i] == 'f':
#         sertype = 'float'
#     if sigtypes[i] == 'b':
#         sertype = 'bool'
#     if sigtypes[i] == 'i':
#         sertype = 'int'
#     if sigtypes[i] == 'I':
#         sertype = 'int'
#     if not (type(df[signames[i]][0]) == np.ndarray):
#         params.append( {'name' : signames[i]  , 'type':  sertype , 'value': df[signames[i]][0] } )    
       
params = list()
for i in np.argsort( m.signames):
    if m._sigbytes[i] <= maxbytes: 
        m.signames[i].split('.')
        if m._sigtypes[i] == 'f':
            sertype = 'float'
        if m._sigtypes[i] == 'b':
            sertype = 'bool'
        if m._sigtypes[i] == 'i':
            sertype = 'int'
        if m._sigtypes[i] == 'I':
            sertype = 'int'
        if not (type(df[m.signames[i]][0]) == np.ndarray):
            params.append( {'name' : m.signames[i]  , 'type':  sertype , 'value': df[m.signames[i]][0] } )


# params = [
#     {'name': 'Save/Restore functionality', 'type': 'group', 'children': [
#         {'name': 'Save State', 'type': 'action'},
#         {'name': 'Save Sttwrate', 'type': 'action'},
#         {'name': 'Save Statwetewte', 'type': 'action'},
#     ]},
# ]




_params = Parameter.create(name='params', type='group',      children=params)
# _params = Parameter.create(name='params',     children=params)

changes_ready_to_transmit = 0
global totalchanges
totalchanges = []

def _enable_apply( param, changes):
    print("tree changes:")
    for param, change, data in changes:
        path = _params.childPath(param)
        if path is not None:
            childName = ".".join(path)
        else:
            childName = param.name()
        print("  parameter: %s" % childName)
        print("  change:    %s" % change)
        print("  data:      %s" % str(data))
        print("  ----------")
    global totalchanges
    totalchanges.append( changes )
    global changes_ready_to_transmit
    changes_ready_to_transmit = 1
    apply_btn.setStyleSheet("background-color: green")
    return


def update_tree():
    if tracerunning:
        thread.stopdata()
    df = m.getallsig( maxbytes )
    if tracerunning:
        thread.resume()
    params = list()
    for i in range(len(m.signames)):
        if m._sigbytes[i] <= maxbytes: 
            if m._sigtypes[i] == 'f':
                sertype = 'float'
            if m._sigtypes[i] == 'b':
                sertype = 'bool'
            if m._sigtypes[i] == 'i':
                sertype = 'int'
            if m._sigtypes[i] == 'I':
                sertype = 'int'
            params.append( {'name' : m.signames[i]  , 'type':  sertype , 'value': df[m.signames[i]][0] } )    
    # _params = Parameter.create(name='params', type='group',      children=params)
    # _params.setValue( params )
    _params.sigTreeStateChanged.disconnect()
    for param in _params:
        param.setValue( df[param.name()][0] )
    _params.sigTreeStateChanged.connect(_enable_apply)
    changes_ready_to_transmit = 0
    apply_btn.setStyleSheet("background-color: grey")
    totalchanges.clear()
    # t.setParameters(_params, showTop=False)

def apply_parameters():

    global changes_ready_to_transmit
    global totalchanges
    
    if changes_ready_to_transmit:
        print("Writing params'")
        for change in totalchanges:
            for param, change, data in change:
                path = _params.childPath(param)
                if path is not None:
                    childName = ".".join(path)
                else:
                    childName = param.name()
                print("  parameter: %s" % childName)
                print("  change:    %s" % change)
                print("  data:      %s" % str(data))
                print("  ----------")
                m.setpar( childName , data )
        apply_btn.setStyleSheet("background-color: grey")
        totalchanges = []
        changes_ready_to_transmit = 0
    return

def start_stop_trace():    
    global tracerunning
    if tracerunning:
        tracerunning = False
        thread.stopdata()
        trace_btn.setText('Start trace')
    else:
        tracerunning = True
        thread.resume()
        trace_btn.setText('Stop trace')
    return

def filter_tree( text ):
    text = text.lower()
    _params.sigTreeStateChanged.disconnect()
    t.hide()
    for param in _params:
        if( text in param.name().lower() ):
            if param.opts['visible'] == 0:
                param.show()
        else:
            if param.opts['visible'] == 1:
                param.hide()
    _params.sigTreeStateChanged.connect(_enable_apply)    
    t.show()


_params.sigTreeStateChanged.connect(_enable_apply)

t = ParameterTree()
t.setParameters(_params, showTop=False)


class MyLineEdit(QLineEdit):
    tabPressed = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        wordList = ["motor" , "motrrr" , "omicron" , "zeta"]
        self._compl = QCompleter( wordList )
        self.tabPressed.connect(self.next_completion)

    def next_completion(self):
        index = self._compl.currentIndex()
        self._compl.popup().setCurrentIndex(index)
        start = self._compl.currentRow()
        if not self._compl.setCurrentRow(start + 1):
            self._compl.setCurrentRow(0)

    def event(self, event):
        if event.type() == QEvent.KeyPress and event.key() == Qt.Key_Tab:
            self.tabPressed.emit()
            return True
        return super().event(event)


textbox = MyLineEdit()
textbox.textChanged.connect(filter_tree)

layout2 = QVBoxLayout()


layout2.addWidget(textbox)

layout2.addWidget(t)

layout3 = QHBoxLayout()
update_btn = QtWidgets.QPushButton('Update')
update_btn.clicked.connect( update_tree )
layout3.addWidget( update_btn)

apply_btn = QtWidgets.QPushButton('Apply Changes')
apply_btn.clicked.connect(apply_parameters)
apply_btn.setStyleSheet("background-color: grey")

trace_btn = QtWidgets.QPushButton('Stop trace')
trace_btn.clicked.connect( start_stop_trace )
# trace_btn.setStyleSheet("background-color: green")

layout3.addWidget( apply_btn)
layout3.addWidget( trace_btn)

layout2.addLayout( layout3)

layouttot = QHBoxLayout()

t.setFixedWidth(350)
textbox.setFixedWidth(350)
layouttot.addLayout( layout2)
layouttot.addLayout( layout)

win.resize( 1000, 700)
# win.show()
win.setLayout(layouttot)
win.show()

thread = Thread()
thread.newData.connect(update)
thread.start()




thread.startdata( [ 'motor.state1.Id_SP', 'motor.state1.Iq_SP', 'motor.state1.Id_meas', 'motor.state1.Iq_meas', 'motor.state1.encoderPos1', 'motor.state2.encoderPos1','motor.state1.Vd','motor.state1.Vq','motor.state1.maxVolt'] )
# thread.startdata( [ 'motor.state1.Iq_SP', 'motor.state2.Iq_SP', 'motor.state1.Iq_meas', 'motor.state2.Iq_meas', 'motor.state1.rmech', 'motor.state1.ymech','motor.state1.Vd','motor.state1.Vq','motor.state1.maxVolt'] )



#%%
text = 'kd'


#%%
from _buildParamTypes import makeAllParamTypes
from PyQt5.QtWidgets import QApplication, QGridLayout, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QLineEdit, QSlider

import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets

app = pg.mkQApp("Parameter Tree Example")
import pyqtgraph.parametertree.parameterTypes as pTypes
from pyqtgraph.parametertree import Parameter, ParameterTree

params = [
    {'name': 'Save/Restore functionality', 'type': 'group', 'children': [
        {'name': 'Save State', 'type': 'action'},
    ]},
]

## Create tree of Parameter objects
p = Parameter.create(name='params', type='group', children=params)

t = ParameterTree()
t.setParameters(p, showTop=False)
t.setWindowTitle('pyqtgraph example: Parameter Tree')


win = QtWidgets.QWidget()
layout = QtWidgets.QGridLayout()
win.setLayout(layout)
layout.addWidget(QtWidgets.QLabel("These are two views of the same data. They should always display the same values."), 0,  0, 1, 2)
layout.addWidget(t, 1, 0, 1, 1)
win.show()


#%%



# import pyqtgraph as pg
# from PyQt5.QtWidgets import QApplication, QGridLayout, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QLineEdit, QSlider
# from PyQt5.QtCore import QObject, pyqtSignal
# from pyqtgraph.parametertree import Parameter, ParameterTree
# from pyqtgraph.Qt import QtCore, QtWidgets
# from pyqtgraph.widgets.PlotWidget import PlotWidget

# pg.setConfigOption('background', 'w')
# pg.setConfigOption('foreground', 'k')

# app = QApplication([])
# win = pg.GraphicsLayoutWidget(title='Teensy Motor Control')
# layout = QVBoxLayout()

# plot1 = PlotWidget()
# plot2 = PlotWidget()
# plot3 = PlotWidget()

# plot1.showGrid(x = True, y = True, alpha = 0.3)                                        
# plot2.showGrid(x = True, y = True, alpha = 0.3)                                        
# plot3.showGrid(x = True, y = True, alpha = 0.3)                                        

# plot2.setXLink(plot1)
# plot3.setXLink(plot1)

# maxdata = 1000
# Tsample = 0.01

# plot1.plotItem.getViewBox().setMouseMode(pg.ViewBox.RectMode)
# plot2.plotItem.getViewBox().setMouseMode(pg.ViewBox.RectMode)
# plot3.plotItem.getViewBox().setMouseMode(pg.ViewBox.RectMode)
# plot1.setXRange(0,maxdata)
# plot2.setXRange(0,maxdata)
# plot3.setXRange(0,maxdata)

# layout.addWidget(plot1)
# layout.addWidget(plot2)
# layout.addWidget(plot3)

# col = [[0, 114.4320, 189.6960],
# [217.6000,  83.2000, 25.0880],
# [237.8240, 177.6640, 32.0000],
# [126.4640,  47.1040, 142.3360],
# [119.2960, 172.5440, 48.1280],
# [77.0560, 190.7200, 238.8480],
# [162.5600,  19.9680, 47.1040],
# [0, 114.4320, 189.6960]]

# width = 1
# curve1a = plot1.plot(pen=pg.mkPen(color=col[0], width=width))
# curve1b = plot1.plot(pen=pg.mkPen(color=col[1], width=width))
# curve1c = plot1.plot(pen=pg.mkPen(color=col[2], width=width))
# curve1d = plot1.plot(pen=pg.mkPen(color=col[3], width=width))
# curve2a = plot2.plot(pen=pg.mkPen(color=col[0], width=width))
# curve2b = plot2.plot(pen=pg.mkPen(color=col[1], width=width))
# curve3a = plot3.plot(pen=pg.mkPen(color=col[0], width=width))
# curve3b = plot3.plot(pen=pg.mkPen(color=col[1], width=width))
# curve3c = plot3.plot(pen=pg.mkPen(color=col[2], width=width))

# tracerunning = True

# # win.showFullScreen()

# import threading
# from collections import deque

# y1a = deque()
# y1b = deque()
# y1c = deque()
# y1d = deque()
# y2a = deque()
# y2b = deque()
# y3a = deque()
# y3b = deque()
# y3c = deque()

# def update(data1 , data2 , data3, data4 , data5 , data6 , data7 , data8 , data9 ):
#     y1a.extend( [data1] )
#     y1b.extend( [data2] )
#     y1c.extend( [data3] )
#     y1d.extend( [data4] )
#     y2a.extend( [data5] )
#     y2b.extend( [data6] )
#     y3a.extend( [data7] )
#     y3b.extend( [data8] )
#     y3c.extend( [data9] )
#     while len(y1a) > maxdata:
#         y1a.popleft() #remove oldest
#         y1b.popleft() #remove oldest
#         y1c.popleft() #remove oldest
#         y1d.popleft() #remove oldest
#         y2a.popleft() #remove oldest
#         y2b.popleft() #remove oldest
#         y3a.popleft() #remove oldest
#         y3b.popleft() #remove oldest
#         y3c.popleft() #remove oldest
#     curve1a.setData( y=y1a)
#     curve1b.setData( y=y1b)
#     curve1c.setData( y=y1c)
#     curve1d.setData( y=y1d)
#     curve2a.setData( y=y2a)
#     curve2b.setData( y=y2b)
#     curve3a.setData( y=y3a)
#     curve3b.setData( y=y3b)
#     curve3c.setData( y=y3c)
#     return

# class Thread(pg.QtCore.QThread):
#     def startdata(self, signals):
#         signals = setTrace(signals)
#         global dtypestrace, buffer
#         dtypestrace = [dtypes[j] for j in ser.signals]
#         buffer = bytearray(int(ser.tracebytes ))
#         # setpar('motor.conf.Ndownsample' , int( 1/Ts ))
#         self.stopdata()
#         setpar('motor.conf.Ndownsample' , int( Tsample/Ts ))
#         ser.write(b'b' + struct.pack('I',  int(2**32-1)))
    
#     def stopdata(self):
#         ser.write(b'b' + struct.pack('I',  int(0)))
#         ser.flushInput()
    
#     def resume(self):
#         ser.write(b'b' + struct.pack('I',  int(2**32-1)))  

#     # newData = pg.QtCore.Signal(object)
#     newData = pg.QtCore.Signal(float , float , float , float , float , float, float , float, float)
#     def run(self):
#         while not win.isHidden():
#             while ser.in_waiting < len(buffer):
#                 bla = 1
#             ser.readinto(buffer)
#             arr = np.ndarray(1, dtype=dtypestrace,  buffer=buffer)
#             # self.newData.emit( self.arr[0][0] , self.arr[0][1] , self.arr[0][2] , self.arr[0][3]  , self.arr[0][4] , self.arr[0][5]  ) # <- Here you emit a signal!
#             self.newData.emit(arr[0][0],arr[0][1],arr[0][2],arr[0][3],arr[0][4],arr[0][5],arr[0][6],arr[0][7],arr[0][8] )
#             # self.newData.emit( self.arr[0] ) # <- Here you emit a signal!
#             # print( self.arr[0][0] )
#         self.stopdata()
            




# df = readall()

# # params = list()
# # for i in np.argsort( signames):
# #     if sigtypes[i] == 'f':
# #         sertype = 'float'
# #     if sigtypes[i] == 'b':
# #         sertype = 'bool'
# #     if sigtypes[i] == 'i':
# #         sertype = 'int'
# #     if sigtypes[i] == 'I':
# #         sertype = 'int'
# #     if not (type(df[signames[i]][0]) == np.ndarray):
# #         params.append( {'name' : signames[i]  , 'type':  sertype , 'value': df[signames[i]][0] } )    
       
# params = list()
# for i in np.argsort( signames):
#     signames[i].split('.')
#     if sigtypes[i] == 'f':
#         sertype = 'float'
#     if sigtypes[i] == 'b':
#         sertype = 'bool'
#     if sigtypes[i] == 'i':
#         sertype = 'int'
#     if sigtypes[i] == 'I':
#         sertype = 'int'
#     if not (type(df[signames[i]][0]) == np.ndarray):
#         params.append( {'name' : signames[i]  , 'type':  sertype , 'value': df[signames[i]][0] } )


# # params = [
# #     {'name': 'Save/Restore functionality', 'type': 'group', 'children': [
# #         {'name': 'Save State', 'type': 'action'},
# #         {'name': 'Save Sttwrate', 'type': 'action'},
# #         {'name': 'Save Statwetewte', 'type': 'action'},
# #     ]},
# # ]




# _params = Parameter.create(name='params', type='group',      children=params)
# # _params = Parameter.create(name='params',     children=params)

# changes_ready_to_transmit = 0
# global totalchanges
# totalchanges = []

# def _enable_apply( param, changes):
#     print("tree changes:")
#     for param, change, data in changes:
#         path = _params.childPath(param)
#         if path is not None:
#             childName = ".".join(path)
#         else:
#             childName = param.name()
#         print("  parameter: %s" % childName)
#         print("  change:    %s" % change)
#         print("  data:      %s" % str(data))
#         print("  ----------")
#     global totalchanges
#     totalchanges.append( changes )
#     global changes_ready_to_transmit
#     changes_ready_to_transmit = 1
#     apply_btn.setStyleSheet("background-color: green")
#     return


# def update_tree():
#     if tracerunning:
#         thread.stopdata()
#     df = readall()
#     if tracerunning:
#         thread.resume()
#     params = list()
#     for i in range(len(signames)):
#         if sigtypes[i] == 'f':
#             sertype = 'float'
#         if sigtypes[i] == 'b':
#             sertype = 'bool'
#         if sigtypes[i] == 'i':
#             sertype = 'int'
#         if sigtypes[i] == 'I':
#             sertype = 'int'
#         params.append( {'name' : signames[i]  , 'type':  sertype , 'value': df[signames[i]][0] } )    
#     # _params = Parameter.create(name='params', type='group',      children=params)
#     # _params.setValue( params )
#     _params.sigTreeStateChanged.disconnect()
#     for param in _params:
#         param.setValue( df[param.name()][0] )
#     _params.sigTreeStateChanged.connect(_enable_apply)
#     changes_ready_to_transmit = 0
#     apply_btn.setStyleSheet("background-color: grey")
#     totalchanges.clear()
#     # t.setParameters(_params, showTop=False)

# def apply_parameters():

#     global changes_ready_to_transmit
#     global totalchanges
    
#     if changes_ready_to_transmit:
#         print("Writing params'")
#         for change in totalchanges:
#             for param, change, data in change:
#                 path = _params.childPath(param)
#                 if path is not None:
#                     childName = ".".join(path)
#                 else:
#                     childName = param.name()
#                 print("  parameter: %s" % childName)
#                 print("  change:    %s" % change)
#                 print("  data:      %s" % str(data))
#                 print("  ----------")
#                 setpar( childName , data )
#         apply_btn.setStyleSheet("background-color: grey")
#         totalchanges = []
#         changes_ready_to_transmit = 0
#     return

# def start_stop_trace():    
#     global tracerunning
#     if tracerunning:
#         tracerunning = False
#         thread.stopdata()
#         trace_btn.setText('Start trace')
#     else:
#         tracerunning = True
#         thread.resume()
#         trace_btn.setText('Stop trace')
#     return


# _params.sigTreeStateChanged.connect(_enable_apply)

# t = ParameterTree()
# t.setParameters(_params, showTop=False)



# layout2 = QVBoxLayout()
# layout2.addWidget(t)

# layout3 = QHBoxLayout()
# update_btn = QtWidgets.QPushButton('Update')
# update_btn.clicked.connect( update_tree )
# layout3.addWidget( update_btn)

# apply_btn = QtWidgets.QPushButton('Apply Changes')
# apply_btn.clicked.connect(apply_parameters)
# apply_btn.setStyleSheet("background-color: grey")

# trace_btn = QtWidgets.QPushButton('Stop trace')
# trace_btn.clicked.connect( start_stop_trace )
# # trace_btn.setStyleSheet("background-color: green")

# layout3.addWidget( apply_btn)
# layout3.addWidget( trace_btn)

# layout2.addLayout( layout3)

# layouttot = QHBoxLayout()

# t.setFixedWidth(350)
# layouttot.addLayout( layout2)
# layouttot.addLayout( layout)

# win.resize( 1000, 700)
# # win.show()
# win.setLayout(layouttot)
# win.show()

# thread = Thread()
# thread.newData.connect(update)
# thread.start()




# # thread.startdata( [ 'motor.state1.Id_SP', 'motor.state1.Iq_SP', 'motor.state1.Id_meas', 'motor.state1.Iq_meas', 'motor.state1.encoderPos1', 'motor.state1.encoderPos2','motor.state1.Vd','motor.state1.Vq','motor.state1.maxVolt'] )
# thread.startdata( [ 'motor.state1.Id_SP', 'motor.state1.Iq_SP', 'motor.state1.Id_meas', 'motor.state1.Iq_meas', 'motor.state1.thetaPark_enc', 'motor.state1.thetaPark_obs','motor.state1.Vd','motor.state1.Vq','motor.state1.maxVolt'] )
