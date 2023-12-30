from pyqtgraph.parametertree import Parameter, ParameterTree
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets


# df = getallsignals()
df = readall()


params = list()
for i in np.argsort( signames):
    if sigtypes[i] == 'f':
        sertype = 'float'
    if sigtypes[i] == 'b':
        sertype = 'bool'
    if sigtypes[i] == 'i':
        sertype = 'int'
    if sigtypes[i] == 'I':
        sertype = 'int'
    if not (type(df[signames[i]][0]) == np.ndarray):
        params.append( {'name' : signames[i]  , 'type':  sertype , 'value': df[signames[i]][0] } )    
   
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


def update():
    df = readall()
    params = list()
    for i in range(len(signames)):
        if sigtypes[i] == 'f':
            sertype = 'float'
        if sigtypes[i] == 'b':
            sertype = 'bool'
        if sigtypes[i] == 'i':
            sertype = 'int'
        if sigtypes[i] == 'I':
            sertype = 'int'
        params.append( {'name' : signames[i]  , 'type':  sertype , 'value': df[signames[i]][0] } )    
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
                setpar( childName , data )
        apply_btn.setStyleSheet("background-color: grey")
        totalchanges = []
        changes_ready_to_transmit = 0
    return

_params.sigTreeStateChanged.connect(_enable_apply)

t = ParameterTree()
t.setParameters(_params, showTop=False)



win = pg.GraphicsLayoutWidget()
layout = QtWidgets.QGridLayout()

layout.addWidget(t, 0, 0)

apply_btn = QtWidgets.QPushButton('Apply Changes')
apply_btn.clicked.connect(apply_parameters)
apply_btn.setStyleSheet("background-color: grey")
layout.addWidget( apply_btn, 1, 0)

update_btn = QtWidgets.QPushButton('Update')
update_btn.clicked.connect( update )
update_btn.setStyleSheet("background-color: green")
layout.addWidget( update_btn, 2, 0)

win.resize( 500, 1000)
win.setLayout(layout)
win.show() 

od = _params.getValues()
d = {k : od[k][0] for k in od}




