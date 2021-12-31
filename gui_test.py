from PyQt5 import QtCore, QtWidgets, QtSerialPort
from PyQt5.QtCore import Qt
import sys
import random
import numpy as np
import pyqtgraph as pg
from serialCAN import serialCAN


class Ui_MainWindow(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super(Ui_MainWindow, self).__init__(*args, **kwargs)
        self.setAutoFillBackground(True)
        p = self.palette()
        p.setColor(self.backgroundRole(), Qt.black)
        p.setColor(self.foregroundRole(), Qt.white)
        self.setPalette(p)
        
    def setupUi(self):
        self.resize(1600, 1000)
        self.centralwidget = QtWidgets.QWidget(self)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.gridLayout_2.addLayout(self.verticalLayout, 1, 0, 1, 1)
        self.groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox.setObjectName("groupBox")
        self.groupBox.setStyleSheet(
            'QGroupBox:title {color: rgb(1, 130, 153);}')
        self.gridLayout_3 = QtWidgets.QGridLayout(self.groupBox)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.lineEdit = QtWidgets.QLineEdit(self.groupBox)
        self.lineEdit.setObjectName("lineEdit")
        self.gridLayout.addWidget(self.lineEdit, 2, 1, 1, 1)
        self.lineEdit_2 = QtWidgets.QLineEdit(self.groupBox)
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.gridLayout.addWidget(self.lineEdit_2, 2, 3, 1, 1)
    
        self.serialSelector = QtWidgets.QComboBox(self.groupBox)
        self.serialSelector.setModelColumn(0)
        self.serialSelector.setObjectName("serialSelector")
        for info in QtSerialPort.QSerialPortInfo.availablePorts():
            self.serialSelector.addItem(info.portName())
        self.gridLayout.addWidget(self.serialSelector, 0, 1, 1, 1)

        self.serialConnectBtn = QtWidgets.QPushButton(self.groupBox)
        self.serialConnectBtn.setObjectName("serialConnectBtn")
        self.serialConnectBtn.setCheckable(True)
        self.gridLayout.addWidget(self.serialConnectBtn, 0, 2, 1, 1)

        # self.enterMoterBtn = QtWidgets.QPushButton(self.groupBox)
        # self.enterMoterBtn.setObjectName("enterMotorBtn")
        # self.enterMoterBtn.setCheckable(True)
        # self.gridLayout.addWidget(self.enterMoterBtn, 0, 3, 1, 1)

        self.label_can = QtWidgets.QLabel(self.groupBox)
        self.label_can.setObjectName("label_can")
        self.label_can.setStyleSheet(
            'QLabel {color: white;}')
        self.gridLayout.addWidget(self.label_can, 0, 4, 1, 1)

        self.lineEdit_can = QtWidgets.QLineEdit(self.groupBox)
        self.lineEdit_can.setObjectName("lineEdit_can")
        self.gridLayout.addWidget(self.lineEdit_can, 0, 5, 1, 1)

        self.positionSlider = QtWidgets.QSlider(self.groupBox)
        self.positionSlider.setMaximum(6290)
        self.positionSlider.setPageStep(10)
        self.positionSlider.setOrientation(QtCore.Qt.Horizontal)
        self.positionSlider.setObjectName("positionSlider")
        self.gridLayout.addWidget(self.positionSlider, 1, 1, 1, 5)
        self.label_8 = QtWidgets.QLabel(self.groupBox)
        self.label_8.setObjectName("label_8")
        self.label_8.setStyleSheet(
            'QLabel {color: white;}')
        self.gridLayout.addWidget(self.label_8, 2, 4, 1, 1)
        self.lineEdit_3 = QtWidgets.QLineEdit(self.groupBox)
        self.lineEdit_3.setObjectName("lineEdit_3")
        self.gridLayout.addWidget(self.lineEdit_3, 2, 5, 1, 1)
        self.label_position = QtWidgets.QLabel(self.groupBox)
        self.label_position.setObjectName("label_position")
        self.label_position.setStyleSheet(
            'QLabel {color: white;}')
        self.gridLayout.addWidget(self.label_position, 1, 0, 1, 1)
        self.label_7 = QtWidgets.QLabel(self.groupBox)
        self.label_7.setObjectName("label_7")
        self.label_7.setStyleSheet(
            'QLabel {color: white;}')
        self.gridLayout.addWidget(self.label_7, 2, 2, 1, 1)
        self.label_kpkd = QtWidgets.QLabel(self.groupBox)
        self.label_kpkd.setObjectName("label_kpkd")
        self.label_kpkd.setStyleSheet(
            'QLabel {color: white;}')
        self.gridLayout.addWidget(self.label_kpkd, 2, 0, 1, 1)
        self.label_serial = QtWidgets.QLabel(self.groupBox)
        self.label_serial.setObjectName("label_serial")
        self.label_serial.setStyleSheet(
            'QLabel {color: white;}')
        self.gridLayout.addWidget(self.label_serial, 0, 0, 1, 1)
        self.gridLayout_3.addLayout(self.gridLayout, 0, 0, 1, 1)
        self.gridLayout_2.addWidget(self.groupBox, 0, 0, 1, 1)
        self.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(self)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 24))
        self.menubar.setObjectName("menubar")
        self.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(self)
        self.statusbar.setObjectName("statusbar")
        self.setStatusBar(self.statusbar)

        self.retranslateUi()
        QtCore.QMetaObject.connectSlotsByName(self)

        self.disable_update() # disable all the update functions until the serial is connected

    def retranslateUi(self):
        _translate = QtCore.QCoreApplication.translate
        self.setWindowTitle(_translate("MainWindow", "Single Motor FOC Test"))
        self.groupBox.setTitle(_translate("MainWindow", "Configuration:"))
        self.serialConnectBtn.setText(_translate("MainWindow", "Connect"))
        self.label_8.setText(_translate("MainWindow", "torque"))
        self.label_can.setText(_translate("MainWindow", "CAN_ID:"))
        self.label_position.setText(_translate("MainWindow", "Position"))
        self.label_7.setText(_translate("MainWindow", "velocity"))
        self.label_kpkd.setText(_translate("MainWindow", "Kp,Kd:"))
        self.label_serial.setText(_translate("MainWindow", "Serial port"))

        self.lineEdit_can.setText('1')
        self.lineEdit.setText('50,1')
        self.lineEdit_2.setText('0')
        self.lineEdit_3.setText('0')

    def setup_callback(self):
        self.serialConnectBtn.clicked.connect(self.connect_to_motor)
        self.positionSlider.valueChanged.connect(self.update_position)
        self.lineEdit_can.returnPressed.connect(self.update_can_id)

    def disable_update(self):
        self.serialConnectBtn.setText('Connect')
        self.serialConnectBtn.setChecked(False)
        self.serialConnectBtn.setStyleSheet(
            'background-color: white')
        self.positionSlider.setEnabled(False)
        self.lineEdit.setEnabled(False)
        self.lineEdit_2.setEnabled(False)
        self.lineEdit_3.setEnabled(False)
        self.serialSelector.setEnabled(True)

    def enable_update(self):
        self.serialConnectBtn.setText('Disconnect')
        self.serialConnectBtn.setChecked(True)
        self.serialConnectBtn.setStyleSheet(
            'background-color: green')
        self.positionSlider.setEnabled(True)
        self.lineEdit.setEnabled(True)
        self.lineEdit_2.setEnabled(True)
        self.lineEdit_3.setEnabled(True)
        self.serialSelector.setEnabled(False)
    
    def connect_to_motor(self, checked):
        if checked:
            try:
                self.serialPort = self.serialSelector.currentText()
                print('connecting to motor {}'.format(self.serialPort))
                self.can = serialCAN(port=self.serialPort, canid=self.canid)
                if self.can.is_connected:  # meaning the handshake with the motor was successful
                    self.statusBar().showMessage('connect to {}'.format(self.serialPort))
                    if self.can.enterMotorMode():
                        self.enable_update()
                        self.target_position = 0
                        self.can.set(position=0, velocity=0, torque=0, kp=50, kd=1)
                        self.statusBar().showMessage('connected to {}, CANID={}, position={}'.format(self.serialPort, self.canid, 0))
                        self.timer.start()
                    else:
                        self.statusBar().showMessage('enter motor mode failed, check the CAN_ID')
                        self.serialConnectBtn.toggle()
                        self.can.close()
            except:
                print('could not connect to motor')
                self.statusBar().showMessage('cannot connect to {}'.format(self.serialPort))
                self.serialConnectBtn.toggle()

        elif checked==0:
            if self.can.exitMotorMode():
                self.timer.stop()
                self.disable_update()
            self.can.exitMotorMode()
            self.can.close()
    
    @property
    def canid(self):
        return int(self.lineEdit_can.text())

    def update_can_id(self):
        try:
            self.can.canid = int(self.lineEdit_can.text())
            print('can id set to {}'.format(self.can.canid))
        except:
            print('fail to change CAN ID at {}'.format(self.serialPort))
            self.can.canid = 1
    
    def update_position(self):
        # print('updating position')
        # print(self.positionSlider.value())
        # resolution is np.pi*2/1000 (0.001 rad)
        self.target_position = self.positionSlider.value()/1000
        self.can.position = self.target_position
        self.statusBar().showMessage(
            'connected to {}, CANID={}, position={} rad'.format(self.serialPort, self.canid, self.can.position))

    def setup_plot(self, n_data=500):
        self.dt = 10 # ms plot rate

        # define a pyqtgraph GraphicsView that can be embedded in a Qt application
        graph_view = pg.GraphicsView()
        self.verticalLayout.addWidget(graph_view)

        # define a graph_layout which hosts all the plots
        graph_layout = pg.GraphicsLayout()
        graph_view.setCentralItem(graph_layout)
        graph_layout.setSpacing(0.)
        graph_layout.setContentsMargins(-10., -10., -10., -10.)

        # Add three subplots: position, velocity, and torque
        self.graphWidget_position = graph_layout.addPlot(0, 0)
        self.graphWidget_velocity = graph_layout.addPlot(1, 0)
        self.graphWidget_torque = graph_layout.addPlot(2, 0)

        # Initialize thd data to plot before data acquisition, all be zeros
        self.targetposdata = [0 for i in range(n_data)]
        self.posdata = [0 for i in range(n_data)]
        self.veldata = [0 for i in range(n_data)]
        self.tordata = [0 for i in range(n_data)]

        # Acquire the curve to update in real time
        self.curve_target_pos = self.graphWidget_position.plot()
        self.curve_pos = self.graphWidget_position.plot()
        self.curve_vel = self.graphWidget_velocity.plot()
        self.curve_tor = self.graphWidget_torque.plot()

        # set the x axis to show the xlabel ticks as time units
        ax0 = self.graphWidget_position.getAxis('bottom')
        ax0.setTicks([[(i, str(i*self.dt*1e-3)) 
                       for i in range(0, n_data, 100)]])

        ax1 = self.graphWidget_velocity.getAxis('bottom')
        ax1.setTicks([[(i, str(i*self.dt*1e-3))
                       for i in range(0, n_data, 100)]])

        ax2 = self.graphWidget_torque.getAxis('bottom')
        ax2.setTicks([[(i, str(i*self.dt*1e-3))
                       for i in range(0, n_data, 100)]])

        # This links three x-axis together
        self.graphWidget_velocity.setXLink(self.graphWidget_position)
        self.graphWidget_torque.setXLink(self.graphWidget_position)

        # set YRange of each y-axis
        self.graphWidget_position.setYRange(0, 6.29, padding=0)
        self.graphWidget_velocity.setYRange(-50, 50, padding=0)
        self.graphWidget_torque.setYRange(-20, 20, padding=0)

        # This align the x position of three y-axes, by setting margin between ylabel and y-axis itself
        self.graphWidget_position.getAxis('left').setWidth(80)
        self.graphWidget_velocity.getAxis('left').setWidth(80)
        self.graphWidget_torque.getAxis('left').setWidth(80)

        # set ylabel of each y-axis
        self.graphWidget_position.setLabel('left', 'Position', units='Rad')
        self.graphWidget_velocity.setLabel('left', 'Velocity', units='Rad/s')
        self.graphWidget_torque.setLabel('left', 'Torque', units='Nm')
        self.graphWidget_torque.setLabel('bottom', 'Time', units='s')

        # Setup a timer to trigger the redraw by calling update_plot.
        self.timer = QtCore.QTimer()
        self.timer.setInterval(self.dt)
        self.timer.timeout.connect(self.update_plot)

    def update_plot(self):
        self.can.refresh()
        self.targetposdata = self.targetposdata[1:] + [self.target_position]
        self.posdata = self.posdata[1:] + [self.can._read_position_rad]
        self.veldata = self.veldata[1:] + [self.can._read_speed_rad]
        self.tordata = self.tordata[1:] + [self.can._read_torque]
        self.curve_target_pos.setData(self.targetposdata, symbolSize=4, symbolBrush=('b'))
        self.curve_pos.setData(self.posdata, symbolSize=4, symbolBrush=('r'))
        self.curve_vel.setData(self.veldata)
        self.curve_tor.setData(self.tordata)


if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    ui = Ui_MainWindow()
    ui.setupUi()
    ui.setup_callback()
    ui.setup_plot()
    ui.show()
    sys.exit(app.exec_())
