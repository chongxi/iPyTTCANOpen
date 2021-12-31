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
        self.positionSlider.setMaximum(6283)
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
    
    def connect_to_motor(self, checked):
        if checked:
            try:
                self.serialPort = self.serialSelector.currentText()
                print('connecting to motor {}'.format(self.serialPort))
                self.can = serialCAN(port=self.serialPort, canid=self.canid)
                if self.can.is_connected:
                    self.statusBar().showMessage('connect to {}'.format(self.serialPort))
                    if self.can.enterMotorMode():
                        self.serialConnectBtn.setText('Disconnect')
                        self.serialConnectBtn.setChecked(True)
                        self.serialConnectBtn.setStyleSheet(
                            'background-color: green')
                        self.positionSlider.setEnabled(True)
                        self.lineEdit.setEnabled(True)
                        self.lineEdit_2.setEnabled(True)
                        self.lineEdit_3.setEnabled(True)
                        self.serialSelector.setEnabled(False)
            except:
                print('could not connect to motor')
                self.statusBar().showMessage('cannot connect to {}'.format(self.serialPort))
                self.serialConnectBtn.setChecked(False)

        elif checked==0:
            if self.can.exitMotorMode():
                self.serialConnectBtn.setText('Connect')
                self.serialConnectBtn.setChecked(False)
                self.serialConnectBtn.setStyleSheet(
                    'background-color: white')
                self.positionSlider.setEnabled(False)
                self.lineEdit.setEnabled(False)
                self.lineEdit_2.setEnabled(False)
                self.lineEdit_3.setEnabled(False)
                self.serialSelector.setEnabled(True)
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
        print('updating position')
        print(self.positionSlider.value())

    def setup_plot(self, n_data=400):
        self.graphWidget_position = pg.PlotWidget()
        self.graphWidget_velocity = pg.PlotWidget()
        self.graphWidget_torque   = pg.PlotWidget()
        self.verticalLayout.addWidget(self.graphWidget_position)
        self.verticalLayout.addWidget(self.graphWidget_velocity)
        self.verticalLayout.addWidget(self.graphWidget_torque)

        self.ydata = [random.randint(0, 1) for i in range(n_data)]

        self.curve_pos = self.graphWidget_position.plot()
        self.curve_vel = self.graphWidget_velocity.plot()
        self.curve_tor = self.graphWidget_torque.plot()

        # Setup a timer to trigger the redraw by calling update_plot.
        self.timer = QtCore.QTimer()
        self.timer.setInterval(10)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()

    def update_plot(self):
        # Drop off the first y element, append a new one.
        self.ydata = self.ydata[1:] + [random.randint(0, 100)]
        # self.graphWidget_position.plot(self.xdata, self.ydata)
        self.curve_pos.setData(self.ydata)
        self.curve_vel.setData(self.ydata)
        self.curve_tor.setData(self.ydata)


if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    ui = Ui_MainWindow()
    ui.setupUi()
    ui.setup_callback()
    ui.setup_plot()
    ui.show()
    sys.exit(app.exec_())
