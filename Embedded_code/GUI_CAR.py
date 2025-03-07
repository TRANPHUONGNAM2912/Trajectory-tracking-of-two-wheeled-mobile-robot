import sys
from PyQt6 import QtCore, QtGui, QtWidgets
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
import requests
import threading
import numpy as np
import random
ip_address = "192.168.250.130"
#192.168.43.85
class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1300, 650)
        MainWindow.setContextMenuPolicy(QtCore.Qt.ContextMenuPolicy.DefaultContextMenu)
        MainWindow.setLayoutDirection(QtCore.Qt.LayoutDirection.LeftToRight)
        MainWindow.setAutoFillBackground(False)
        self.centralwidget = QtWidgets.QWidget(parent=MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.noi_dung = QtWidgets.QLabel(parent=self.centralwidget)
        self.noi_dung.setGeometry(QtCore.QRect(200, 0, 441, 31))
        font = QtGui.QFont()
        font.setFamily("Times New Roman")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.noi_dung.setFont(font)
        self.noi_dung.setContextMenuPolicy(QtCore.Qt.ContextMenuPolicy.DefaultContextMenu)
        self.noi_dung.setLayoutDirection(QtCore.Qt.LayoutDirection.LeftToRight)
        self.noi_dung.setAutoFillBackground(False)
        self.noi_dung.setStyleSheet("background:rgb(85, 255, 255)")
        self.noi_dung.setLineWidth(1)
        self.noi_dung.setTextFormat(QtCore.Qt.TextFormat.AutoText)
        self.noi_dung.setScaledContents(False)
        self.noi_dung.setWordWrap(False)
        self.noi_dung.setObjectName("noi_dung")
        # self.toc_do_do1 = QtWidgets.QLabel(parent=self.centralwidget)
        # self.toc_do_do1.setGeometry(QtCore.QRect(280, 170, 31, 31))
        # self.toc_do_do1.setStyleSheet("background:rgb(85, 255, 127)")
        # self.toc_do_do1.setObjectName("toc_do_do1")
        # self.vt = QtWidgets.QLineEdit(parent=self.centralwidget)
        # self.vt.setGeometry(QtCore.QRect(310, 170, 41, 31))
        # self.vt.setObjectName("vt")
        self.start = QtWidgets.QPushButton(parent=self.centralwidget)
        self.start.setGeometry(QtCore.QRect(220, 50, 71, 31))
        self.start.setStyleSheet("background:rgb(0, 255, 0)")
        self.start.setObjectName("start")
        self.reset = QtWidgets.QPushButton(parent=self.centralwidget)
        self.reset.setGeometry(QtCore.QRect(510, 50, 71, 31))
        self.reset.setStyleSheet("background:rgb(170, 255, 0)")
        self.reset.setObjectName("reset")
        self.continuee = QtWidgets.QPushButton(parent=self.centralwidget)
        self.continuee.setGeometry(QtCore.QRect(365, 50, 71, 31))
        self.continuee.setStyleSheet("background:rgb(255, 225, 0)")
        self.continuee.setObjectName("stop")
        # self.Xn = QtWidgets.QLineEdit(parent=self.centralwidget)
        # self.Xn.setGeometry(QtCore.QRect(80, 90, 41, 31))
        # self.Xn.setObjectName("Xn")
        # self.Yn = QtWidgets.QLineEdit(parent=self.centralwidget)
        # self.Yn.setGeometry(QtCore.QRect(80, 130, 41, 31))
        # self.Yn.setObjectName("Yn")
        # self.Yd = QtWidgets.QLineEdit(parent=self.centralwidget)
        # self.Yd.setGeometry(QtCore.QRect(680, 130, 41, 31))
        # self.Yd.setObjectName("Yd")
        # self.Xd = QtWidgets.QLineEdit(parent=self.centralwidget)
        # self.Xd.setGeometry(QtCore.QRect(680, 90, 41, 31))
        # self.Xd.setObjectName("Xd")
        # self.label_3 = QtWidgets.QLabel(parent=self.centralwidget)
        # self.label_3.setGeometry(QtCore.QRect(40, 90, 41, 31))
        # self.label_3.setStyleSheet("Background:rgb(85, 170, 255)")
        # self.label_3.setObjectName("label_3")
        # self.label_4 = QtWidgets.QLabel(parent=self.centralwidget)
        # self.label_4.setGeometry(QtCore.QRect(40, 130, 41, 31))
        # self.label_4.setStyleSheet("Background:rgb(85, 170, 255)")
        # self.label_4.setObjectName("label_4")
        # self.label_5 = QtWidgets.QLabel(parent=self.centralwidget)
        # self.label_5.setGeometry(QtCore.QRect(720, 90, 41, 31))
        # self.label_5.setStyleSheet("Background:rgb(85, 170, 255)")
        # self.label_5.setObjectName("label_5")
        # self.label_6 = QtWidgets.QLabel(parent=self.centralwidget)
        # self.label_6.setGeometry(QtCore.QRect(720, 130, 41, 31))
        # self.label_6.setStyleSheet("Background:rgb(85, 170, 255)")
        # self.label_6.setObjectName("label_6")
        # self.thetan = QtWidgets.QLineEdit(parent=self.centralwidget)
        # self.thetan.setGeometry(QtCore.QRect(80, 170, 41, 31))
        # self.thetan.setObjectName("thetan")
        # self.theta = QtWidgets.QLabel(parent=self.centralwidget)
        # self.theta.setGeometry(QtCore.QRect(40, 170, 41, 31))
        # self.theta.setStyleSheet("Background:rgb(85, 170, 255)")
        # self.theta.setObjectName("theta")
        # self.thetad = QtWidgets.QLineEdit(parent=self.centralwidget)
        # self.thetad.setGeometry(QtCore.QRect(680, 170, 41, 31))
        # self.thetad.setObjectName("giatriangle")
        # self.angle = QtWidgets.QLabel(parent=self.centralwidget)
        # self.angle.setGeometry(QtCore.QRect(720, 170, 41, 31))
        # self.angle.setStyleSheet("Background:rgb(85, 170, 255)")
        # self.angle.setObjectName("angle")
        # self.bankinh = QtWidgets.QLabel(parent=self.centralwidget)
        # self.bankinh.setGeometry(QtCore.QRect(280, 90, 31, 31))
        # self.bankinh.setStyleSheet("background:rgb(255, 170, 0)")
        # self.bankinh.setObjectName("bankinh")
        # self.so_ban_kinh = QtWidgets.QLineEdit(parent=self.centralwidget)
        # self.so_ban_kinh.setGeometry(QtCore.QRect(310, 90, 41, 31))
        # self.so_ban_kinh.setObjectName("so_ban_kinh")
        # self.lineEdit_3 = QtWidgets.QLineEdit(parent=self.centralwidget)
        # self.lineEdit_3.setGeometry(QtCore.QRect(350, 90, 31, 31))
        # self.lineEdit_3.setObjectName("lineEdit_3")
        # self.goc = QtWidgets.QLabel(parent=self.centralwidget)
        # self.goc.setGeometry(QtCore.QRect(420, 90, 31, 31))
        # self.goc.setStyleSheet("background:rgb(255, 170, 0)")
        # self.goc.setObjectName("goc")
        # self.so_goc = QtWidgets.QLineEdit(parent=self.centralwidget)
        # self.so_goc.setGeometry(QtCore.QRect(450, 90, 41, 31))
        # self.so_goc.setObjectName("so_goc")
        # self.lineEdit_4 = QtWidgets.QLineEdit(parent=self.centralwidget)
        # self.lineEdit_4.setGeometry(QtCore.QRect(490, 90, 31, 31))
        # self.lineEdit_4.setObjectName("lineEdit_4")
        # self.label_7 = QtWidgets.QLabel(parent=self.centralwidget)
        # self.label_7.setGeometry(QtCore.QRect(280, 130, 31, 31))
        # self.label_7.setStyleSheet("background:rgb(255, 85, 0)")
        # self.label_7.setObjectName("label_7")
        # self.label_8 = QtWidgets.QLabel(parent=self.centralwidget)
        # self.label_8.setGeometry(QtCore.QRect(420, 130, 31, 31))
        # self.label_8.setStyleSheet("background:rgb(255, 85, 0)")
        # self.label_8.setObjectName("label_8")
        # self.vc = QtWidgets.QLineEdit(parent=self.centralwidget)
        # self.vc.setGeometry(QtCore.QRect(310, 130, 41, 31))
        # self.vc.setObjectName("vc")
        # self.wc = QtWidgets.QLineEdit(parent=self.centralwidget)
        # self.wc.setGeometry(QtCore.QRect(450, 130, 41, 31))
        # self.wc.setObjectName("wc")
        # self.lineEdit_6 = QtWidgets.QLineEdit(parent=self.centralwidget)
        # self.lineEdit_6.setGeometry(QtCore.QRect(350, 130, 31, 31))
        # self.lineEdit_6.setObjectName("lineEdit_6")
        # self.lineEdit_7 = QtWidgets.QLineEdit(parent=self.centralwidget)
        # self.lineEdit_7.setGeometry(QtCore.QRect(490, 130, 31, 31))
        # self.lineEdit_7.setObjectName("lineEdit_7")
        # self.lineEdit_8 = QtWidgets.QLineEdit(parent=self.centralwidget)
        # self.lineEdit_8.setGeometry(QtCore.QRect(350, 170, 31, 31))
        # self.lineEdit_8.setObjectName("lineEdit_8")
        # self.toc_do_do1_2 = QtWidgets.QLabel(parent=self.centralwidget)
        # self.toc_do_do1_2.setGeometry(QtCore.QRect(420, 170, 31, 31))
        # self.toc_do_do1_2.setStyleSheet("background:rgb(85, 255, 127)")
        # self.toc_do_do1_2.setObjectName("toc_do_do1_2")
        # self.wt = QtWidgets.QLineEdit(parent=self.centralwidget)
        # self.wt.setGeometry(QtCore.QRect(450, 170, 41, 31))
        # self.wt.setObjectName("wt")
        # self.lineEdit_9 = QtWidgets.QLineEdit(parent=self.centralwidget)
        # self.lineEdit_9.setGeometry(QtCore.QRect(490, 170, 31, 31))
        # self.lineEdit_9.setObjectName("lineEdit_9")
        # self.label = QtWidgets.QLabel(parent=self.centralwidget)
        # self.label.setGeometry(QtCore.QRect(160, 90, 41, 31))
        # self.label.setStyleSheet("Background:rgb(85, 170, 255)")
        # self.label.setObjectName("label")
        # self.label_2 = QtWidgets.QLabel(parent=self.centralwidget)
        # self.label_2.setGeometry(QtCore.QRect(160, 130, 41, 31))
        # self.label_2.setStyleSheet("Background:rgb(85, 170, 255)")
        # self.label_2.setObjectName("label_2")
        # self.label_9 = QtWidgets.QLabel(parent=self.centralwidget)
        # self.label_9.setGeometry(QtCore.QRect(160, 170, 41, 31))
        # self.label_9.setStyleSheet("Background:rgb(85, 170, 255)")
        # self.label_9.setObjectName("label_9")
        # self.ex = QtWidgets.QLineEdit(parent=self.centralwidget)
        # self.ex.setGeometry(QtCore.QRect(200, 90, 41, 31))
        # self.ex.setObjectName("ex")
        # self.ey = QtWidgets.QLineEdit(parent=self.centralwidget)
        # self.ey.setGeometry(QtCore.QRect(200, 130, 41, 31))
        # self.ey.setObjectName("ey")
        # self.etheta = QtWidgets.QLineEdit(parent=self.centralwidget)
        # self.etheta.setGeometry(QtCore.QRect(200, 170, 41, 31))
        # self.etheta.setObjectName("etheta")
        # self.label_10 = QtWidgets.QLabel(parent=self.centralwidget)
        # self.label_10.setGeometry(QtCore.QRect(600, 170, 41, 31))
        # self.label_10.setStyleSheet("Background:rgb(85, 170, 255)")
        # self.label_10.setObjectName("label_10")
        # self.label_11 = QtWidgets.QLabel(parent=self.centralwidget)
        # self.label_11.setGeometry(QtCore.QRect(600, 130, 41, 31))
        # self.label_11.setStyleSheet("Background:rgb(85, 170, 255)")
        # self.label_11.setObjectName("label_11")
        # self.ky = QtWidgets.QLineEdit(parent=self.centralwidget)
        # self.ky.setGeometry(QtCore.QRect(560, 130, 41, 31))
        # self.ky.setObjectName("ky")
        # self.kx = QtWidgets.QLineEdit(parent=self.centralwidget)
        # self.kx.setGeometry(QtCore.QRect(560, 90, 41, 31))
        # self.kx.setObjectName("kx")
        # self.ktheta = QtWidgets.QLineEdit(parent=self.centralwidget)
        # self.ktheta.setGeometry(QtCore.QRect(560, 170, 41, 31))
        # self.ktheta.setObjectName("lineEdit_13")
        # self.label_12 = QtWidgets.QLabel(parent=self.centralwidget)
        # self.label_12.setGeometry(QtCore.QRect(600, 90, 41, 31))
        # self.label_12.setStyleSheet("Background:rgb(85, 170, 255)")
        # self.label_12.setObjectName("label_12")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(parent=MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 18))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(parent=MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        # # Thêm một Widget cho Matplotlib
        # self.plot_widget = QtWidgets.QWidget(self.centralwidget)
        # self.plot_widget.setGeometry(QtCore.QRect(85, 220, 631, 411))

        # # Tạo một Figure và Canvas của Matplotlib
        # self.figure = plt.figure()
        # self.canvas = FigureCanvas(self.figure)
        # self.canvas.setParent(self.plot_widget)
        self.reset.clicked.connect(self.resetall)
        self.start.clicked.connect(self.toggleButton)
        self.continuee.clicked.connect(self.toggleButton1)
        self.is_running = False
        self.update_thread = threading.Thread(target=self.update_values)
        self.update_thread.start()
        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def update_values(self):
        while self.update_thread_running:
            if self.is_running:  
                try:
                    response = requests.get(f"http://{self.ip_address}/")
                    if response.status_code == 200:
                        data = response.json()
                        self.update_ui_with_data(data)
                    else:
                        print(f"Failed to fetch data. Status code: {response.status_code}")
                except Exception as e:
                    print("Error:", e)
                # QtCore.QThread.msleep(300)
            else:
                QtCore.QThread.msleep(300) 
    
    def resetall(self):
        self.send_request(f"http://{self.ip_address}/reset")
        # # Xóa tất cả các điểm cũ
        self.points = []
        self.points1 = []
        self.points2 = []
        self.points3 = []

        # # Xóa tất cả các giá trị x và y của điểm cũ
        self.x_values = []
        self.y_values = []
        self.x_values1 = []
        self.y_values1 = []
        self.x_values2 = []
        self.y_values2 = []

        
        self.ax.clear()
        self.ax.set_xlim(-100, 100)
        self.ax.set_ylim(-100, 100)
        self.ax.set_xticks(range(-100, 101, 100))
        self.ax.set_yticks(range(-100, 101, 100))
        self.ax.set_xlabel('X axis label(cm)')
        self.ax.set_ylabel('Y axis label(cm)')
        # default_point = (0, 0)
        # self.ax.plot(*default_point, 'bo') 
        # self.ax.axhline(0, color='black', linewidth=0.5)
        # self.ax.axvline(0, color='black', linewidth=0.5) 

        legend_elements = [
            Line2D([], [], color='blue', lw=2, label='Reference'),
            Line2D([], [], color='red', lw=2, label='Trajectory Tracking')
        ]
        self.ax.legend(handles=legend_elements, loc='upper center', bbox_to_anchor=(0.5, 1.15), ncol=2)
        self.canvas.draw()
        
    def toggleButton(self):
        if self.is_running:
            self.is_running = False
            self.start.setText("START")
            self.start.setStyleSheet("background:rgb(0, 255, 0)")
            self.send_request(f"http://{self.ip_address}/stop")  # Dừng việc cập nhật giá trị khi STOP được bấm
        else:
            self.is_running = True
            self.start.setText("STOP")
            self.start.setStyleSheet("background:rgb(255, 0, 0)")
            if self.is_running:
                self.send_values_to_server()
    def toggleButton1(self):
        self.send_values_to_server1()

    def stop_update_thread(self):
        self.update_thread_running = False  
        if self.update_thread and self.update_thread.is_alive():
            self.update_thread.join()
    def __init__(self):
        self.update_thread_running = True
    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.noi_dung.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\">CONTROL DIFFERENTIAL 2-WHEELED ROBOT </p></body></html>"))
        # self.toc_do_do1.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-size:10pt;\">Vt</span></p></body></html>"))
        self.start.setText(_translate("MainWindow", "START"))
        self.continuee.setText(_translate("MainWindow", "CONTINUE"))
        self.reset.setText(_translate("MainWindow", "RESET"))
        # self.label_3.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-size:10pt;\">X</span></p></body></html>"))
        # self.label_4.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-size:10pt;\">Y</span></p></body></html>"))
        # self.label_5.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-size:10pt;\">Xd</span></p></body></html>"))
        # self.label_6.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-size:10pt;\">Yd</span></p></body></html>"))
        # self.theta.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-size:10pt;\">Theta</span></p></body></html>"))
        # self.angle.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-size:10pt;\">Thetad</span></p></body></html>"))
        # self.bankinh.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\">R</p></body></html>"))
        # self.lineEdit_3.setWhatsThis(_translate("MainWindow", "<html><head/><body><p align=\"center\">cm</p></body></html>"))
        # self.lineEdit_3.setText(_translate("MainWindow", "cm"))
        # self.goc.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\">W</p></body></html>"))
        # self.lineEdit_4.setText(_translate("MainWindow", "rad/s"))
        # self.label_7.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-size:10pt;\">Vc</span></p></body></html>"))
        # self.label_8.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\">Wc</p></body></html>"))
        # self.lineEdit_6.setText(_translate("MainWindow", "m/s"))
        # self.lineEdit_7.setText(_translate("MainWindow", "rad/s"))
        # self.lineEdit_8.setText(_translate("MainWindow", "m/s"))
        # self.toc_do_do1_2.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\">Wt</p></body></html>"))
        # self.lineEdit_9.setText(_translate("MainWindow", "rad/s"))
        # self.label.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-size:10pt;\">ex</span></p></body></html>"))
        # self.label_2.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-size:10pt;\">ey</span></p></body></html>"))
        # self.label_9.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-size:10pt;\">etheta</span></p></body></html>"))
        # self.label_10.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-size:10pt;\">Ktheta</span></p></body></html>"))
        # self.label_11.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-size:10pt;\">Ky</span></p></body></html>"))
        # self.label_12.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-size:10pt;\">Kx</span></p></body></html>"))
        
class CoordPlotApp(QtWidgets.QMainWindow, Ui_MainWindow):
    ip_address = "192.168.250.130"
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setGeometry(0, 0, 1300, 650)
        # self.setWindowTitle('Đồ Án 1')
        self.plot_widget = QtWidgets.QWidget(self.centralwidget)
        self.plot_widget.setGeometry(QtCore.QRect(85, 80, 571, 571))
        self.figure = Figure(figsize=(5.71, 5.71), dpi=100)
        self.canvas = FigureCanvas(self.figure)
        layout = QtWidgets.QVBoxLayout(self.plot_widget)
        layout.addWidget(self.canvas)

        self.ax = self.figure.add_subplot(111)
        self.ax.set_xlim(-100, 100)
        self.ax.set_ylim(-100, 100)
        self.ax.set_xticks(range(-100, 101, 100))
        self.ax.set_yticks(range(-100, 101, 100))

        # Đặt tiêu đề cho trục x và trục y
        self.ax.set_xlabel('X axis label(cm)')
        self.ax.set_ylabel('Y axis label(cm)')
        self.points1 = []
        self.points2 = []
        self.points3 = []
        # Tạo đường thẳng song song với trục x và đi qua điểm (0, 0)
        # self.ax.axhline(0, color='black', linewidth=0.5)  # Horizontal line at y=0 in black color and linewidth 1.0
        # self.ax.axvline(0, color='black', linewidth=0.5) 
        # Tạo các phần tử Line2D cho chú thích
        legend_elements = [
            Line2D([], [], color='blue', lw=2, label='Reference'),
            Line2D([], [], color='red', lw=2, label='Trajectory Tracking')
            
        ]

        # Đặt vị trí chú thích và hiển thị ngay khi chạy chương trình
        self.ax.legend(handles=legend_elements, loc='upper center', bbox_to_anchor=(0.5, 1.15), ncol=3)
        self.canvas.draw()
        self.plot_widget2 = QtWidgets.QWidget(self.centralwidget)
        self.plot_widget2.setGeometry(QtCore.QRect(780, 0, 531, 311))

        # Create a new Figure and Canvas for the second plot
        self.figure2 = plt.Figure(figsize=(5.31, 3.11), dpi=75)
        self.canvas2 = FigureCanvas(self.figure2)
        layout2 = QtWidgets.QVBoxLayout(self.plot_widget2)
        layout2.addWidget(self.canvas2)
        self.ax2 = self.figure2.add_subplot(111)

        # Customize the appearance of the second plot as needed
        self.ax2.set_xlim(0, 120)
        # self.ax2.set_ylim(-10, 10)
        # self.ax2.set_xticks(range(-5, 5.1, 5))
        # self.ax2.set_yticks(range(-5, 5.1, 5))
        self.ax2.set_xlabel('Time[s]')
        self.ax2.set_ylabel('ex[cm], ey[cm], etheta[rad/s]')

        self.ax2.axhline(0, color='black', linewidth=0.5)
        # self.ax2.axvline(0, color='black', linewidth=0.5)

        legend_elements2 = [
            Line2D([], [], color='green', lw=1, label='ex'),
            Line2D([], [], color='magenta', lw=1, label='ey'),
            Line2D([], [], color='cyan', lw=1, label='etheta')
            ]
        self.ax2.legend(handles=legend_elements2, loc='upper center', bbox_to_anchor=(0.5, 1.15), ncol=3)
        self.canvas2.draw()

        self.plot_widget3 = QtWidgets.QWidget(self.centralwidget)
        self.plot_widget3.setGeometry(QtCore.QRect(780, 320, 531, 311))


        self.figure3 = plt.Figure(figsize=(5.31, 3.11), dpi=75)
        self.canvas3 = FigureCanvas(self.figure3)
        layout3 = QtWidgets.QVBoxLayout(self.plot_widget3)
        layout3.addWidget(self.canvas3)
        self.ax3 = self.figure3.add_subplot(111)

        self.ax3.set_xlim(0, 120)  
        # self.ax3.set_ylim(-2, 2)  
        self.ax3.set_xlabel('Time[s]')
        self.ax3.set_ylabel('Vd[m/s], Wd[rad/s]')
        self.ax3.axhline(0, color='black', linewidth=0.5)


        legend_elements3 = [
            Line2D([], [], color='lime', lw=1, label='Vd '),
            Line2D([], [], color='blue', lw=1, label='Wd'),
            
            ]
        self.ax3.legend(handles=legend_elements3, loc='upper center', bbox_to_anchor=(0.5, 1.15), ncol=2)
        self.canvas3.draw()

        self.canvas.mpl_connect('scroll_event', self.on_scroll)
        self.canvas2.mpl_connect('scroll_event', self.on_scroll2)
        self.canvas3.mpl_connect('scroll_event', self.on_scroll3)
        self.x_values = []
        self.y_values = []
        self.x_values1 = []
        self.y_values1 = []
        self.x_values2 = []
        self.y_values2 = []
        self.ex_point = []
        self.ey_point = []
        self.etheta_point = []
        self.t_point = []
        self.vc_point = []
        self.wc_point = []

    def update_ui_with_data(self, data):
        try:
            # tocdodem = data["tocdomotor"]
            toadox = data["toadox"]
            toadoy = data["toadoy"]
            xdat = data["xdat"]
            ydat = data["ydat"]
            theta = data["theta"]
            ex = data["ex"]
            ey = data["ey"]
            eth = data["eth"]
            Thetad = data["Thetad"]
            vdat = data["Vc"]
            wdat = data["Wc"]
            # wt = data["wthuc"]
            t = data["t"]
            # x_world=float(data["xword"])
            # y_world=float(data["yword"])
            # self.wt.setText(str(wt))
            # self.ex.setText(str(ex))
            # self.ey.setText(str(ey))
            # self.etheta.setText(str(eth))
            # # self.vt.setText(str(tocdodem))
            # self.vc.setText(str(vdat))
            # self.wc.setText(str(wdat))
            # self.Xn.setText(str(toadox))
            # self.Yn.setText(str(toadoy))
            # self.Xd.setText(str(xdat))
            # self.Yd.setText(str(ydat))
            # self.thetan.setText(str(theta))
            # self.thetad.setText(str(Thetad))
         
            toadox = float(data["toadox"])
            toadoy = float(data["toadoy"])
            xdat = float(data["xdat"])
            ydat = float(data["ydat"])
            ex = float(data["ex"])
            ey = float(data["ey"])
            etheta = float(data["eth"])
            t = float(data["t"])
            kx = float(data["Kx"])
            ky = float(data["Ky"])
            kth = float(data["Kth"])
            # print(kx)
            # if ex < 2 or ex>-2:
            #     ex = random.uniform(-0.25, 0.25)
            # if ey < 2 or ey>-2:
            #     ey = random.uniform(-0.25, 0.25)
                # print(ex)
            self.t_point.append(t)
            self.ex_point.append(ex)
            self.ey_point.append(ey)
            self.etheta_point.append(etheta)
            self.vc_point.append(vdat)
            self.wc_point.append(wdat)

            if hasattr(self, 'line'):
                self.line.remove()

            self.points2.append((xdat, ydat))
            x_values2, y_values2 = zip(*self.points2)
            self.line, = self.ax.plot(x_values2, y_values2, 'b-')

            self.points1.append((toadox, toadoy))
            x_values1, y_values1 = zip(*self.points1)
            self.line, = self.ax.plot(x_values1, y_values1, 'r-')
            
            # self.points3.append((x_world, y_world))
            # x_values3, y_values3 = zip(*self.points3)
            # self.line, = self.ax.plot(x_values3, y_values3, 'lime')
            current_xlim = self.ax2.get_xlim()
            if t >= current_xlim[1]:
                new_xlim = (current_xlim[1], current_xlim[1] + 120)
                self.ax2.set_xlim(new_xlim)
            current_xlim2 = self.ax3.get_xlim()
            if t >= current_xlim2[1]:
                new_xlim2 = (current_xlim2[1], current_xlim2[1] + 120)
                self.ax3.set_xlim(new_xlim2)
            
            # current_ylim = self.ax2.get_ylim()
            # min_value = min(ex, ey, etheta)
            # max_value = max(ex, ey, etheta)

            # if min_value < current_ylim[0] or max_value > current_ylim[1]:
            # new_ylim = (min_value * 2, max_value * 2)  # You can adjust the multiplication factor as needed
            # self.ax2.set_ylim(new_ylim)

            if hasattr(self, 'line_ex'):
                self.line_ex.remove()
            self.line_ex, = self.ax2.plot(self.t_point, self.ex_point, 'g-')

            if hasattr(self, 'line_ey'):
                self.line_ey.remove()
            self.line_ey, = self.ax2.plot(self.t_point, self.ey_point, 'm-')

            if hasattr(self, 'line_etheta'):
                self.line_etheta.remove()
            self.line_etheta, = self.ax2.plot(self.t_point, self.etheta_point, 'c-')

            if hasattr(self, 'line_vc'):
                self.line_vc.remove()
            self.line_vc, = self.ax3.plot(self.t_point, self.vc_point, 'lime')

            if hasattr(self, 'line_wc'):
                self.line_wc.remove()
            self.line_wc, = self.ax3.plot(self.t_point, self.wc_point, 'b-')

            
            self.canvas.draw()
            self.canvas2.draw()
            self.canvas3.draw()
        except KeyError as e:
            print("Missing key in response data:", e)
            self.canvas.draw()
            self.canvas2.draw()
            self.canvas3.draw()

    def on_scroll(self, event):
        if event.inaxes == self.ax:
            factor = 1.1  
            xlim = self.ax.get_xlim()
            ylim = self.ax.get_ylim()

            if event.button == 'up':
                new_xlim = (xlim[0] * factor, xlim[1] * factor)
                new_ylim = (ylim[0] * factor, ylim[1] * factor)
            elif event.button == 'down':
                new_xlim = (xlim[0] / factor, xlim[1] / factor)
                new_ylim = (ylim[0] / factor, ylim[1] / factor)

            self.ax.set_xlim(new_xlim)
            self.ax.set_ylim(new_ylim)
            self.ax.set_xticks(range(int(new_xlim[0]), int(new_xlim[1]) + 1, 50))
            self.ax.set_yticks(range(int(new_ylim[0]), int(new_ylim[1]) + 1, 50))
            self.canvas.draw()

    def on_scroll2(self, event):
        if event.inaxes == self.ax2:
            factor = 1.5  
            ylim2 = self.ax2.get_ylim()

            if event.button == 'up':
                # self.ax2.set_ylim(-2, 2)
                new_ylim2 = (ylim2[0] * factor, ylim2[1] * factor)
            elif event.button == 'down':
                new_ylim2 = (ylim2[0] / factor, ylim2[1] / factor)
                # self.ax2.set_ylim(-3, 3)

            self.ax2.set_ylim(new_ylim2)
            # self.ax2.set_ylim(-3, 3)
            self.canvas2.draw()
    def on_scroll3(self, event):
        if event.inaxes == self.ax3:
            factor = 1.5  
            ylim3 = self.ax3.get_ylim()

            if event.button == 'up':
                new_ylim3 = (ylim3[0] * factor, ylim3[1] * factor)
            elif event.button == 'down':
                new_ylim3 = (ylim3[0] / factor, ylim3[1] / factor)

            self.ax3.set_ylim(new_ylim3)
            self.canvas3.draw()
    def send_request(self, url):
            try:
                response = requests.get(url)
                if response.status_code != 200:
                    print(f"Failed to send request. Status code: {response.status_code}")
            except Exception as e:
                print("Error sending request:", e)
    
    def send_values_to_server(self):
        if self.is_running: 
            url = f"http://{self.ip_address}/start?"
            self.send_request(url)

    def send_values_to_server1(self): 
        bankinh_value = self.so_ban_kinh.text()
        omega_value = self.so_goc.text()
        ktheta_value = self.ktheta.text()
        kx_value = self.kx.text()
        ky_value = self.ky.text()
        url = f"http://{self.ip_address}/continuee?bankinh={bankinh_value}&omega={omega_value}&kx={kx_value}&ky={ky_value}&ktheta={ktheta_value}"
        self.send_request(url)

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    window = CoordPlotApp() 
    window.show()
    app.aboutToQuit.connect(ui.stop_update_thread)
    app.aboutToQuit.connect(window.stop_update_thread)
    sys.exit(app.exec())