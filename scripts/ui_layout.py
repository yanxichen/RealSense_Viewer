from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    """
    Setting up the UI layout
    """
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(758, 614)
        MainWindow.setDocumentMode(False)

        # Initialize central widget
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setStyleSheet("")
        self.centralwidget.setObjectName("centralwidget")

        # Snapshot button
        self.Snapshot = QtWidgets.QPushButton(self.centralwidget)
        self.Snapshot.setGeometry(QtCore.QRect(200, 510, 113, 32))
        self.Snapshot.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.Snapshot.setStyleSheet("font: 13pt \"Cochin\";")
        self.Snapshot.setObjectName("Snapshot")

        # Initialize main frame
        self.main_frame = QtWidgets.QFrame(self.centralwidget)
        self.main_frame.setGeometry(QtCore.QRect(0, 0, 571, 491))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.main_frame.sizePolicy().hasHeightForWidth())

        self.main_frame.setSizePolicy(sizePolicy)
        self.main_frame.setBaseSize(QtCore.QSize(500, 500))
        self.main_frame.setCursor(QtGui.QCursor(QtCore.Qt.OpenHandCursor))
        self.main_frame.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.main_frame.setAutoFillBackground(True)
        self.main_frame.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.main_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.main_frame.setObjectName("main_frame")

        self.widget = QtWidgets.QWidget(self.main_frame)
        self.widget.setGeometry(QtCore.QRect(0, 0, 571, 491))
        self.widget.setObjectName("widget")

        # Initialize reset button
        self.reset = QtWidgets.QPushButton(self.centralwidget)
        self.reset.setGeometry(QtCore.QRect(610, 30, 113, 32))
        self.reset.setStyleSheet("font: 13pt \"Cochin\";")
        self.reset.setObjectName("reset")

        # Roll button
        self.Roll = QtWidgets.QPushButton(self.centralwidget)
        self.Roll.setGeometry(QtCore.QRect(610, 180, 113, 32))
        self.Roll.setStyleSheet("\n"
                                "font: 13pt \"Cochin\";")
        self.Roll.setObjectName("Roll")

        # Azimuth button
        self.Azimuth = QtWidgets.QPushButton(self.centralwidget)
        self.Azimuth.setGeometry(QtCore.QRect(610, 230, 113, 32))
        self.Azimuth.setStyleSheet("\n"
                                   "font: 13pt \"Cochin\";")
        self.Azimuth.setObjectName("Azimuth")

        # Pitch button
        self.Pitch = QtWidgets.QPushButton(self.centralwidget)
        self.Pitch.setGeometry(QtCore.QRect(610, 280, 113, 32))
        self.Pitch.setStyleSheet("\n"
                                 "font: 13pt \"Cochin\";")
        self.Pitch.setObjectName("Pitch")

        MainWindow.setCentralWidget(self.centralwidget)

        # Menu
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 758, 21))
        self.menubar.setObjectName("menubar")
        self.menuFile = QtWidgets.QMenu(self.menubar)
        self.menuFile.setObjectName("menuFile")
        self.menuView = QtWidgets.QMenu(self.menubar)
        self.menuView.setObjectName("menuView")

        MainWindow.setMenuBar(self.menubar)

        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")

        MainWindow.setStatusBar(self.statusbar)

        self.actionSave = QtWidgets.QAction(MainWindow)
        self.actionSave.setObjectName("actionSave")
        self.actionExit = QtWidgets.QAction(MainWindow)
        self.actionExit.setObjectName("actionExit")

        self.menuFile.addAction(self.actionSave)
        self.menuFile.addAction(self.actionExit)

        self.menubar.addAction(self.menuFile.menuAction())
        self.menubar.addAction(self.menuView.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "RealSense---VTK"))
        self.Snapshot.setText(_translate("MainWindow", "Snapshot"))
        self.reset.setText(_translate("MainWindow", "Reset Position"))
        self.Roll.setText(_translate("MainWindow", "Roll"))
        self.Azimuth.setText(_translate("MainWindow", "Azimuth"))
        self.Pitch.setText(_translate("MainWindow", "Pitch"))
        self.menuFile.setTitle(_translate("MainWindow", "File"))
        self.menuView.setTitle(_translate("MainWindow", "View"))
        self.actionSave.setText(_translate("MainWindow", "Save"))
        self.actionExit.setText(_translate("MainWindow", "Exit"))
