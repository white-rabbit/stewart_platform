import sys
import yaml
import numpy as np


import OpenGL.GL as gl
import OpenGL.GLU as glu

from PyQt5.QtCore import pyqtSignal, QPoint, QSize, Qt
from PyQt5.QtGui import QColor, QPainter, QFont
from PyQt5.QtWidgets import (QApplication, QHBoxLayout, QVBoxLayout,
    QOpenGLWidget, QSlider, QLabel, QWidget, QPushButton)


from qglscenecontroller import QGLSceneController
from stewart import StewartPlatform


class ApplicationWindow(QWidget):

    def __init__(self, platform_model):
        super(ApplicationWindow, self).__init__()

        self.platform_model = platform_model
        self.glWidget = StewartPlatformWidget(platform_model)

        self._sliders = []

        # todo(savegor): specify limits explicitly
        self.rollLayout = self.createSlider(
            'Roll', self.glWidget.setRoll, self.glWidget.rollValueSignal)
        self.pitchLayout = self.createSlider(
            'Pitch', self.glWidget.setPitch, self.glWidget.pitchValueSignal)
        self.yawLayout = self.createSlider(
            'Yaw', self.glWidget.setYaw, self.glWidget.yawValueSignal)
        self.xLayout = self.createSlider(
            'X', self.glWidget.setX, self.glWidget.xValueSignal)
        self.yLayout = self.createSlider(
            'Y', self.glWidget.setY, self.glWidget.yValueSignal)
        self.zLayout = self.createSlider(
            'Z', self.glWidget.setZ, self.glWidget.zValueSignal)
        self.resetButton = QPushButton('Reset')

        self.resetButton.clicked.connect(self.reset)

        self.glWidget.emitPlatformPosition()

        mainLayout = QHBoxLayout()
        mainLayout.addWidget(self.glWidget)
        sliderLayout = QVBoxLayout()
        sliderLayout.addLayout(self.rollLayout)
        sliderLayout.addLayout(self.pitchLayout)
        sliderLayout.addLayout(self.yawLayout)
        sliderLayout.addLayout(self.xLayout)
        sliderLayout.addLayout(self.yLayout)
        sliderLayout.addLayout(self.zLayout)
        sliderLayout.addWidget(self.resetButton)
        mainLayout.addLayout(sliderLayout)
        self.setLayout(mainLayout)

        self.setWindowTitle("Stewart Platform Demo")

    def createSlider(self, name, onValueChanged, modelValueUpdate=None):
        layout = QHBoxLayout()
        
        label = QLabel(name)

        slider = QSlider(Qt.Horizontal)

        # todo(savegor): provide limits as method argumets
        slider.setRange(-40, 40)
        slider.setSingleStep(1)
        slider.setTickPosition(QSlider.TicksRight)
        slider.setObjectName(name)
        slider.setValue(0)
        slider.valueChanged.connect(onValueChanged)

        value = QLabel()
        if modelValueUpdate:
            modelValueUpdate.connect(value.setText)

        layout.addWidget(label)
        layout.addWidget(slider)
        layout.addWidget(value)
        self._sliders.append(slider)
        return layout

    def reset(self):
        for slider in self._sliders:
            slider.setValue(0)



class StewartPlatformWidget(QGLSceneController):
    rollValueSignal = pyqtSignal(str)
    pitchValueSignal = pyqtSignal(str)
    yawValueSignal = pyqtSignal(str)
    xValueSignal = pyqtSignal(str)
    yValueSignal = pyqtSignal(str)
    zValueSignal = pyqtSignal(str)

    def __init__(self, platform_model, parent=None):
        super(StewartPlatformWidget, self).__init__(parent)
        self.platform_model = platform_model
        self.home_translation = platform_model.home_translation
        self.home_rotation = platform_model.home_rotation
        self.translation = self.home_translation.copy()
        self.rotation = self.home_rotation.copy()
        self.state = platform_model.calc_state(
            self.translation, self.rotation)
        self.broken_legs = []

    def refresh(self):
        self.emitPlatformPosition()
        state = self.platform_model.calc_state(
            self.translation, self.rotation)

        self.broken_legs = []
        for i, angle in enumerate(state.angles):
            if angle is None:
                self.broken_legs.append(i)

        if not self.broken_legs:
            self.state = state
        self.update()

    @staticmethod
    def _toRadian(degree):
        return degree / 180.0 * np.pi

    @staticmethod
    def _toDegree(radians):
        return radians / np.pi * 180

    def emitPlatformPosition(self):
        self.rollValueSignal.emit('%.1f' % self._toDegree(self.rotation[0]))
        self.pitchValueSignal.emit('%.1f' % self._toDegree(self.rotation[1]))
        self.yawValueSignal.emit('%.1f' % self._toDegree(self.rotation[2]))
        self.xValueSignal.emit('%.1f cm' % (self.translation[0] * 100.0))
        self.yValueSignal.emit('%.1f cm' % (self.translation[1] * 100.0))
        self.zValueSignal.emit('%.1f cm' % (self.translation[2] * 100.0))

    def setRoll(self, value):
        self.rotation[0] = self.home_rotation[0] + self._toRadian(value)
        self.refresh()

    def setPitch(self, value):
        self.rotation[1] = self.home_rotation[1] + self._toRadian(value)
        self.refresh()

    def setYaw(self, value):
        self.rotation[2] = self.home_rotation[2] + self._toRadian(value)
        self.refresh()

    def setX(self, value):
        self.translation[0] = self.home_translation[0] + value * 0.015
        self.refresh()

    def setY(self, value):
        self.translation[1] = self.home_translation[1] + value * 0.015
        self.refresh()

    def setZ(self, value):
        self.translation[2] = self.home_translation[2] + value * 0.01
        self.refresh()

    def drawCylinder(self, quadric, x, y, r):
        dx = y[0] - x[0]
        dy = y[1] - x[1]
        dz = y[2] - x[2]

        d = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5

        cosx = dz / d

        angle  = (np.arccos(cosx) / np.pi) * 180

        gl.glTranslatef(x[0], x[1], x[2])
        glu.gluSphere(quadric, r, 30, 30)
        gl.glRotatef(angle, -dy, dx, 0)
        glu.gluCylinder(quadric, r, r, d, 30, 10)
        gl.glRotatef(-angle, -dy, dx, 0)
        gl.glTranslatef(-x[0], -x[1], -x[2])

        gl.glTranslatef(y[0], y[1], y[2])
        glu.gluSphere(quadric, r, 30, 30)
        gl.glTranslatef(-y[0], -y[1], -y[2])
        

    def draw(self):
        gl.glColor4d(1.0, 0.8, 0.8, 1.0)
        gl.glBegin(gl.GL_POLYGON)
        base_r, base_center = self.state.base
        engine_r = 0.05
        base_center = base_center.copy()
        base_center[2] -= engine_r

        for phi in np.linspace(0, np.pi * 2.0, 100):
            p = base_center + base_r * np.array((np.cos(phi), np.sin(phi), 0.0))
            gl.glVertex3dv(p)
        gl.glEnd()

        gl.glColor4d(0.8, 1.0, 0.8, 1.0)
        gl.glBegin(gl.GL_POLYGON)
        for p in self.state.platform:
            gl.glVertex3dv(p)
        gl.glEnd()

        q = glu.gluNewQuadric()

        # todo(savegor): we need to setup the servos in the
        # platform config instead of this imitation
        for index, servo in enumerate(self.state.servos):
            broken = index in self.broken_legs

            if broken:
                gl.glColor4d(1.0, 0.0, 0.0, 1.0)
            else:
                gl.glColor4d(0.3, 0.3, 0.3, 1.0)
        
            start = base_center + [0, 0, engine_r]
            end = start + 0.9 * servo 
            start += 0.5 * servo
            self.drawCylinder(q, start, end, engine_r)

        for leg in self.state.legs:
            a, b, c, d = leg

            gl.glColor4d(0.1, 0.1, 0.8, 1.0)
            self.drawCylinder(q, a, b, 0.02)
            
            gl.glColor4d(0.1, 0.8, 0.1, 1.0)
            self.drawCylinder(q, b, c, 0.02)
            
            gl.glColor4d(0.4, 0.4, 0.8, 1.0)
            self.drawCylinder(q, c, d, 0.02)
        
        glu.gluDeleteQuadric(q)

        for index, servo in enumerate(self.state.servos):
            raw_angle = self.state.angles[index]
            if raw_angle > np.pi * 0.5:
                raw_angle = np.pi - raw_angle
            angle = "%.1f" % (raw_angle / np.pi * 180.0)
            self.renderText(servo * 1.4, angle)

        # it is necessary to enable depth test because QPainter disables it internally!
        gl.glEnable(gl.GL_DEPTH_TEST)

    def renderText(self, textPosWorld, text):
        width = self.width
        height = self.height

        textPosX, textPosY = self.toScreen(textPosWorld)
        textPosY = height - textPosY

        painter = QPainter(self)
        painter.setPen(Qt.black)
        painter.setFont(QFont("Helvetica", 16))
        painter.setRenderHints(QPainter.Antialiasing | QPainter.TextAntialiasing)
        painter.drawText(textPosX, textPosY, text)
        painter.end()

    def toScreen(self, point):
        viewport = self.viewportMatrix
        projection = self.projectionMatrix.transpose()
        modelview =  self.modelViewMatrix.transpose()

        PM = np.dot(projection, modelview)

        p = np.array([point[0], point[1], point[2], 1.0])
        
        sp = np.dot(PM, p).transpose()
        
        x, y, z, w = sp
        x /= w; y /= w; z /= w

        
        x = x * 0.5 + 0.5
        y = y * 0.5 + 0.5
        z = z * 0.5 + 0.5
        x = x * viewport[2] + viewport[0]
        y = y * viewport[3] + viewport[1]
    
        return int(x), int(y)

        

if __name__ == '__main__':
    zerge_cfg = yaml.load(open('configs/platform_shuyak.yml', 'r'))
    zerge_model = StewartPlatform(zerge_cfg)
    app = QApplication(sys.argv)
    window = ApplicationWindow(zerge_model)
    window.resize(1020, 600)
    window.show()
    sys.exit(app.exec_())
