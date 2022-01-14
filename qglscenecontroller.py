import numpy as np


import OpenGL.GL as gl
import OpenGL.GLU as glu

from PyQt5.QtCore import pyqtSignal, QPoint, QSize, Qt
from PyQt5.QtGui import QColor, QPainter, QFont
from PyQt5.QtWidgets import (QApplication, QHBoxLayout, QVBoxLayout,
    QOpenGLWidget, QSlider, QLabel, QWidget, QPushButton)


class QGLSceneController(QOpenGLWidget):

    def __init__(self, parent=None, bgColor=None):
        super(QGLSceneController, self).__init__(parent)

        self.width = None
        self.height = None
        self.backgroundColor = bgColor or QColor(210, 210, 220)
        
        # 3D scene control by mouse
        self.lastPos = QPoint()
        self.center = np.zeros(3, np.float32)
        self.shift = np.zeros(3, np.float32)
        self.xRot = 0
        self.yRot = 0

        self.modelViewMatrix = None
        self.prevModelViewMatrix = None
        self.rotationMatrix = None
        self.rotationDeltaMatrix = None

    def minimumSizeHint(self):
        return QSize(60, 40)

    def sizeHint(self):
        return QSize(800, 700)

    def calcMatrixRotation(self):
        gl.glPushMatrix()
        gl.glLoadIdentity()
        gl.glLoadMatrixf(self.rotationDeltaMatrix)
        gl.glRotatef(
            self.xRot,
            self.rotationMatrix[0][0],
            self.rotationMatrix[1][0],
            self.rotationMatrix[2][0])
        gl.glRotatef(
            self.yRot,
            self.rotationMatrix[0][1],
            self.rotationMatrix[1][1],
            self.rotationMatrix[2][1])
        self.rotationDeltaMatrix = gl.glGetDouble(gl.GL_MODELVIEW_MATRIX)
        gl.glPopMatrix()

        gl.glPushMatrix()
        gl.glLoadMatrixf(self.rotationMatrix)

        gl.glRotatef(
            self.xRot,
            self.rotationMatrix[0][0],
            self.rotationMatrix[1][0],
            self.rotationMatrix[2][0])
        gl.glRotatef(
            self.yRot,
            self.rotationMatrix[0][1],
            self.rotationMatrix[1][1],
            self.rotationMatrix[2][1])

        self.xRot = 0
        self.yRot = 0
        
        self.rotationMatrix = gl.glGetDouble(gl.GL_MODELVIEW_MATRIX)

        gl.glPopMatrix()

    def _enableLighting(self):
        gl.glEnable(gl.GL_LIGHTING)
        gl.glEnable(gl.GL_LIGHT0)
        gl.glEnable(gl.GL_COLOR_MATERIAL)
        gl.glEnable(gl.GL_NORMALIZE)
        gl.glMaterialfv(gl.GL_FRONT_AND_BACK, gl.GL_SPECULAR, [0.9,0.7,0.7,1]);
        gl.glMaterialfv(gl.GL_FRONT_AND_BACK, gl.GL_SHININESS, 50.0)

    def initializeGL(self):
        gl.glClearColor(self.backgroundColor.redF(),
                        self.backgroundColor.greenF(),
                        self.backgroundColor.blueF(),
                        self.backgroundColor.alphaF())
        gl.glEnable(gl.GL_DEPTH_TEST)

        self._enableLighting()

        gl.glMatrixMode(gl.GL_MODELVIEW)
        self.rotationMatrix = gl.glGetDouble(gl.GL_MODELVIEW_MATRIX)
        self.rotationDeltaMatrix = gl.glGetDouble(gl.GL_MODELVIEW_MATRIX)

    def _modelviewTranform(self):
        if self.prevModelViewMatrix is not None:
            gl.glTranslatef(self.shift[0], self.shift[1], self.shift[2])
            gl.glMultMatrixf(self.prevModelViewMatrix)
        else:
            gl.glTranslatef(0, 0, -3.0)

        gl.glTranslatef(self.center[0], self.center[1], self.center[2])
        gl.glMultMatrixf(self.rotationMatrix)
        gl.glTranslatef(-self.center[0], -self.center[1], -self.center[2])

        self.modelViewMatrix = gl.glGetDouble(gl.GL_MODELVIEW_MATRIX)
        if self.prevModelViewMatrix is None:
            self.prevModelViewMatrix = self.modelViewMatrix

    def resizeGL(self, width, height):
        if height == 0: height = 1

        self.width = width
        self.height = height

        gl.glViewport(0, 0, width, height)
        
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        aspect = width / float(height)
        glu.gluPerspective(45.0, aspect, 0.2, 40000.0)

        self.viewportMatrix = gl.glGetIntegerv(gl.GL_VIEWPORT)
        self.projectionMatrix = gl.glGetDoublev(gl.GL_PROJECTION_MATRIX)

        gl.glMatrixMode(gl.GL_MODELVIEW)

    def paintGL(self):
        gl.glClear(
            gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)

        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glLoadIdentity()

        self.calcMatrixRotation()
        self._modelviewTranform()

        self.draw()

    def _calcSelectLine(self, screenX, screenY):
        vx = screenX
        vy = self.height - screenY - 1

        screenNearWorld = np.array(glu.gluUnProject(vx, vy, -1,
            self.modelViewMatrix, self.projectionMatrix, self.viewportMatrix))
        screenFarWorld = np.array(glu.gluUnProject(vx, vy, 1,
            self.modelViewMatrix, self.projectionMatrix, self.viewportMatrix))

        return screenNearWorld, screenFarWorld

    def _normalizeAngle(self, angle):
        angle *= 0.3
        while angle < 0:
            angle += 360 * 2
        while angle > 360 * 2:
            angle -= 360 * 2
        return angle

    def _lineAsTwoPlanes(self, p1, p2):
        v = p2 - p1

        if np.linalg.norm(v) == 0.0:
            raise RuntimeError('Points must be different!')

        n1 = np.array([1.0, 0.0, 0.0])

        if np.abs(np.cross(n1, v)).sum() == 0.0:
            n1 = np.array([0.0, 1.0, 0.0])

        if np.dot(n1, v) != 0.0:
            n1 = np.cross(n1, v)

        n2 = np.cross(n1, v)

        d1 = np.dot(n1, p1)
        d2 = np.dot(n2, p1)

        return n1, d1, n2, d2

    def _projectionToCentralPlane(self, screenX, screenY):
        n1 = self.rotationMatrix[:,2][:3]
        d1 = np.dot(n1, self.center)

        t1, t2 = self._calcSelectLine(screenX, screenY)
        
        n2, d2, n3, d3 = self._lineAsTwoPlanes(t1, t2)
        
        matrix = np.array([n1, n2, n3])
        free = np.array([d1, d2, d3])

        if np.linalg.det(matrix) == 0.0:
            raise RuntimeError(
                'Unexpected state: determinant should always be non zero!')

        return np.linalg.solve(matrix, free)


    def wheelEvent(self, event):     
        screenPoint, _ = self._calcSelectLine(event.x(), event.y())

        shiftDirection = self.center - screenPoint

        delta = np.sign(event.angleDelta().y())
        invModelview = np.linalg.inv(self.modelViewMatrix)

        shiftLocal = delta * shiftDirection * 0.1
        shiftWorld = np.dot(invModelview[:3,:3], shiftLocal)

        if delta < 0:
            if np.dot(shiftWorld, shiftWorld) < 2.9e8:
                self.shift -= shiftWorld
        else:
            while np.dot(self.center - screenPoint,
                         self.center + shiftLocal - screenPoint) < 0:
                shiftLocal *= 0.99
                shiftWorld *= 0.99   
            if np.linalg.norm(shiftWorld) > 0.05:
                self.shift -= shiftWorld

        self.update()

    def mousePressEvent(self, event):
        self.lastPos = event.pos()

    def mouseMoveEvent(self, event):
        dx = event.x() - self.lastPos.x()
        dy = event.y() - self.lastPos.y()

        if event.buttons() & Qt.LeftButton:
            self.yRot = self._normalizeAngle(dx)
            self.xRot = self._normalizeAngle(dy)
        elif event.buttons() & Qt.RightButton:
            prevProjPoint = self._projectionToCentralPlane(
                self.lastPos.x(), self.lastPos.y())
            curProjPoint = self._projectionToCentralPlane(
                event.x(), event.y())
            
            projShift = curProjPoint - prevProjPoint
        
            invModelViewMatrix = np.linalg.inv(self.modelViewMatrix)
            projShiftWorld = np.dot(invModelViewMatrix[:3,:3], projShift)
            self.shift += projShiftWorld

        self.lastPos = QPoint(event.pos())
        self.update()
