import numpy as np
import math

class Matrix:
    @staticmethod
    def makeIdentity():
        return np.identity(4, dtype=float)

    @staticmethod
    def makeTranslation(x, y, z):
        return np.array([[1, 0, 0, x],
                         [0, 1, 0, y],
                         [0, 0, 1, z],
                         [0, 0, 0, 1]], dtype=float)

    @staticmethod
    def makeRotationX(angle):
        c, s = math.cos(angle), math.sin(angle)
        return np.array([[1, 0, 0, 0],
                         [0, c, -s, 0],
                         [0, s,  c, 0],
                         [0, 0, 0, 1]], dtype=float)

    @staticmethod
    def makeRotationY(angle):
        c, s = math.cos(angle), math.sin(angle)
        return np.array([[ c, 0, s, 0],
                         [ 0, 1, 0, 0],
                         [-s, 0, c, 0],
                         [ 0, 0, 0, 1]], dtype=float)

    @staticmethod
    def makeRotationZ(angle):
        c, s = math.cos(angle), math.sin(angle)
        return np.array([[c, -s, 0, 0],
                         [s,  c, 0, 0],
                         [0,  0, 1, 0],
                         [0,  0, 0, 1]], dtype=float)

    @staticmethod
    def makePerspective(angleOfView=60, aspectRatio=1, near=0.1, far=1000):
        a = angleOfView * math.pi / 180.0
        d = 1.0 / math.tan(a / 2)
        r = aspectRatio
        b = (far + near) / (near - far)
        c = 2 * far * near / (near - far)
        return np.array([[d/r, 0, 0, 0],
                         [0,   d, 0, 0],
                         [0,   0, b, c],
                         [0,   0,-1, 0]], dtype=float)