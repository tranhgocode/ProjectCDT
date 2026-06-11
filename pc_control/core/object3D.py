import numpy as np
from core.matrix import Matrix

class Object3D:
    def __init__(self):
        self.transform = Matrix.makeIdentity()
        self.parent = None
        self.children = []

    def add(self, child):
        self.children.append(child)
        child.parent = self

    def remove(self, child):
        self.children.remove(child)
        child.parent = None

    def getWorldMatrix(self):
        if self.parent is None:
            return self.transform
        else:
            return self.parent.getWorldMatrix() @ self.transform

    def setPosition(self, position):
        self.transform[0, 3] = position[0]
        self.transform[1, 3] = position[1]
        self.transform[2, 3] = position[2]

    def getPosition(self):
        return np.array([self.transform[0, 3], self.transform[1, 3], self.transform[2, 3]])

    def setRotationX(self, angle):
        pos = [self.transform[0,3], self.transform[1,3], self.transform[2,3]]
        self.transform = Matrix.makeRotationX(angle)
        self.setPosition(pos)

    def setRotationY(self, angle):
        pos = [self.transform[0,3], self.transform[1,3], self.transform[2,3]]
        self.transform = Matrix.makeRotationY(angle)
        self.setPosition(pos)

    def setRotationZ(self, angle):
        pos = [self.transform[0,3], self.transform[1,3], self.transform[2,3]]
        self.transform = Matrix.makeRotationZ(angle)
        self.setPosition(pos)
