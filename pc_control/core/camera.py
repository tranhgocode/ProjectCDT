from core.object3D import Object3D
from core.matrix import Matrix
from numpy.linalg import inv
import numpy as np

class Camera(Object3D):
    def __init__(self, angleOfView=60, aspectRatio=1, near=0.1, far=1000):
        super().__init__()
        self.projectionMatrix = Matrix.makePerspective(angleOfView, aspectRatio, near, far)
        self.viewMatrix = Matrix.makeIdentity()

    def updateViewMatrix(self):
        self.viewMatrix = inv(self.getWorldMatrix())
    
    def lookAt(self, target, up=np.array([0, 1, 0])):
        """Set camera to look at target point with up direction - set view matrix directly."""
        pos = self.getPosition()
        forward = target - pos
        forward = forward / np.linalg.norm(forward)
        
        right = np.cross(forward, up)
        right = right / np.linalg.norm(right)
        
        new_up = np.cross(right, forward)
        
        # Build view matrix directly
        view_matrix = np.eye(4)
        view_matrix[0, :3] = right
        view_matrix[1, :3] = new_up
        view_matrix[2, :3] = -forward
        view_matrix[:3, 3] = -pos
        
        self.viewMatrix = view_matrix
