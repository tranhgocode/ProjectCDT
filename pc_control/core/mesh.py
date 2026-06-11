from core.object3D import Object3D
class Mesh(Object3D):
    def __init__(self, geometry, material):
        super().__init__()
        self.geometry = geometry
        self.material = material
