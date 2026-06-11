import math
from core.mesh import Mesh
from graphics.geometry import BoxGeometry
from graphics.surfaceMaterial import SurfaceMaterial
from core.object3D import Object3D

class GreenTarget(Object3D):
    def __init__(self, size=0.2):
        super().__init__()
        geometry = BoxGeometry(width=size, height=size, depth=size)
        material = SurfaceMaterial({"color": [0.0, 0.8, 0.1]}) # Xanh lá hiện đại
        
        self.mesh = Mesh(geometry, material)
        self.add(self.mesh)
        
    def update_position(self, position):
        """Cập nhật vị trí của khối hộp (truyền vào [x, y, z])"""
        # Lưu ý: Nếu muốn vật nằm TRÊN mặt đất (không bị chìm nửa khối)
        # ta cộng thêm một nửa kích thước vào trục Y
        half_size = 0.1 # Vì size mặc định là 0.2
        self.setPosition([position[0], position[1] + half_size, position[2]])

def create_target_cube(scene, position, size=0.2):
    """Hàm tiện ích để tạo nhanh target và add vào scene"""
    target = GreenTarget(size)
    target.update_position(position)
    scene.add(target)
    return target
