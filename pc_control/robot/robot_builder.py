import math
from core.mesh import Mesh
from graphics.geometry import BoxGeometry, CylinderGeometry
from graphics.surfaceMaterial import SurfaceMaterial
from core.object3D import Object3D

def create_pitch_joint(radius=0.2, thickness=0.8, color=[0.2, 0.2, 0.2]):
    """Khớp gập dạng chốt xuyên qua tay, trục pin nằm ngang theo X."""
    joint = Object3D()
    cylinder = Mesh(CylinderGeometry(radiusTop=radius, radiusBottom=radius, height=thickness),
                    SurfaceMaterial({"color": color}))
    cylinder.setRotationY(math.pi / 2)
    joint.add(cylinder)
    return joint

def create_capsule_link(length, width, thickness, color=[0.75, 0.75, 0.75]):
    link = Object3D()
    radius = width / 2.0

    box = Mesh(BoxGeometry(width=width, height=thickness, depth=length),
               SurfaceMaterial({"color": color}))
    box.setPosition([0, 0, length / 2])
    link.add(box)

    cyl_bottom = Mesh(CylinderGeometry(radiusTop=radius, radiusBottom=radius, height=thickness),
                      SurfaceMaterial({"color": color}))
    cyl_bottom.setPosition([0, 0, 0])
    link.add(cyl_bottom)

    cyl_top = Mesh(CylinderGeometry(radiusTop=radius, radiusBottom=radius, height=thickness),
                   SurfaceMaterial({"color": color}))
    cyl_top.setPosition([0, 0, length])
    link.add(cyl_top)

    return link

def build_robot_4dof(scene):
    ARM_WIDTH = 0.5
    ARM_THICK = 0.25
    PIN_RADIUS = 0.2 
    JOINT_THICK = 0.8 
    
    L1, L2, L3 = 1.2, 1.2, 1.0

    base = Object3D()
    scene.add(base)
    
    # =========================================================
    base_height = 0.4
    post_height = 0.77
    pedestal = Mesh(CylinderGeometry(radiusTop=0.8, radiusBottom=0.3, height=base_height),
                    SurfaceMaterial({"color": [0.4, 0.4, 0.4]}))
    pedestal.setRotationX(math.pi / 2) # Dựng đứng
    pedestal.setPosition([0, base_height / 2, 0])
    base.add(pedestal)

    # --- KHỚP 1: YAW (Khớp xoay đế) ---
    joint1 = Object3D()
    joint1.setPosition([0, base_height, 0])

    yaw_disk = Mesh(CylinderGeometry(radiusTop=0.25, radiusBottom=0.3, height=0.1),
                    SurfaceMaterial({"color": [0.2, 0.2, 0.2]}))
    yaw_disk.setRotationX(math.pi / 2)
    yaw_disk.setPosition([0, 0.05, 0])
    joint1.add(yaw_disk)
    base.add(joint1)

    vertical_post = Mesh(BoxGeometry(width=0.4, height=post_height, depth=ARM_THICK), 
                         SurfaceMaterial({"color": [0.25, 0.25, 0.25]}))
    vertical_post.setPosition([0, 0.1 + post_height / 2, 0])
    joint1.add(vertical_post)

    # --- KHỚP 2: PITCH (Khớp vai) ---
    joint2 = create_pitch_joint(radius=PIN_RADIUS, thickness=JOINT_THICK)
    joint2.setPosition([0, 0.1 + post_height, 0])
    joint1.add(joint2)

    link1 = create_capsule_link(length=L1, width=ARM_WIDTH, thickness=ARM_THICK)
    joint2.add(link1)

    # --- KHỚP 3: PITCH (Khớp khuỷu tay) ---
    joint3 = create_pitch_joint(radius=PIN_RADIUS, thickness=JOINT_THICK)
    joint3.setPosition([0, 0, L1])
    joint2.add(joint3)

    link2 = create_capsule_link(length=L2, width=ARM_WIDTH, thickness=ARM_THICK)
    joint3.add(link2)

    # --- KHỚP 4: PITCH (Khớp cổ tay) ---
    joint4 = create_pitch_joint(radius=PIN_RADIUS * 0.8, thickness=JOINT_THICK)
    joint4.setPosition([0, 0, L2])
    joint3.add(joint4)

    # --- ĐẦU HÚT ---
    end_effector = Object3D()
    ee_box = Mesh(BoxGeometry(width=0.2, height=0.2, depth=L3),
                  SurfaceMaterial({"color": [0.15, 0.15, 0.15]}))
    ee_box.setPosition([0, 0, L3 / 2])
    end_effector.add(ee_box)

    ee_cup = Mesh(CylinderGeometry(radiusTop=0.15, radiusBottom=0.05, height=0.1),
                  SurfaceMaterial({"color": [0.8, 0.1, 0.1]}))
    ee_cup.setPosition([0, 0, L3 + 0.05])
    end_effector.add(ee_cup)

    joint4.add(end_effector)

    return {
        "joint1": joint1,
        "joint2": joint2,
        "joint3": joint3,
        "joint4": joint4,
    }

def apply_joint_rotations(joints, angles):
    # Dùng trực tiếp giá trị angles truyền vào
    joints["joint1"].setRotationY(angles[0]) 
    joints["joint2"].setRotationX(-angles[1])
    joints["joint3"].setRotationX(-angles[2])
    joints["joint4"].setRotationX(-angles[3])
