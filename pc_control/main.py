import math
import random
import numpy as np
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
import multiprocessing as mp

from core.scene import Scene
from graphics.renderer import Renderer
from core.camera import Camera
from robot.kinematics import (
    GROUND_TARGET_MAX_RADIUS,
    check_ground_collision,
    inverse_kinematics_optimized,
)
from robot.robot_builder import build_robot_4dof, apply_joint_rotations
from graphics.drawing_utils import draw_grid
from robot.target_object import create_target_cube


SAFE_TARGET_RADIUS = GROUND_TARGET_MAX_RADIUS - 0.02
TWO_PI = 2 * math.pi
REAL_ROBOT_HOME_DEG = np.array([0.0, 0.86, -14.76], dtype=float)
WRIST_HOME_DEG = 0.0
MOTOR_TO_SIM_SIGNS = np.array([-1.0, 1.0, 1.0], dtype=float)


def normalize_yaw(angle):
    return angle % TWO_PI


def shortest_yaw_delta(current, target):
    return (target - current + math.pi) % TWO_PI - math.pi


def motor_angles_to_sim_radians(motor_angles_deg):
    motor_angles_deg = np.asarray(motor_angles_deg, dtype=float)
    if motor_angles_deg.shape != (3,):
        raise ValueError("Expected [motor1, motor2, motor3] angles in degrees")

    yaw_deg, shoulder_deg, elbow_deg = motor_angles_deg * MOTOR_TO_SIM_SIGNS
    q = np.radians([yaw_deg, shoulder_deg, elbow_deg, WRIST_HOME_DEG])
    q[0] = normalize_yaw(q[0])
    return q


def clamp_ground_target(target):
    target = np.array(target, dtype=float)
    target[1] = 0.0
    radius = math.hypot(target[0], target[2])

    if radius > SAFE_TARGET_RADIUS:
        scale = SAFE_TARGET_RADIUS / radius
        target[0] *= scale
        target[2] *= scale

    return target


def choose_random_target(radius=SAFE_TARGET_RADIUS, height=0.0):
    radius = min(radius, SAFE_TARGET_RADIUS)
    angle = random.random() * 2 * math.pi
    r = math.sqrt(random.random()) * radius
    x = r * math.cos(angle)
    z = r * math.sin(angle)
    return np.array([x, height, z], dtype=float)


def compute_joint_positions(angles):
    q1, q2, q3, q4 = angles

    base_pos = np.array([0.0, 1.27, 0.0])
    L1, L2, L3 = 1.2, 1.2, 1.0

    r1 = L1 * math.cos(q2)
    y1 = L1 * math.sin(q2)
    p1 = base_pos + np.array([r1 * math.sin(q1), y1, r1 * math.cos(q1)])

    r2 = L2 * math.cos(q2 + q3)
    y2 = L2 * math.sin(q2 + q3)
    p2 = p1 + np.array([r2 * math.sin(q1), y2, r2 * math.cos(q1)])

    r3 = L3 * math.cos(q2 + q3 + q4)
    y3 = L3 * math.sin(q2 + q3 + q4)
    p3 = p2 + np.array([r3 * math.sin(q1), y3, r3 * math.cos(q1)])

    return [base_pos, p1, p2, p3]


def print_joint_status(q, label="Current"):
    degrees = np.degrees(q)
    degrees[0] = math.degrees(normalize_yaw(q[0]))
    positions = compute_joint_positions(q)
    print(f"\n[{label}] Góc các khớp (độ): {[round(x, 2) for x in degrees]}")
    for idx, pos in enumerate(positions):
        name = "Base" if idx == 0 else f"Joint {idx}"
        print(f"  {name:7}: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")


def run_3d_simulation(target_queue):
    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, pygame.DOUBLEBUF | pygame.OPENGL)
    pygame.display.set_caption("3D Robot Simulation - Optimized Movement")

    scene = Scene()
    renderer = Renderer()
    camera = Camera(angleOfView=50, aspectRatio=display[0] / display[1], near=0.1, far=100.0)
    camera.setPosition([0.0, 4.0, 12.0])  # Camera phía trước, cao hơn, nhìn vào robot
    camera.lookAt(np.array([0.0, 1.0, 0.0]), up=np.array([0, 1, 0]))

    joints = build_robot_4dof(scene)
    curr_q = motor_angles_to_sim_radians(REAL_ROBOT_HOME_DEG)
    apply_joint_rotations(joints, curr_q.tolist())
    print_joint_status(curr_q, label="Home")
    target_pos = choose_random_target(height=0.0)  # Gắp vật trên mặt đất
    target_cube = create_target_cube(scene, target_pos, size=0.2)
    dest_q = curr_q.copy()

    # State machine cho chuyển động
    STATE_IDLE = 0
    STATE_ROTATING_BASE = 1
    STATE_MOVING_ARM = 2
    state = STATE_IDLE
    
    step = 0.05  # Tốc độ di chuyển
    clock = pygame.time.Clock()

    while True:
        # Lấy mục tiêu mới từ màn hình 2D
        if not target_queue.empty():
            target_pos = clamp_ground_target(target_queue.get())
            res = inverse_kinematics_optimized(target_pos, curr_q)
            if res is not None:
                dest_q = np.array(res, dtype=float)
                dest_q[0] = normalize_yaw(dest_q[0])
                
                # **KIỂM TRA: Đảm bảo không có joint nào xuyên qua mặt đất, end-effector tiếp xúc mặt đất**
                safe, heights = check_ground_collision(dest_q)
                if safe:
                    target_cube.update_position(target_pos)
                    print(f"\n✓ Target position: x={target_pos[0]:.3f}, y={target_pos[1]:.3f}, z={target_pos[2]:.3f}")
                    print(f" ✓ End-effector touching ground (y={heights['p3']:.4f})")
                    print(f" Joint heights - p1: {heights['p1']:.3f}, p2: {heights['p2']:.3f}, p3: {heights['p3']:.3f}")
                    state = STATE_ROTATING_BASE
                else:
                    print(f"\n ⚠️ WARNING: Invalid configuration!")
                    print(f" Joint heights - p1: {heights['p1']:.3f}, p2: {heights['p2']:.3f}, p3: {heights['p3']:.3f}")
                    if heights['p3'] > 0.05:
                        print(f" ✗ End-effector not touching ground (y={heights['p3']:.3f}, should be ≈ 0)")
                    if heights['p1'] < 0 or heights['p2'] < 0:
                        print(f" ✗ Some joints below ground")
                    print(f" Keeping current position")
            else:
                print("Error: Failed to compute inverse kinematics")

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return
        if state == STATE_ROTATING_BASE:
            delta = shortest_yaw_delta(curr_q[0], dest_q[0])
            if abs(delta) > 0.01:
                curr_q[0] = normalize_yaw(curr_q[0] + np.sign(delta) * min(step, abs(delta)))
                apply_joint_rotations(joints, curr_q.tolist())
            else:
                curr_q[0] = normalize_yaw(dest_q[0])
                state = STATE_MOVING_ARM

        elif state == STATE_MOVING_ARM:
            reached = True
            for i in [1, 2, 3]:
                delta = dest_q[i] - curr_q[i]
                if abs(delta) > 0.01:
                    curr_q[i] += np.sign(delta) * min(step, abs(delta))
                    reached = False
                else:
                    curr_q[i] = dest_q[i]

            apply_joint_rotations(joints, curr_q.tolist())
            if reached:
                state = STATE_IDLE
                print_joint_status(curr_q, label="Đã tới mục tiêu")

        # OpenGL render
        glClearColor(0.12, 0.12, 0.14, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_PROJECTION); glLoadMatrixf(camera.projectionMatrix.T)
        glMatrixMode(GL_MODELVIEW); camera.updateViewMatrix(); glLoadMatrixf(camera.viewMatrix.T)
        glBegin(GL_QUADS)
        glColor3f(0.2, 0.2, 0.2)
        glVertex3f(-5, 0, -5)
        glVertex3f(5, 0, -5)
        glVertex3f(5, 0, 5)
        glVertex3f(-5, 0, 5)
        glEnd()

        draw_grid(size=10, divisions=20)
        renderer.render(scene, camera)

        pygame.display.flip()
        clock.tick(60)


def create_2d_icon(size=80):
    icon = pygame.Surface((size, size), pygame.SRCALPHA)
    icon.fill((0, 0, 0, 0))
    pygame.draw.rect(icon, (180, 120, 40), (10, 20, 60, 40), border_radius=10)
    pygame.draw.rect(icon, (210, 170, 90), (14, 24, 52, 32), border_radius=8)
    pygame.draw.line(icon, (120, 70, 20), (10, 34), (70, 34), 4)
    pygame.draw.line(icon, (120, 70, 20), (28, 20), (28, 60), 4)
    pygame.draw.line(icon, (120, 70, 20), (48, 20), (48, 60), 4)
    pygame.draw.circle(icon, (255, 255, 255), (25, 30), 3)
    pygame.draw.circle(icon, (255, 255, 255), (45, 30), 3)
    return icon


def run_2d_interface(target_queue):
    pygame.init()
    display = (600, 600)
    screen = pygame.display.set_mode(display)
    pygame.display.set_caption("2D Control Interface - Click to Set Target")

    font_large = pygame.font.SysFont(None, 30)
    font_small = pygame.font.SysFont(None, 22)
    icon = create_2d_icon(72)

    scale = 50
    offset_x = display[0] // 2
    offset_y = display[1] // 2
    target_pos = np.array([0.0, 0.0, 0.0], dtype=float)
    target_text = "Click on the grid to place the ground target"

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    mouse_x, mouse_y = event.pos
                    world_x = (mouse_x - offset_x) / scale
                    world_z = (offset_y - mouse_y) / scale
                    target_pos = clamp_ground_target([world_x, 0.0, world_z])
                    target_queue.put(target_pos.copy())
                    target_text = f"Target set to ({target_pos[0]:.2f}, 0.00, {target_pos[2]:.2f})"

        screen.fill((40, 42, 54))

        # Góc 1: X>0, Z>0 - xanh lá
        pygame.draw.rect(screen, (0, 128, 0), (offset_x, 0, display[0] - offset_x, offset_y))
        # Góc 2: X<0, Z>0 - xanh dương
        pygame.draw.rect(screen, (0, 0, 128), (0, 0, offset_x, offset_y))
        # Góc 3: X<0, Z<0 - vàng
        pygame.draw.rect(screen, (128, 128, 0), (0, offset_y, offset_x, display[1] - offset_y))
        # Góc 4: X>0, Z<0 - đỏ
        pygame.draw.rect(screen, (128, 0, 0), (offset_x, offset_y, display[0] - offset_x, display[1] - offset_y))

        # Header image panel with icon and instructions
        panel_rect = pygame.Rect(10, 10, display[0] - 20, 110)
        pygame.draw.rect(screen, (30, 32, 44), panel_rect, border_radius=12)
        pygame.draw.rect(screen, (80, 100, 150), panel_rect, 2, border_radius=12)
        screen.blit(icon, (24, 24))
        title = font_large.render("2D Target Selector", True, (240, 240, 240))
        screen.blit(title, (120, 24))
        hint = font_small.render("Click anywhere on the ground grid to place the object target.", True, (200, 200, 210))
        screen.blit(hint, (120, 54))
        status = font_small.render(target_text, True, (180, 220, 180))
        screen.blit(status, (120, 82))

        for i in range(-10, 11):
            y_line = offset_y + i * scale
            x_line = offset_x + i * scale
            pygame.draw.line(screen, (70, 70, 90), (0, y_line), (display[0], y_line), 1)
            pygame.draw.line(screen, (70, 70, 90), (x_line, 0), (x_line, display[1]), 1)

        pygame.draw.line(screen, (255, 100, 100), (0, offset_y), (display[0], offset_y), 2)
        pygame.draw.line(screen, (100, 180, 255), (offset_x, 0), (offset_x, display[1]), 2)
        pygame.draw.circle(screen, (0, 200, 120), (offset_x, offset_y), 8)

        target_screen_x = int(offset_x + target_pos[0] * scale)
        target_screen_y = int(offset_y - target_pos[2] * scale)
        pygame.draw.circle(screen, (255, 180, 0), (target_screen_x, target_screen_y), 10)
        pygame.draw.circle(screen, (255, 255, 255), (target_screen_x, target_screen_y), 4)
        pygame.draw.circle(screen, (255, 255, 0), (offset_x, offset_y), int(SAFE_TARGET_RADIUS * scale), 2)

        pygame.display.flip()
        pygame.time.wait(40)
    pygame.quit()


def main():
    target_queue = mp.Queue()
    
    p3d = mp.Process(target=run_3d_simulation, args=(target_queue,))
    p2d = mp.Process(target=run_2d_interface, args=(target_queue,))
    
    p3d.start()
    p2d.start()
    
    p3d.join()
    p2d.join()


if __name__ == "__main__":
    main()
