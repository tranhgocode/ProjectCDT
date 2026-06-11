"""Drawing utilities for grid and target points in a Z-up coordinate system."""
from OpenGL.GL import *


def draw_grid(size=20, divisions=20):
    """Vẽ lưới tọa độ trên mặt phẳng XZ (mặt đất, Y=0).
    
    Args:
        size: Kích thước lưới
        divisions: Số ô lưới
    """
    glBegin(GL_LINES)
    glColor3f(0.4, 0.4, 0.4)
    step = size / divisions
    for i in range(-divisions, divisions + 1):
        # Vẽ các đường thẳng song song trục Z
        glVertex3f(i * step, 0, -size)
        glVertex3f(i * step, 0, size)
        # Vẽ các đường thẳng song song trục X
        glVertex3f(-size, 0, i * step)
        glVertex3f(size, 0, i * step)
    glEnd()


def draw_target_point(pos, size=10):
    """Vẽ điểm mục tiêu end-effector (target point).
    
    Args:
        pos: Position [x, y, z]
        size: Point size
    """
    glDisable(GL_LIGHTING)
    glPointSize(size)
    glColor3f(1.0, 0.0, 0.0)
    glBegin(GL_POINTS)
    glVertex3f(pos[0], pos[1], pos[2])
    glEnd()
    glEnable(GL_LIGHTING)


def draw_axis(length=1.0):
    """Vẽ trục XYZ tại gốc tọa độ (optional). Y là hướng lên trên."""
    glDisable(GL_LIGHTING)
    glLineWidth(2.0)
    
    glBegin(GL_LINES)
    # Trục X - Đỏ
    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(0, 0, 0)
    glVertex3f(length, 0, 0)
    
    # Trục Y - Xanh lá
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(0, 0, 0)
    glVertex3f(0, length, 0)
    
    # Trục Z - Xanh dương
    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(0, 0, 0)
    glVertex3f(0, 0, length)
    glEnd()
    
    glLineWidth(1.0)
    glEnable(GL_LIGHTING)
