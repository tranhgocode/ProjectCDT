from OpenGL.GL import *
from OpenGL.GLU import *
from core.mesh import Mesh
from graphics.geometry import BoxGeometry, SphereGeometry, CylinderGeometry, ConeGeometry

class Renderer:
    def __init__(self):
        # Nền đen + đổ bóng cơ bản
        glEnable(GL_DEPTH_TEST)
        glClearColor(0.0, 0.0, 0.0, 1.0)

        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glShadeModel(GL_SMOOTH)

        glLightfv(GL_LIGHT0, GL_POSITION, [4.0, 8.0, 10.0, 1.0])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.1, 0.1, 0.1, 1.0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.75, 0.75, 0.75, 1.0])
        glLightfv(GL_LIGHT0, GL_SPECULAR, [0.9, 0.9, 0.9, 1.0])

    def render(self, scene, camera):
        # DON'T clear here - let caller handle it
        # camera.updateViewMatrix()  # Commented out - view matrix set directly in lookAt

        # Set up matrices
        glMatrixMode(GL_PROJECTION)
        glLoadMatrixf(camera.projectionMatrix.T)
        glMatrixMode(GL_MODELVIEW)

        # Get all meshes and render
        descendants = self.getDescendants(scene)
        meshList = [x for x in descendants if isinstance(x, Mesh)]

        for mesh in meshList:
            mvMatrix = camera.viewMatrix @ mesh.getWorldMatrix()
            glLoadMatrixf(mvMatrix.T)

            glColor3f(*mesh.material.color)

            if isinstance(mesh.geometry, BoxGeometry):
                self.drawBox(mesh.geometry.width, mesh.geometry.height, mesh.geometry.depth)
            elif isinstance(mesh.geometry, SphereGeometry):
                self.drawSphere(mesh.geometry.radius, mesh.geometry.slices, mesh.geometry.stacks)
            elif isinstance(mesh.geometry, CylinderGeometry):
                self.drawCylinder(mesh.geometry.radiusTop, mesh.geometry.radiusBottom, 
                                 mesh.geometry.height, mesh.geometry.radialSegments)
            elif isinstance(mesh.geometry, ConeGeometry):
                self.drawCone(mesh.geometry.radius, mesh.geometry.height, mesh.geometry.radialSegments)

    def getDescendants(self, obj):
        res = []
        for child in obj.children:
            res.append(child)
            res += self.getDescendants(child)
        return res

    def drawGrid(self, size=10, step=1):
        glDisable(GL_LIGHTING)
        glColor3f(0.25, 0.25, 0.25)
        glBegin(GL_LINES)
        for i in range(-size, size + 1, step):
            glVertex3f(i, 0, -size)
            glVertex3f(i, 0, size)
            glVertex3f(-size, 0, i)
            glVertex3f(size, 0, i)
        glEnd()
        glEnable(GL_LIGHTING)

    def drawBox(self, w, h, d):
        w, h, d = w/2, h/2, d/2
        vertices = [
            [w, h, -d], [-w, h, -d], [-w, h, d], [w, h, d],
            [w, -h, d], [-w, -h, d], [-w, -h, -d], [w, -h, -d]
        ]
        faces = [
            (0,1,2,3), (3,2,5,4), (4,5,6,7),
            (7,6,1,0), (7,0,3,4), (5,2,1,6)
        ]
        normals = [
            (0, 1, 0),   # Top
            (0, 0, 1),   # Front
            (0, -1, 0),  # Bottom
            (0, 0, -1),  # Back
            (-1, 0, 0),  # Left
            (1, 0, 0),   # Right
        ]

        glBegin(GL_QUADS)
        for normal, face in zip(normals, faces):
            glNormal3fv(normal)
            for vertex in face:
                glVertex3fv(vertices[vertex])
        glEnd()

    def drawSphere(self, radius, slices, stacks):
        quad = gluNewQuadric()
        gluQuadricNormals(quad, GLU_SMOOTH)
        gluSphere(quad, radius, slices, stacks)
        gluDeleteQuadric(quad)

    def drawCylinder(self, radiusTop, radiusBottom, height, radialSegments):
        """Vẽ hình trụ (cylinder)"""
        quad = gluNewQuadric()
        gluQuadricNormals(quad, GLU_SMOOTH)

        glPushMatrix()
        glTranslatef(0, 0, -height/2)
        gluCylinder(quad, radiusBottom, radiusTop, height, radialSegments, 16)
        glPopMatrix()

        gluDeleteQuadric(quad)

    def drawCone(self, radius, height, radialSegments):
        """Vẽ hình nón (cone)"""
        quad = gluNewQuadric()
        gluQuadricNormals(quad, GLU_SMOOTH)

        glPushMatrix()
        glTranslatef(0, 0, -height/2)
        gluCylinder(quad, radius, 0, height, radialSegments, 16)
        glPopMatrix()

        gluDeleteQuadric(quad)
