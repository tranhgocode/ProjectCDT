class BoxGeometry:
    def __init__(self, width=1, height=1, depth=1):
        self.width = width
        self.height = height
        self.depth = depth


class SphereGeometry:
    def __init__(self, radius=1.0, slices=16, stacks=16):
        self.radius = radius
        self.slices = slices
        self.stacks = stacks


class CylinderGeometry:
    """Hình trụ (cylinder) - dùng cho các link/rod của robot"""
    def __init__(self, radiusTop=1.0, radiusBottom=1.0, height=2.0, radialSegments=16, heightSegments=1):
        self.radiusTop = radiusTop
        self.radiusBottom = radiusBottom
        self.height = height
        self.radialSegments = radialSegments
        self.heightSegments = heightSegments


class ConeGeometry:
    """Hình nón (cone) - dùng cho base/pyramid"""
    def __init__(self, radius=1.0, height=2.0, radialSegments=16, heightSegments=1):
        self.radius = radius
        self.height = height
        self.radialSegments = radialSegments
        self.heightSegments = heightSegments