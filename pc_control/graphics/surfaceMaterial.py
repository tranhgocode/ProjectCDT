class SurfaceMaterial:
    def __init__(self, properties):
        # properties là một từ điển chứa màu sắc, ví dụ {"color": [1, 0, 0]}
        self.color = properties.get("color", [1, 1, 1])