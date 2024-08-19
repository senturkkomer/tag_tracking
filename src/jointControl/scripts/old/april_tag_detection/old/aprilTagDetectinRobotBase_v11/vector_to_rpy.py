import numpy as np
import math

class Transformation:
    
    @staticmethod
    def rotation_matrix_from_vectors(vec1, vec2):
        a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)

        v = np.cross(a, b)
        c = np.dot(a, b)
        s = np.linalg.norm(v)
        kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
        return rotation_matrix

    @staticmethod
    def r_to_rpy(R):
        # Ensure that the matrix is a valid rotation matrix
        assert np.allclose(np.dot(R, R.T), np.eye(3)), "Matrix is not orthogonal"
        assert np.isclose(np.linalg.det(R), 1), "Matrix is not a proper rotation matrix"
        
        # Roll (x-axis rotation)
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6  # If the sy value is close to zero

        if not singular:
            rx = math.atan2(R[2, 1], R[2, 2])
            ry = math.atan2(-R[2, 0], sy)
            rz = math.atan2(R[1, 0], R[0, 0])
        else:
            rx = math.atan2(-R[1, 2], R[1, 1])
            ry = math.atan2(-R[2, 0], sy)
            rz = 0

        return rx, ry, rz