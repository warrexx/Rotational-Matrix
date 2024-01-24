import numpy as np

def calculate_angle(v1, v2):
    """Calculate the angle between two vectors."""
    v1_norm = np.linalg.norm(v1)
    v2_norm = np.linalg.norm(v2)
    
    if v1_norm == 0 or v2_norm == 0:
        raise ValueError("One of the vectors is a zero vector.")

    cos_angle = np.dot(v1, v2) / (v1_norm * v2_norm)
    # Correct potential floating point errors
    cos_angle = max(min(cos_angle, 1), -1)
    return np.arccos(cos_angle)

def calculate_rotation_axis(v1, v2):
    """Calculate the rotation axis (normal vector) for two vectors."""
    return np.cross(v1, v2)

def calculate_rotational_matrix(v1, v2):
    """Calculate the rotational matrix to rotate v1 to v2."""
    angle_rad = calculate_angle(v1, v2)
    rotation_axis = calculate_rotation_axis(v1, v2)
    rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)  # Normalize the axis

    cos_angle = np.cos(angle_rad)
    sin_angle = np.sin(angle_rad)
    ux, uy, uz = rotation_axis

    # Construct the rotation matrix
    rotational_matrix = np.array([
        [cos_angle + ux**2 * (1 - cos_angle), ux * uy * (1 - cos_angle) - uz * sin_angle, ux * uz * (1 - cos_angle) + uy * sin_angle],
        [uy * ux * (1 - cos_angle) + uz * sin_angle, cos_angle + uy**2 * (1 - cos_angle), uy * uz * (1 - cos_angle) - ux * sin_angle],
        [uz * ux * (1 - cos_angle) - uy * sin_angle, uz * uy * (1 - cos_angle) + ux * sin_angle, cos_angle + uz**2 * (1 - cos_angle)]
    ])

    return rotational_matrix

# Example usage
v1 = np.array([1, 0, 0])
v2 = np.array([0, 1, 0])
rot_matrix = calculate_rotational_matrix(v1, v2)
print(rot_matrix)
