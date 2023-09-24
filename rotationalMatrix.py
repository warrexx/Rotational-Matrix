import numpy as np
import math

def angle(v1, v2) :     
    return math.acos(np.dot(v1, v2)/(np.linalg.norm(v1)*np.linalg.norm(v2)))
    
def rotationAxis(v1, v2):
    return np.cross(v1, v2)
    
def rotationalMatrix(v1, v2):
    angle_rad = angle(v1, v2) 
    
    rotation_axis = rotationAxis(v1, v2)
    
    rotational_matrix = np.array([[rotation_axis[0]*rotation_axis[0]*(1-math.cos(angle_rad))+math.cos(angle_rad)                 , rotation_axis[1]*rotation_axis[0]*(1-math.cos(angle_rad))-rotation_axis[2]*math.sin(angle_rad), rotation_axis[2]*rotation_axis[0]*(1-math.cos(angle_rad))+rotation_axis[1]*math.sin(angle_rad)],
                                  [rotation_axis[0]*rotation_axis[1]*(1-math.cos(angle_rad)+rotation_axis[2]*math.sin(angle_rad)), rotation_axis[1]*rotation_axis[1]*(1-math.cos(angle_rad))+math.cos(angle_rad)                 , rotation_axis[2]*rotation_axis[1]*(1-math.cos(angle_rad))-rotation_axis[0]*math.sin(angle_rad)],
                                  [rotation_axis[0]*rotation_axis[2]*(1-math.cos(angle_rad)-rotation_axis[1])*math.sin(angle_rad), rotation_axis[1]*rotation_axis[2]*(1-math.cos(angle_rad))+rotation_axis[0]*math.sin(angle_rad), rotation_axis[2]*rotation_axis[2]*(1-math.cos(angle_rad))+math.cos(angle_rad)]])
    
    return rotational_matrix 
    
    