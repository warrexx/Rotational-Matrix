import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

# v1 = [0.59, 9.15, -3.47] # input vector
# # normalized_v1 = v1/np.linalg.norm(v1)


# v2 = [0, 0, -1] # target direction
# # normalized_v2 = v2/np.linalg.norm(v2)

# angle_rad = math.acos(np.dot(v1, v2)/(np.linalg.norm(v1)*np.linalg.norm(v2)))
# angle_deg = np.rad2deg(angle_rad)

# rotation_axis = np.cross(v1, v2)
# rotation_axis = rotation_axis/np.linalg.norm(rotation_axis)

# rotation_matrix = np.array([[rotation_axis[0]*rotation_axis[0]*(1-math.cos(angle_rad))+math.cos(angle_rad)                 , rotation_axis[1]*rotation_axis[0]*(1-math.cos(angle_rad))-rotation_axis[2]*math.sin(angle_rad), rotation_axis[2]*rotation_axis[0]*(1-math.cos(angle_rad))+rotation_axis[1]*math.sin(angle_rad)],
#                             [rotation_axis[0]*rotation_axis[1]*(1-math.cos(angle_rad)+rotation_axis[2]*math.sin(angle_rad)), rotation_axis[1]*rotation_axis[1]*(1-math.cos(angle_rad))+math.cos(angle_rad)                 , rotation_axis[2]*rotation_axis[1]*(1-math.cos(angle_rad))-rotation_axis[0]*math.sin(angle_rad)],
#                             [rotation_axis[0]*rotation_axis[2]*(1-math.cos(angle_rad)-rotation_axis[1])*math.sin(angle_rad), rotation_axis[1]*rotation_axis[2]*(1-math.cos(angle_rad))+rotation_axis[0]*math.sin(angle_rad), rotation_axis[2]*rotation_axis[2]*(1-math.cos(angle_rad))+math.cos(angle_rad)]])

# rotated_v1 = np.matmul(rotation_matrix, v1)

# fig = plt.figure(figsize = plt.figaspect(1))
# ax = fig.add_subplot(projection = '3d')
# ax.plot([0, v1[0]],[0, v1[1]],[0, v1[2]], linewidth = 3)
# ax.plot([0, v2[0]],[0, v2[1]],[0, v2[2]], linewidth = 3)
# ax.plot([0, rotation_axis[0]],[0, rotation_axis[1]],[0, rotation_axis[2]], linewidth = 3)
# ax.plot([0, rotated_v1[0]],[0, rotated_v1[1]],[0, rotated_v1[2]], linewidth = 5)

# ax.set_xlabel('x')
# ax.set_ylabel('y')
# ax.set_zlabel('z')

# # ax.plot([0, 0],[0, 0],[-50, 50],'k--')
# # ax.plot([0, 0],[-50, 50],[0, 0],'k--')
# # ax.plot([-50, 50],[0, 0],[0, 0],'k--')
# # plt.show()

# rotation_matrix
 

class RotationalMatrix:
    def __init__(self, v1, v2):
        self.v1 = v1
        self.v2 = v2
    
    def angle(self, v1, v2) :     
        return math.acos(np.dot(v1, v2)/(np.linalg.norm(v1)*np.linalg.norm(v2)))
    
    def rotationAxis(self, v1, v2):
        return np.cross(v1, v2)
    
    def rotational_matrix(self, v1, v2):
       angle_rad = angle(v1, v2) 
       
       rotation_axis = rotationAxis(v1, v2)
       
       self.rotational_matrix = np.array([[rotation_axis[0]*rotation_axis[0]*(1-math.cos(angle_rad))+math.cos(angle_rad)                 , rotation_axis[1]*rotation_axis[0]*(1-math.cos(angle_rad))-rotation_axis[2]*math.sin(angle_rad), rotation_axis[2]*rotation_axis[0]*(1-math.cos(angle_rad))+rotation_axis[1]*math.sin(angle_rad)],
                                          [rotation_axis[0]*rotation_axis[1]*(1-math.cos(angle_rad)+rotation_axis[2]*math.sin(angle_rad)), rotation_axis[1]*rotation_axis[1]*(1-math.cos(angle_rad))+math.cos(angle_rad)                 , rotation_axis[2]*rotation_axis[1]*(1-math.cos(angle_rad))-rotation_axis[0]*math.sin(angle_rad)],
                                          [rotation_axis[0]*rotation_axis[2]*(1-math.cos(angle_rad)-rotation_axis[1])*math.sin(angle_rad), rotation_axis[1]*rotation_axis[2]*(1-math.cos(angle_rad))+rotation_axis[0]*math.sin(angle_rad), rotation_axis[2]*rotation_axis[2]*(1-math.cos(angle_rad))+math.cos(angle_rad)]])
        
    
    