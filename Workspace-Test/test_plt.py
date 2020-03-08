import ikpy
import numpy as np
import time
import serial
import sys
from math import pi
#import detectFruit
from ikpy import plot_utils
from ikpy import geometry_utils
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D

points = list()

def main():
    fruit_harvest_chain = Chain(name='fruit_harvester', links=[
                URDFLink(
                    name = "55kg",
                    translation_vector=[-0.1843, 0.1202, -0.6482],
                    orientation=[0,0,0],
                    rotation=[0,0,1],
                    bounds=[-1.04719, 1.04719],
                ),
                URDFLink(
                  name="40kg",
                  translation_vector=[0, -0.01807, 0.0678],   #place holder values TODO
                  orientation=[0, 0, 0],
                  rotation=[1, 0, 0],
                  bounds=[-1.308,0.087266],
                ),
                URDFLink(
                  name="25kg",
                  translation_vector=[0, 0.00861, 0.435],    #place holder values TODO
                  orientation=[0, 0, 0],
                  rotation=[-1, 0, 0],
                  bounds=[-1.74533, 1.74533],
                 ),
                URDFLink(
                    name = "gripper",
                    translation_vector=[0, 0.350, -0.06268],
                    orientation=[0,0,0],
                    rotation=[0,0,0],
                )
            ])
    start_pos = [-0.1843, 0.1202-0.01807+0.00861+0.350, -0.6482+0.0678+0.435-0.06268]

    ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d', navigate=1)
    

    for x in np.linspace(-1, 0.5, 26):
        for y in np.linspace(0.5, 1, 11):
            for z in np.linspace(-0.3, 0.3, 11):
                OUT_OF_RANGE = 0
                
                target_frame = [[1, 0, 0,(x)],
                                    [0, 1, 0,(y)],
                                    [0, 0, 1, (z)],
                                    [0, 0, 0, 1]]
                angles = fruit_harvest_chain.inverse_kinematics(target_frame)
                for u in range(len(angles)):
                    angle = angles[u]*180/pi
                    if u == 0:
                        if abs(angle) > 60:
                            OUT_OF_RANGE = 1
                            break
                    elif u == 1:
                        if angle > 0 or angle < -75:
                            OUT_OF_RANGE = 1
                            break
                    elif u == 2:
                        if abs(angle) > 100:
                            OUT_OF_RANGE = 1
                            break
                if not OUT_OF_RANGE:
                    nodes = []
                    transformation_matrixes = fruit_harvest_chain.forward_kinematics(angles, full_kinematics=True)
                    # Get the nodes and the orientation from the tranformation matrix
                    for (index, link) in enumerate(fruit_harvest_chain.links):
                        (node, rotation) = geometry_utils.from_transformation_matrix(transformation_matrixes[index])
                        nodes.append(node)
                    if (np.linalg.norm(nodes[-1][:3] - np.array([i,j,k]))< 0.001):
                        points.append([nodes[-1][0], nodes[-1][1], nodes[-1][2]])
                        fruit_harvest_chain.plot(angles, ax)
    p = np.array(points)
    np.savetxt('points.csv',p, delimiter=',')
    plot_utils.show_figure()

if __name__=='__main__':
    try:
        main()
    except(KeyboardInterrupt):
        p = np.array(points)
        np.savetxt('points.csv',p, delimiter=',',fmt='%d')
        plot_utils.show_figure()