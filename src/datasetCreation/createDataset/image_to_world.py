from math import cos, sin
import numpy as np

def getT(th, c):
    Ry = np.matrix([[cos(th), 0, sin(th), 0],[0, 1, 0, 0],[-sin(th),0,cos(th),0],[0, 0, 0, 1]])
    c = np.matrix([[1, 0, 0, -c[0]], [0, 1, 0, -c[1]], [0, 0, 1, -c[2]], [0, 0, 0, 1]])
    return np.dot(Ry, c)

def unmap(val):
    minIn = -180
    maxIn = 493
    minOut = -126
    maxOut = 360
    return (val-minOut)/(maxOut-minOut)*(maxIn-minIn)

def fromImageToWorld(imageCoords):
    width = 1920
    height = 1080
    xImage = imageCoords[0]
    yImage = imageCoords[1]
    xl = xImage/width-0.5
    yl = unmap(yImage-height/2-130)/height
    th = -3.1415926535/6
    fx = 0.405
    camera = [-0.39, 0.59, 1.39]
    T = getT(th, camera)
    pc1_pc0 = -xl/fx
    pc2_pc0 = -yl
    a = (pc2_pc0*0.5977499099999999+0.255073)/(sin(0.523599)-pc2_pc0*cos(0.523599))
    b = 0.59+pc1_pc0*0.5977499099999999+pc1_pc0*cos(0.523599)*a
    return [a, b, 0.87]
