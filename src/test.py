import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def set_camera(rot):
    a = rot[0]
    b = rot[1]
    c = rot[2]
    Rz = np.array([[np.cos(c), -np.sin(c), 0.],
                   [np.sin(c), np.cos(c), 0.],
                   [0., 0., 1.]])

    Ry = np.array([[np.cos(b), 0., np.sin(b)],
                   [0., 1., 0.],
                   [-np.sin(b), 0., np.cos(b)]])

    Rx = np.array([[1., 0., 0.],
                   [0., np.cos(a), -np.sin(a)],
                   [0., np.sin(a), np.cos(a)]])

    R = np.dot(Rz, np.dot(Ry, Rx))

    return R


def pix2vect(u, v):
    """
    Calculates the direction vector in camera coords

    :param u: x pixel
    :param v: y pixel
    :return: vector end  in camera local coords
    """

    alpha = np.deg2rad(((1024 / 2) - u) * 0.043)
    beta = np.deg2rad((v - (768 / 2)) * 0.05)
    x = np.sin(alpha) * np.cos(beta)
    z = np.cos(alpha) * np.cos(beta)
    y = np.sin(beta)

    return np.array([x, y, z])


def vect2coord(pose, rot, vect):
    """
    Returns the end point of the vector in global coords

    :param pose: position of the camera
    :param rot: rotation matrix of the camera
    :param vect: vector in camera coords
    :return: coord: vector end point
    """

    rot_vect = np.dot(rot, vect)
    coord = pose + rot_vect
    return coord


def line_distance(a0, a1, b0, b1):
    # Calculate denomitator
    A = a1 - a0
    B = b1 - b0
    magA = np.linalg.norm(A)
    magB = np.linalg.norm(B)

    _A = A / magA
    _B = B / magB

    cross = np.cross(_A, _B)
    denom = np.linalg.norm(cross) ** 2

    # Lines criss-cross: Calculate the projected closest points
    t = (b0 - a0)
    detA = np.linalg.det([t, _B, cross])
    detB = np.linalg.det([t, _A, cross])

    t0 = detA / denom
    t1 = detB / denom

    pA = a0 + (_A * t0)  # Projected closest point on segment A
    pB = b0 + (_B * t1)  # Projected closest point on segment B

    return pA, pB, np.linalg.norm(pA - pB)


cam1_pix = np.array([[695, 262],
                     [602, 285]])

cam2_pix = np.array([[574, 404],
                     [658, 430]])

cam1_pose = np.array([0.585, -1.68, 1.885])
cam2_pose = np.array([-0.585, -1.68, 1.885])
cam1_rot = np.array([[1, 0, 0],
                     [0, 1, 0],
                     [0, 0, 1]])
cam2_rot = np.array([[1, 0, 0],
                     [0, 1, 0],
                     [0, 0, 1]])

tilt1 = np.array([42, -15, 0])
tilt2 = np.array([50, 20, -2])
cam1_angle = np.array([np.deg2rad(90 + tilt1[0]), np.deg2rad(tilt1[1]), np.deg2rad(180 + tilt1[2])])
cam2_angle = np.array([np.deg2rad(90 + tilt2[0]), np.deg2rad(tilt2[1]), np.deg2rad(180 + tilt2[2])])

# set the rotation matrix for each wii
cam1_rot = set_camera(cam1_angle)
cam2_rot = set_camera(cam2_angle)

loc1 = np.array([[-1., -1., -1.],
                 [-1., -1., -1.]])

loc2 = np.array([[-1., -1., -1.],
                 [-1., -1., -1.]])

glob1 = np.array([[-1., -1., -1.],
                  [-1., -1., -1.]])
glob2 = np.array([[-1., -1., -1.],
                  [-1., -1., -1.]])

point1 = np.array([[-1., -1., -1.],
                  [-1., -1., -1.]])

point2 = np.array([[-1., -1., -1.],
                  [-1., -1., -1.]])

for i in range(0, 2):
    loc1[i] = pix2vect(cam1_pix[i][0], cam1_pix[i][1])
    glob1[i] = vect2coord(cam1_pose, cam1_rot, loc1[i])
    print("Local {} cam 1 = {}".format(i, loc1))
    print("Global {} cam 1 = {}".format(i, glob1[i]))
    loc2[i] = pix2vect(cam2_pix[i][0], cam2_pix[i][1])
    glob2[i] = vect2coord(cam2_pose, cam2_rot, loc2[i])
    print("Local {} cam 2 = {}".format(i, loc2[i]))
    print("Global {} cam 2 = {}".format(i, glob2[i]))

detected = np.array([[-1., -1., -1],
                     [-1., -1., -1]])
found = np.array([False, False])
for i in range(0, 2):
    distance = np.array([10., 10., 10.])
    for j in range(0, 2):
        if found[j]:
            continue
        else:
            if not np.any(glob2[j]):
                break
            else:
                point1[j], point2[j], distance[j] = line_distance(cam1_pose, glob1[i],
                                                                  cam2_pose, glob2[j])
    index = distance.argmin()
    point1[i] = point1[index]
    point2[i] = point2[index]
    found[index] = True

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
"""
ax.plot3D(np.array([cam1_pose[0], glob1[0][0]]), np.array([cam1_pose[1], glob1[0][1]]),
          np.array([cam1_pose[2], glob1[0][2]]))
ax.plot3D(np.array([cam1_pose[0], glob1[1][0]]), np.array([cam1_pose[1], glob1[1][1]]),
          np.array([cam1_pose[2], glob1[1][2]]), c='r')

ax.plot3D(np.array([cam2_pose[0], glob2[0][0]]), np.array([cam2_pose[1], glob2[0][1]]),
          np.array([cam2_pose[2], glob2[0][2]]))
ax.plot3D(np.array([cam2_pose[0], glob2[1][0]]), np.array([cam2_pose[1], glob2[1][1]]),
          np.array([cam2_pose[2], glob2[1][2]]), c='r')
"""
ax.plot3D(np.array([cam1_pose[0], point1[0][0]]), np.array([cam1_pose[1], point1[0][1]]),
          np.array([cam1_pose[2], point1[0][2]]))
ax.plot3D(np.array([cam1_pose[0], point1[1][0]]), np.array([cam1_pose[1], point1[1][1]]),
          np.array([cam1_pose[2], point1[1][2]]), c='r')


ax.plot3D(np.array([cam2_pose[0], point2[0][0]]), np.array([cam2_pose[1], point2[0][1]]),
          np.array([cam2_pose[2], point2[0][2]]))
ax.plot3D(np.array([cam2_pose[0], point2[1][0]]), np.array([cam2_pose[1], point2[1][1]]),
          np.array([cam2_pose[2], point2[1][2]]), c='r')

ax.scatter(0.37, -0.12, 0.08)
ax.scatter(point1[0][0], point1[0][1], point1[0][2])
ax.scatter(point2[0][0], point2[0][1], point2[0][2])

ax.scatter(point1[1][0], point1[1][1], point1[1][2])
ax.scatter(point2[1][0], point2[1][1], point2[1][2])

ax.set_xlabel('x axis')
ax.set_ylabel('y axis')
ax.set_zlabel('z axis')
plt.show()

