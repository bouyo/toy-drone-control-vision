import numpy as np
import time
import matplotlib.pyplot as plt
import csv
from mpl_toolkits.mplot3d import Axes3D
import matplotlib


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

    alpha = np.deg2rad(((1024 / 2) - u) * 0.04474)
    beta = np.deg2rad((v - (768 / 2)) * 0.04474)
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


def draw_lines(cam_pose, end_poses, axis, color='b'):

    axis.set_xlabel('x axis')
    axis.set_ylabel('y axis')
    axis.set_zlabel('z axis')

    for x in range(0, len(end_poses)):
        axis.plot3D(np.array([cam_pose[0], end_poses[x][0]]), np.array([cam_pose[1], end_poses[x][1]]),
                    np.array([cam_pose[2], end_poses[x][2]]), c=color)
    return


cam1_pix = np.array([[57, 513],
                     [277, 476]])

cam2_pix = np.array([[115, 202],
                     [350, 314]])

cam1_pose = np.array([0.585, -1.68, 1.885])
cam2_pose = np.array([-0.585, -1.68, 1.885])
cam1_rot = np.array([[1, 0, 0],
                     [0, 1, 0],
                     [0, 0, 1]])
cam2_rot = np.array([[1, 0, 0],
                     [0, 1, 0],
                     [0, 0, 1]])

tilt1 = np.array([51, -9, -2])
tilt2 = np.array([57, 11, 2])
cam1_angle = np.array([np.deg2rad(90 + tilt1[0]), np.deg2rad(tilt1[1]), np.deg2rad(180 + tilt1[2])])
cam2_angle = np.array([np.deg2rad(90 + tilt2[0]), np.deg2rad(tilt2[1]), np.deg2rad(180 + tilt2[2])])

# set the rotation matrix for each wii
cam1_rot = set_camera(cam1_angle)
cam2_rot = set_camera(cam2_angle)

loc1 = np.array([[-1., -1., -1.],
                 [-1., -1., -1.]])

loc2 = np.array([[-1., -1., -1.],
                 [-1., -1., -1.]])

glob1 = np.array([[-1., -1., -1.]])
glob2 = np.array([[-1., -1., -1.]])

point1 = np.array([[-1., -1., -1.]])

point2 = np.array([[-1., -1., -1.]])

"""
for i in range(0, 1):
    loc1[i] = pix2vect(cam1_pix[i][0], cam1_pix[i][1])
    glob1[i] = vect2coord(cam1_pose, cam1_rot, loc1[i])
    loc2[i] = pix2vect(cam2_pix[i][0], cam2_pix[i][1])
    glob2[i] = vect2coord(cam2_pose, cam2_rot, loc2[i])

detected = np.array([[-1., -1., -1],
                     [-1., -1., -1]])
found = np.array([False, False])
for i in range(0, 1):
    distance = np.array([10., 10., 10.])
    for j in range(0, 1):
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
draw_lines(cam1_pose, point1, ax, 'r')
draw_lines(cam2_pose, point2, ax)
ax.scatter(0.29, -0.95, 0.05)
plt.show()



ax.scatter(0.37, -0.12, 0.08)
ax.scatter(point1[0][0], point1[0][1], point1[0][2])
ax.scatter(point2[0][0], point2[0][1], point2[0][2])

ax.scatter(point1[1][0], point1[1][1], point1[1][2])
ax.scatter(point2[1][0], point2[1][1], point2[1][2])
"""


def tansig(n):
    return 2 / (1 + np.exp(-2 * n)) - 1


x_offset = np.array([15, 31, 64, 32])
x_gain = np.array([0.00205549845837616, 0.0033167495854063, 0.00222222222222222, 0.00334448160535117])

y_offset = np.array([-75, 79, 0])
y_gain = np.array([0.0121951219512195, 0.027027027027027, 0.0666666666666667])

a1 = np.hstack((cam1_pix[0], cam2_pix[0]))
print(a1)
inputs = a1 - x_offset
inputs = np.multiply(inputs, x_gain)
inputs = inputs - 1
print(inputs)
# Layer 1
b1 = np.array([-3.6355608447828018, -0.37478230313700545, -0.28365517100965948, 0.13475925179971274, 0.12345034182560134, 2.048057605462501, 2.2073576920513833, 0.56867066317350934])
IW1_1 = np.array([[4.1748350906340397, -3.2631853112120033, -0.58331098450434704, -1.0752272479650751], [1.1735931177016041, -0.93294031927571019, -1.4430537002579684, -0.26914138183361153], [-0.11388486674129081, 0.057641196462793913, 1.2510735375148498, 0.074943426676298372], [-0.50044938248853454, -0.50865164492642589, 0.5186203261889818, 0.34061728847158995], [-1.3544537606133746, 0.57793663896364011, 1.245973501489779, 0.3195777254715827], [-2.8777765994882669, -0.047375183963823113, 6.5595533967483961, 0.52332452189244216], [-1.3024313160189775, 0.24942608549470482, 5.2583191907756204, 0.21293113718583509], [-1.3031167441443161, 1.0420572303421776, 0.96436912373400441, -0.42009274529707108]])

# Layer 2
b2 = np.array([0.32367322349027206, -0.044989430932538252, -0.63309494832268998])
LW2_1 = np.array([[-0.038678448131542899, -0.17886892060549772, 0.96594692808927485, -0.68962723007823057, -0.38619815870059776, 1.9616296134453197, -1.7345597039146632, -0.41852499725389125], [0.013726639873650937, -2.6961197186996286, 1.3845703827315146, -3.865549342410914, -3.7979221525551745, -1.5287157311592205, 1.9763341179609812, 1.1438001999151957], [-0.11255988909142788, 4.4589846055153322, 0.3103862125049473, -0.98175057088615003, 13.248050747190007, 1.2412420421959969, -1.5573678152922075, -4.3090812945330326]])

a11 = np.dot(IW1_1, inputs)
print(a11)
a12 = np.add(a11, b1)
print(a12)

out1 = tansig(a12)
print(out1)

a2 = np.dot(LW2_1, out1)
a21 = np.add(a2, b2)
print(a21)

out = a21 + 1
out = np.divide(out, y_gain)
out = out + y_offset
print(out)
