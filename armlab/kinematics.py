import numpy as np
#expm is a matrix exponential function
from scipy.linalg import expm


"""
TODO: Here is where you will write all of your kinematics functions
There are some functions to start with, you may need to implement a few more

"""
# link length parameter
L0 = 80.38 #75.61
L1 = 37.78 #40.82
L2 = 102.03 #99.20
L3 = 71.91 #68.70
L4 = 38.10 #39.20
Lt = 97.10 #107.84


def trans_mat(al, a, d, th):
    H = np.array([
        [np.cos(th), -np.sin(th), 0, a],
        [np.sin(th)*np.cos(al), np.cos(th)*np.cos(al), -np.sin(al), -np.sin(al)*d],
        [np.sin(th)*np.sin(al), np.cos(th)*np.sin(al), np.cos(al), np.cos(al)*d],
        [0, 0, 0, 1]
        ])
    return H


def FK_dh(joint_angles, link):
    theta = joint_angles+np.array([0, -np.pi/2, -np.pi/2, 0, 0, 0])
    DH = [[0, 0, L0+L1, 0],
          [0, 0, 0, theta[0]],
          [-np.pi/2, 0, 0, theta[1]],
          [0, L2, 0, theta[2]],
          [-np.pi/2, 0, L3+L4, theta[3]],
          [np.pi/2, 0, 0, theta[4]],
          [-np.pi/2, 0, 0, theta[5]],
          [0, 0, Lt, 0]]
    T = np.eye(4)
    for i in range(link):
        T = np.dot(T, trans_mat(DH[i][0], DH[i][1], DH[i][2], DH[i][3]))
    T[np.abs(T)<1e-10] = 0
    return T

def FK_pox(joint_angles):
    """
    TODO: implement this function

    Calculate forward kinematics for rexarm
    using product of exponential formulation

    return a 4-tuple (x, y, z, phi) representing the pose of the
    desired link

    note: phi is the euler angle about y in the base frame

    """
    pass


def IK(pose):
    Tbt = pose_to_T(pose)
    Tb0 = trans_mat(0, 0, L0+L1, 0)
    T6t = trans_mat(0, 0, Lt, 0)
    T06 = np.dot(np.dot(np.linalg.inv(Tb0), Tbt), np.linalg.inv(T6t))
    pw = [T06[0][3], T06[1][3], T06[2][3]]
    r = np.sqrt(pw[0]**2+pw[1]**2+pw[2]**2)
    if r<=(L2+L3+L4) and r>=np.sqrt(L2**2+(L3+L4)**2):
        # th1~th3
        if (np.abs(pw[0])<1e-10) and (np.abs(pw[1])<1e-10):
            th1 = 0
        else:
            th1 = np.arctan2(pw[1], pw[0])
        if (np.abs(r-(L2+L3+L4)))<1e-10:
            print()
            th3 = 0
            psi = 0
        else:
            th3 = np.arccos((pw[0]**2+pw[1]**2+pw[2]**2-(L3+L4)**2-L2**2)/(2*(L3+L4)*L2))
            psi = np.arccos((pw[0]**2+pw[1]**2+pw[2]**2+L2**2-(L3+L4)**2)/(2*np.sqrt(pw[0]**2+pw[1]**2+pw[2]**2)*L2))
        th2 = np.pi/2-psi-np.arctan2(pw[2], np.sqrt(pw[0]**2+pw[1]**2))
        M = [
            [th1, th2, th3],
            [th1, th2+2*psi, -th3],
            [np.mod(th1+2*np.pi, 2*np.pi)-np.pi, -th2, -th3],
            [np.mod(th1+2*np.pi, 2*np.pi)-np.pi, -th2-2*psi, th3]
            ]
        # th4~th6
        for i in range(4):
            th46 = wrist_rot(M[i][0], M[i][1], M[i][2], T06)
            for j in range(3):
                M[i].append(th46[j])
        return M
    else:
        print("out of workspace")
        return np.zeros((4,6))


def wrist_rot(th1, th2, th3, T06):
    DH = [[0, 0, 0, th1],
          [-np.pi/2, 0, 0, th2-np.pi/2],
          [0, L2, 0, th3-np.pi/2]]
    T03 = np.eye(4)
    for i in range(3):
        T03 = np.dot(T03, trans_mat(DH[i][0], DH[i][1], DH[i][2], DH[i][3]))
    T = np.dot(np.linalg.inv(T03), T06)
    if (np.abs(T[0][2])<1e-10) and (np.abs(T[2][2])<1e-10) and (np.abs(T[1][2]-1)<1e-10):
        th4 = np.arctan2(-T[2][0], T[0][0])
        th5 = 0.0
        th6 = 0.0
    elif (np.abs(T[0][2])<1e-10) and (np.abs(T[2][2])<1e-10) and (np.abs(T[1][2]+1)<1e-10):
        th4 = np.arctan2(T[2][0], -T[0][0])
        th5 = 0.0
        th6 = 0.0
    else:
        th4 = np.arctan2(T[2][2], -T[0][2])
        th5 = np.arctan2(np.sqrt((T[0][2])**2+(T[2][2])**2), T[1][2])
        th6 = np.arctan2(-T[1][1], T[1][0])
    return [th4, th5, th6]


def pose_to_T(pose):
    T = np.zeros((4, 4))
    T[0][0] = np.cos(pose[3])*np.cos(pose[4])*np.cos(pose[5])-np.sin(pose[3])*np.sin(pose[5])
    T[0][1] = -np.cos(pose[3])*np.cos(pose[4])*np.sin(pose[5])-np.sin(pose[3])*np.cos(pose[5])
    T[0][2] = np.cos(pose[3])*np.sin(pose[4])
    T[0][3] = pose[0]
    T[1][0] = np.sin(pose[3])*np.cos(pose[4])*np.cos(pose[5])+np.cos(pose[3])*np.sin(pose[5])
    T[1][1] = -np.sin(pose[3])*np.cos(pose[4])*np.sin(pose[5])+np.cos(pose[3])*np.cos(pose[5])
    T[1][2] = np.sin(pose[3])*np.sin(pose[4])
    T[1][3] = pose[1]
    T[2][0] = -np.sin(pose[4])*np.cos(pose[5])
    T[2][1] = np.sin(pose[4])*np.sin(pose[5])
    T[2][2] = np.cos(pose[4])
    T[2][3] = pose[2]
    T[3][0] = 0
    T[3][1] = 0
    T[3][2] = 0
    T[3][3] = 1
    return T


def get_euler_angles_from_T(T):
    if (np.abs(T[0][2])<1e-10) and (np.abs(T[1][2])<1e-10) and (np.abs(T[2][2]-1)<1e-10):
        theta = 0.0
        phi = 0.0
        psi = np.arctan2(-T[0][1], T[0][0])
    elif (np.abs(T[0][2])<1e-10) and (np.abs(T[1][2])<1e-10) and (np.abs(T[2][2]+1)<1e-10):
        theta = np.pi
        phi = 0.0
        psi = np.arctan2(T[0][1], -T[0][0])
    else:
        theta = np.arctan2(np.sqrt((T[2][0])**2+(T[2][1])**2), T[2][2])
        phi = np.arctan2(T[1][2], T[0][2])
        psi = np.arctan2(T[2][1], -T[2][0])
    return [phi, theta, psi]


def get_pose_from_T(T):
    """
    TODO: implement this function
    return the joint pose from a T matrix
    of the form (x,y,z,phi) where phi is rotation about base frame y-axis

    """
    pass


def to_s_matrix(w,v):
    """
    TODO: implement this function
    Find the [s] matrix for the POX method e^([s]*theta)
    """
    pass
