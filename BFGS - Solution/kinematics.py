import sympy as sy
import time
import numpy as np
from scipy.optimize import minimize
import scipy.spatial.transform as transform
import pickle
import os
import re
from numba import njit, types, float64
from MQTT import MqttClient
mqtt_client = MqttClient()
deg = np.pi/180
rad = 180/np.pi

def mdh(alpha,a,d,theta):
    link = sy.Matrix([
    [sy.cos(theta), -sy.sin(theta), 0, a],
    [sy.sin(theta)*sy.cos(alpha), sy.cos(theta)*sy.cos(alpha), -sy.sin(alpha), -sy.sin(alpha)*d],
    [sy.sin(theta)*sy.sin(alpha), sy.cos(theta)*sy.sin(alpha), sy.cos(alpha), sy.cos(alpha)*d],
    [0, 0, 0, 1]
    ])
    #print(f"alpha: {alpha}  a: {a}  d: {d}  theta: {theta}")
    return link

def transform_matrix(joints,simpl=True):
    # Return: list of transformations [T01, T02,.. T06] in symbolic form.
    st = time.time()
    Tmatrix = joints[0]
    t_list = [Tmatrix]
    for i in range(len(joints)-1):
        Tmatrix = Tmatrix * joints[i+1]
        t_list.append(Tmatrix)
    if (simpl):
        for j in t_list:
            j = sy.simplify(j)
    et = time.time()
    print(f"Took {et-st} seconds to make transform matrix!")
    return t_list

def matrix(t):
    #x, y, z, rx, ry, rz
    M = np.array([
        [np.cos(t[5])*np.cos(t[4]),np.cos(t[5])*np.sin(t[4])*np.sin(t[3])-np.sin(t[5])*np.cos(t[3]),np.cos(t[5])*np.sin(t[4])*np.cos(t[3])+np.sin(t[5])*np.sin(t[3]),t[0]],
        [np.sin(t[5])*np.cos(t[4]),np.sin(t[5])*np.sin(t[4])*np.sin(t[3])+np.cos(t[5])*np.cos(t[3]),np.sin(t[5])*np.sin(t[4])*np.cos(t[3])-np.cos(t[5])*np.sin(t[3]),t[1]],
        [-np.sin(t[4]),np.cos(t[4])*np.sin(t[3]),np.cos(t[4])*np.cos(t[3]),t[2]],
        [0,0,0,1]
        ])
    return M

def ik_err_function(x0,target,prev_joints,debug=False):
    # Compute the transformation matrix m of the robot arm
    m = matrix_function(x0[0],x0[1],x0[2],x0[3],x0[4],x0[5])

    # Compute difference between current and target rotation matrices
    m_rotmat = m[:3, :3]
    t_rotmat = target[:3, :3]
    R = np.dot(m_rotmat.T, t_rotmat)
    e_rot_weight = 28
    e_rot = np.linalg.norm(transform.Rotation.from_matrix(R).as_rotvec())

    # Compute the euler distance between the target and current position
    e_loc_m= m[:3,3]
    e_loc_t = target[:3,3]
    e_loc_weight = 100
    e_loc = np.sqrt(np.sum((e_loc_m - e_loc_t) ** 2))

    # Compute regularization term
    e_loc_reg_weight = 2e-2
    e_loc_reg = np.linalg.norm(x0 - prev_joints) * e_loc_reg_weight

    if debug:
        m_r = transform.Rotation.from_matrix(m[:3, :3]).as_euler('xyz',degrees=True)
        t_r = transform.Rotation.from_matrix(target[:3, :3]).as_euler('xyz',degrees=True)
        print(f"Loc: {np.round((e_loc_m-e_loc_t)*1000,6)}mm Rot: {np.round(m_r-t_r,6)}deg")
        print(f"Error Weighted: {np.round(e_loc*e_loc_weight,6)}  {np.round(e_rot*e_rot_weight,6)}  {np.round(e_loc_reg*e_loc_reg_weight,6)}")
    # Sum all the cost terms together
    error = e_loc*e_loc_weight + e_rot*e_rot_weight + e_loc_reg*e_loc_reg_weight
    return error

def ik(target,x0,debug=False):
    target = np.array(target)
    x0 = np.array(x0)
    location = target[:3]  # first three elements
    rotation = target[3:]  # last three elements
    # convert from intrinsic to extrinsic rpy
    rotation = transform.Rotation.from_euler('XYZ', rotation).as_euler('xyz')
    # convert from mm to m
    location = location/1000
    target = np.concatenate((location, rotation))
    target_m = matrix(target)
    args = (target_m,x0)
    tol = 1e-7
    options = {'maxiter': 1000}
    solution = minimize(ik_err_function,x0,args=args,method='BFGS',tol=tol,options=options)
    # print end stats for error function
    if debug:
        ik_err_function(solution.x,target_m,x0,debug=True)
    if solution.fun > 5:
        print("Inverse kinematics could not find a good solution! Possible singularity!")
        mqtt_client.publish_message("P2/AAU/info", "Possible singularity!")
        return x0
    return solution.x

def generate_fk_matrix(matrix):
    theta1, theta2, theta3, theta4, theta5, theta6 = sy.symbols('theta1 theta2 theta3 theta4 theta5 theta6')
    calculation_matrix = f"[\
    [{matrix[0,0]},{matrix[0,1]},{matrix[0,2]},{matrix[0,3]}],\
    [{matrix[1,0]},{matrix[1,1]},{matrix[1,2]},{matrix[1,3]}],\
    [{matrix[2,0]},{matrix[2,1]},{matrix[2,2]},{matrix[2,3]}],\
    [0.0,0.0,0.0,1.0]]"
    calculation_matrix = calculation_matrix.replace("sqrt","np.sqrt")
    calculation_matrix = calculation_matrix.replace("sin","np.sin")
    calculation_matrix = calculation_matrix.replace("cos","np.cos")
    calculation_matrix = re.sub(r"np\.cos\((\w+)\s*[\+\-]\s*3\.1415\d*\)",r"-np.cos(\1)",calculation_matrix)
    calculation_matrix = re.sub(r"np\.sin\((\w+)\s*[\+\-]\s*3\.1415\d*\)",r"-np.sin(\1)",calculation_matrix)
    calculation_matrix = re.sub(r"np\.sin\((\w+)\s*\+\s*1\.5707\d*\)",r"np.cos(\1)",calculation_matrix)
    calculation_matrix = re.sub(r"np\.cos\((\w+)\s*\+\s*1\.5707\d*\)",r"-np.sin(\1)",calculation_matrix)
    calculation_matrix = re.sub(r"np\.sin\((\w+)\s*\-\s*(\w+)\)",r"(np.sin(\1)*np.cos(\2)-np.cos(\1)*np.sin(\2))",calculation_matrix)
    calculation_matrix = re.sub(r"np\.cos\((\w+)\s*\-\s*(\w+)\)",r"(np.cos(\1)*np.cos(\2)+np.sin(\1)*np.sin(\2))",calculation_matrix)
    calculation_matrix = calculation_matrix.replace("np.sin(theta1)","st1")
    calculation_matrix = calculation_matrix.replace("np.sin(theta2)","st2")
    calculation_matrix = calculation_matrix.replace("np.sin(theta3)","st3")
    calculation_matrix = calculation_matrix.replace("np.sin(theta4)","st4")
    calculation_matrix = calculation_matrix.replace("np.sin(theta5)","st5")
    calculation_matrix = calculation_matrix.replace("np.sin(theta6)","st6")
    calculation_matrix = calculation_matrix.replace("np.cos(theta1)","ct1")
    calculation_matrix = calculation_matrix.replace("np.cos(theta2)","ct2")
    calculation_matrix = calculation_matrix.replace("np.cos(theta3)","ct3")
    calculation_matrix = calculation_matrix.replace("np.cos(theta4)","ct4")
    calculation_matrix = calculation_matrix.replace("np.cos(theta5)","ct5")
    calculation_matrix = calculation_matrix.replace("np.cos(theta6)","ct6")
    calculation_matrix = calculation_matrix.replace("np.sqrt(3)","sqrt3")
    matrix_function_string = f"@njit('float64[:, :](float64, float64, float64, float64, float64, float64)')\ndef matrix_function(theta1,theta2,theta3,theta4,theta5,theta6):\n\
    st1, ct1 = np.sin(theta1), np.cos(theta1)\n\
    st2, ct2 = np.sin(theta2), np.cos(theta2)\n\
    st3, ct3 = np.sin(theta3), np.cos(theta3)\n\
    st4, ct4 = np.sin(theta4), np.cos(theta4)\n\
    st5, ct5 = np.sin(theta5), np.cos(theta5)\n\
    st6, ct6 = np.sin(theta6), np.cos(theta6)\n\
    sqrt3 = np.sqrt(3)\n\
    return np.array({calculation_matrix})"
    exec(matrix_function_string,globals())

def setup_module(clean=False,debug=False):
    #print("<<<IK calculations for Kinova j2n6s300>>>")
    # All rotations follows fixed XYZ (Z*Y*X)
    # Rotations are calculated in rad, and displayed in deg
    # Lenghts are in meters

    # Load/Calculate fk transformation matrix
    t_filepath = 'fk_matrix.pkl'
    if os.path.exists(t_filepath) and not clean:
        with open(t_filepath,'rb') as f:
            if debug:
                print("Loading transformation matrix...")
            t_matrix = pickle.load(f)
    else:
        if debug:
            print("Calculating transformation matrix...")
            print("May take some time")
        theta1, theta2, theta3, theta4, theta5, theta6 = sy.symbols('theta1 theta2 theta3 theta4 theta5 theta6')
        #Kinova j2n6s300
        D1 = 0.2755
        D2 = 0.41
        e2 = 0.0098
        D3 = 0.2073
        D4 = 0.0741
        D5 = 0.0741
        D6 = 0.16    
        aa = sy.pi/6
        d4b = D3+(sy.sin(aa)/sy.sin(2*aa))*D4
        d5b = (sy.sin(aa)/sy.sin(2*aa))*D4+(sy.sin(aa)/sy.sin(2*aa))*D5
        d6b = (sy.sin(aa)/sy.sin(2*aa))*D5+D6
        j_01 = mdh(0,0,D1,-theta1+sy.pi)
        j_12 = mdh(sy.pi/2,0,0,-theta2+sy.pi/2)
        j_23 = mdh(sy.pi,-D2,e2,-theta3+sy.pi/2+sy.pi)
        j_34 = mdh(sy.pi/2,0,-d4b,theta4+sy.pi*2)
        j_45 = mdh(-2*aa,0,-d5b,theta5+sy.pi*2)
        j_56 = mdh(2*aa,0,0,theta6)
        j_6t = mdh(sy.pi,0,d6b,-sy.pi/2)
        j_5t = j_56 @ j_6t
        t_matrix = transform_matrix([j_01, j_12, j_23, j_34, j_45, j_5t],True)
        with open(t_filepath,'wb') as f:
            pickle.dump(t_matrix,f)
    generate_fk_matrix(t_matrix[5])
    if debug:
        print("Done!")

if __name__ == "__main__":
    setup_module(debug=True)
    for i in range(100):
        ik(np.random.uniform(-np.pi, np.pi, 6),[0.25,2.2,5,1,1,-1.3],debug=True)
else:
    setup_module()