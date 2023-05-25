import sympy as sy
import time
import numpy as np
import pickle
import os
import re
import math
from numba import njit, types, float64, jit, guvectorize, cuda
import numba



import linecache
def cache_safe_exec(source,lcs=None,gbls=None,cache_name='cache-safe'):
    fp = "<ipython-%s>" %cache_name
    lines = [line + '\n' for line in source.splitlines()]
    linecache.cache[fp] = (len(source), None, lines, fp)
    code = compile(source,fp,'exec')
    l = lcs if lcs is not None else {}
    g = gbls if gbls is not None else globals()
    exec(code,g,l)
    return l,g



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

def compute_jacobian(t):
    # t is a [6][4x4] matrix
    # r values to compute the jacobiant matrix
    r0 = t[0][:3, 2]
    r1 = t[1][:3, 2]
    r2 = t[2][:3, 2]
    r3 = t[3][:3, 2]
    r4 = t[4][:3, 2]
    r5 = t[5][:3, 2]
    p1 = t[0][:3, 3]
    p2 = t[1][:3, 3]
    p3 = t[2][:3, 3]
    p4 = t[3][:3, 3]
    p5 = t[4][:3, 3]
    p6 = t[5][:3, 3]

    # z values to compute the jacobian matrix
    z0 = sy.Matrix.cross(r0, p6 - p1)
    z1 = sy.Matrix.cross(r1, p6 - p2)
    z2 = sy.Matrix.cross(r2, p6 - p3)
    z3 = sy.Matrix.cross(r3, p6 - p4)
    z4 = sy.Matrix.cross(r4, p6 - p5)
    z5 = sy.Matrix.cross(r5, p6 - p6)
    
    #jacobiant matricen
    jaco_matrix = sy.Matrix([[z0[0],z1[0],z2[0],z3[0],z4[0],z5[0]],
                             [z0[1],z1[1],z2[1],z3[1],z4[1],z5[1]],
                             [z0[2],z1[2],z2[2],z3[2],z4[2],z5[2]],
                             [r0[0],r1[0],r2[0],r3[0],r4[0],r5[0]],
                             [r0[1],r1[1],r2[1],r3[1],r4[1],r5[1]],
                             [r0[2],r1[2],r2[2],r3[2],r4[2],r5[2]]])
    
    # Black box correction. Please do not look more into it
    cols_to_flip = [0,1,2,5]
    for col in cols_to_flip:
        jaco_matrix[:, col] = -jaco_matrix[:, col]
    
    theta1, theta2, theta3, theta4, theta5, theta6 = sy.symbols('theta1 theta2 theta3 theta4 theta5 theta6')
    calculation_matrix = f"[\
    [{jaco_matrix[0,0]},{jaco_matrix[0,1]},{jaco_matrix[0,2]},{jaco_matrix[0,3]},{jaco_matrix[0,4]},{jaco_matrix[0,5]}],\
    [{jaco_matrix[1,0]},{jaco_matrix[1,1]},{jaco_matrix[1,2]},{jaco_matrix[1,3]},{jaco_matrix[1,4]},{jaco_matrix[1,5]}],\
    [{jaco_matrix[2,0]},{jaco_matrix[2,1]},{jaco_matrix[2,2]},{jaco_matrix[2,3]},{jaco_matrix[2,4]},{jaco_matrix[2,5]}],\
    [{jaco_matrix[3,0]},{jaco_matrix[3,1]},{jaco_matrix[3,2]},{jaco_matrix[3,3]},{jaco_matrix[3,4]},{jaco_matrix[3,5]}],\
    [{jaco_matrix[4,0]},{jaco_matrix[4,1]},{jaco_matrix[4,2]},{jaco_matrix[4,3]},{jaco_matrix[4,4]},{jaco_matrix[4,5]}],\
    [{jaco_matrix[5,0]},{jaco_matrix[5,1]},{jaco_matrix[5,2]},{jaco_matrix[5,3]},{jaco_matrix[5,4]},{jaco_matrix[5,5]}]]"
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
    with open("jacobian_raw.txt","w") as f:
        f.write(calculation_matrix)
    f.close()
    matrix_function_string = f"@jit('float64[:, :](float64[:])', nopython=True, cache=True)\ndef jacob0(qt):\n\
    st1, ct1 = np.sin(qt[0]), np.cos(qt[0])\n\
    st2, ct2 = np.sin(qt[1]), np.cos(qt[1])\n\
    st3, ct3 = np.sin(qt[2]), np.cos(qt[2])\n\
    st4, ct4 = np.sin(qt[3]), np.cos(qt[3])\n\
    st5, ct5 = np.sin(qt[4]), np.cos(qt[4])\n\
    st6, ct6 = np.sin(qt[5]), np.cos(qt[5])\n\
    sqrt3 = np.sqrt(3)\n\
    arr = np.array({calculation_matrix})\n\
    return arr"
    #print(matrix_function_string)
    j,k = cache_safe_exec(matrix_function_string, gbls={**globals()})#,globals()) 
    global jacob0
    jacob0 = j['jacob0']


def compute_b_jacobian(t):
    # t is a [6][4x4] matrix
    # r values to compute the jacobiant matrix
    r0 = t[0][:3, 2]
    r1 = t[1][:3, 2]
    r2 = t[2][:3, 2]
    r3 = t[3][:3, 2]
    r4 = t[4][:3, 2]
    r5 = t[5][:3, 2]
    p1 = t[0][:3, 3]
    p2 = t[1][:3, 3]
    p3 = t[2][:3, 3]
    p4 = t[3][:3, 3]
    p5 = t[4][:3, 3]
    p6 = t[5][:3, 3]

    # z values to compute the jacobian matrix
    z0 = sy.Matrix.cross(r0, p6 - p1)
    z1 = sy.Matrix.cross(r1, p6 - p2)
    z2 = sy.Matrix.cross(r2, p6 - p3)
    z3 = sy.Matrix.cross(r3, p6 - p4)
    z4 = sy.Matrix.cross(r4, p6 - p5)
    z5 = sy.Matrix.cross(r5, p6 - p6)
    
    #jacobiant matricen
    jaco_matrix = sy.Matrix([[z0[0],z1[0],z2[0],z3[0],z4[0],z5[0]],
                             [z0[1],z1[1],z2[1],z3[1],z4[1],z5[1]],
                             [z0[2],z1[2],z2[2],z3[2],z4[2],z5[2]],
                             [r0[0],r1[0],r2[0],r3[0],r4[0],r5[0]],
                             [r0[1],r1[1],r2[1],r3[1],r4[1],r5[1]],
                             [r0[2],r1[2],r2[2],r3[2],r4[2],r5[2]]])
    
    # Black box correction. Please do not look more into it
    cols_to_flip = [0,1,2,5]
    for col in cols_to_flip:
        jaco_matrix[:, col] = -jaco_matrix[:, col]
    
    theta1, theta2, theta3, theta4, theta5, theta6 = sy.symbols('theta1 theta2 theta3 theta4 theta5 theta6')
    calculation_matrix = f"[\
    [{jaco_matrix[0,0]},{jaco_matrix[0,1]},{jaco_matrix[0,2]},{jaco_matrix[0,3]},{jaco_matrix[0,4]},{jaco_matrix[0,5]}],\
    [{jaco_matrix[1,0]},{jaco_matrix[1,1]},{jaco_matrix[1,2]},{jaco_matrix[1,3]},{jaco_matrix[1,4]},{jaco_matrix[1,5]}],\
    [{jaco_matrix[2,0]},{jaco_matrix[2,1]},{jaco_matrix[2,2]},{jaco_matrix[2,3]},{jaco_matrix[2,4]},{jaco_matrix[2,5]}],\
    [{jaco_matrix[3,0]},{jaco_matrix[3,1]},{jaco_matrix[3,2]},{jaco_matrix[3,3]},{jaco_matrix[3,4]},{jaco_matrix[3,5]}],\
    [{jaco_matrix[4,0]},{jaco_matrix[4,1]},{jaco_matrix[4,2]},{jaco_matrix[4,3]},{jaco_matrix[4,4]},{jaco_matrix[4,5]}],\
    [{jaco_matrix[5,0]},{jaco_matrix[5,1]},{jaco_matrix[5,2]},{jaco_matrix[5,3]},{jaco_matrix[5,4]},{jaco_matrix[5,5]}]]"
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
    matrix_function_string = f"@jit('float64[:, :, :](float64[:, :])', nopython=True, cache=True)\ndef b_jacob0(qt):\n\
    out = np.zeros((qt.shape[0],6,6))\n\
    for i in range(qt.shape[0]):\n\
        st1, ct1 = np.sin(qt[i, 0]), np.cos(qt[i, 0])\n\
        st2, ct2 = np.sin(qt[i, 1]), np.cos(qt[i, 1])\n\
        st3, ct3 = np.sin(qt[i, 2]), np.cos(qt[i, 2])\n\
        st4, ct4 = np.sin(qt[i, 3]), np.cos(qt[i, 3])\n\
        st5, ct5 = np.sin(qt[i, 4]), np.cos(qt[i, 4])\n\
        st6, ct6 = np.sin(qt[i, 5]), np.cos(qt[i, 5])\n\
        sqrt3 = np.sqrt(3)\n\
        arr = np.array({calculation_matrix})\n\
        out[i] = arr\n\
    return out"
    #print(matrix_function_string)
    j,k = cache_safe_exec(matrix_function_string, gbls={**globals()})#,globals()) 
    global b_jacob0
    b_jacob0 = j['b_jacob0']


def matrix(t):
    #x, y, z, rx, ry, rz
    M = np.array([
        [np.cos(t[5])*np.cos(t[4]),np.cos(t[5])*np.sin(t[4])*np.sin(t[3])-np.sin(t[5])*np.cos(t[3]),np.cos(t[5])*np.sin(t[4])*np.cos(t[3])+np.sin(t[5])*np.sin(t[3]),t[0]],
        [np.sin(t[5])*np.cos(t[4]),np.sin(t[5])*np.sin(t[4])*np.sin(t[3])+np.cos(t[5])*np.cos(t[3]),np.sin(t[5])*np.sin(t[4])*np.cos(t[3])-np.cos(t[5])*np.sin(t[3]),t[1]],
        [-np.sin(t[4]),np.cos(t[4])*np.sin(t[3]),np.cos(t[4])*np.cos(t[3]),t[2]],
        [0,0,0,1]
        ])
    return M

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
    with open("fkine_raw.txt","w") as f:
        f.write(calculation_matrix)
    f.close()
    matrix_function_string = f"@jit('float64[:, :](float64[:])', nopython=True, cache=True)\ndef fkine(qt):\n\
    st1, ct1 = np.sin(qt[0]), np.cos(qt[0])\n\
    st2, ct2 = np.sin(qt[1]), np.cos(qt[1])\n\
    st3, ct3 = np.sin(qt[2]), np.cos(qt[2])\n\
    st4, ct4 = np.sin(qt[3]), np.cos(qt[3])\n\
    st5, ct5 = np.sin(qt[4]), np.cos(qt[4])\n\
    st6, ct6 = np.sin(qt[5]), np.cos(qt[5])\n\
    sqrt3 = np.sqrt(3)\n\
    arr = np.array({calculation_matrix})\n\
    return arr"
    l, g = cache_safe_exec(matrix_function_string, gbls={**globals()})#,globals())
    global fkine
    fkine = l['fkine']


def generate_b_fk_matrix(matrix):
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
    
    matrix_function_string = f"@jit('float64[:, :, :](float64[:, :])', nopython=True, cache=True)\ndef b_fkine(qt):\n\
    out = np.zeros((qt.shape[0],4,4))\n\
    for i in range(qt.shape[0]):\n\
        st1, ct1 = np.sin(qt[i, 0]), np.cos(qt[i, 0])\n\
        st2, ct2 = np.sin(qt[i, 1]), np.cos(qt[i, 1])\n\
        st3, ct3 = np.sin(qt[i, 2]), np.cos(qt[i, 2])\n\
        st4, ct4 = np.sin(qt[i, 3]), np.cos(qt[i, 3])\n\
        st5, ct5 = np.sin(qt[i, 4]), np.cos(qt[i, 4])\n\
        st6, ct6 = np.sin(qt[i, 5]), np.cos(qt[i, 5])\n\
        sqrt3 = np.sqrt(3)\n\
        arr = np.array({calculation_matrix})\n\
        out[i] = arr\n\
    return out"
    l, g = cache_safe_exec(matrix_function_string, gbls={**globals()})#,globals())
    global b_fkine
    b_fkine = l['b_fkine']
    
    
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
        
        # DH params in alpha, a, d, theta
        j_01 = mdh(0,0,D1,-theta1+sy.pi)
        j_12 = mdh(sy.pi/2,0,0,-theta2+sy.pi/2)
        j_23 = mdh(sy.pi,-D2,e2,-theta3+sy.pi/2+sy.pi)
        j_34 = mdh(sy.pi/2,0,-d4b,theta4+sy.pi*2)
        j_45 = mdh(-2*aa,0,-d5b,theta5+sy.pi*2)
        j_56 = mdh(2*aa,0,0,theta6)
        j_6t = mdh(sy.pi,0,d6b,-sy.pi/2) # a for tool er 42mm i Rhod, d er 155.67+128.3 i Rhod
        j_5t = j_56 @ j_6t
        
        t_matrix = transform_matrix([j_01, j_12, j_23, j_34, j_45, j_5t],True)
        with open(t_filepath,'wb') as f:
            pickle.dump(t_matrix,f)
    if debug:
        print("generating fk matrix and computing jacobian...")
    generate_fk_matrix(t_matrix[5])
    generate_b_fk_matrix(t_matrix[5])
    compute_jacobian(t_matrix)
    compute_b_jacobian(t_matrix)
    if debug:
        print("Done!")

if __name__ == "__main__":
    setup_module()
    # setup_module(debug=True)
    # from Jaco2_erobot import KinovaJaco2
    # robot = KinovaJaco2()
    
    # #current = np.array([350,150,70,260,120,500])*deg
    # #target = np.array([-206.268310546875,223.7817347049713,296.8748211860657,2.564805269241333,-0.014723804779350758,-0.047133155167102814])
    # """ 
    # current = np.array([614.1727905273438,223.640625,137.4264678955078,257.3863830566406,177.88636779785156,438.40911865234375])*deg
    # target = np.array([201.63077116012573,-452.2587060928345,343.17320585250854,-2.631370782852173,0.09115708619356155,3.003751277923584])

    # target = np.array([356.196,64.9857,1031.75,-0.00197093,0.797187,-1.66086])
    # current = np.array([-0.11029411852359772, 179.765625, 225.44117736816406, 0.06818182021379471, 0.0, 5.25])*deg

    # current = np.array([275.3492736816406,167.625,57.352943420410156,240.4091033935547,83.11363983154297,435.68182373046875])*deg
    # target = np.array([213.15357089042664,-256.72733783721924,507.89934396743774,1.6402174234390259,1.1175113916397095,0.13594458997249603])
    
    # current = np.array([324.0992736816406,203.296875,94.6875,222.20455932617188,147.40908813476562,400.3636474609375])*deg
    # target = np.array([-128.40500473976135,-399.2099165916443,499.55448508262634,1.691123604774475,1.0607322454452515,0.09861937165260315])

    # current = np.array([413.8786926269531,202.5,85.36764526367188,305.18182373046875,109.63636779785156,475.7045593261719])*deg
    # target = np.array([-206.268310546875,223.7817347049713,296.8748211860657,2.564805269241333,-0.014723804779350758,-0.047133155167102814])
    # """
    # #solution = ik(target,current)
    # #print(f"Calculated joints: {np.round(solution*rad,4)}")
    # diff_dist_total = []
    # diff_ang_total = []
    # st = time.time()
    # for i in range(10):
    #     q = np.random.uniform(-3.14*2, 3.14*4, 6)
    #     a = jacob0(*q)
    #     b = robot.jacob0(q)
    #     diff = b - a
    #     diff_dist = np.sum(diff[:3])
    #     diff_ang = np.sum(diff[3:])
    #     diff_dist_total.append(diff_dist)
    #     diff_ang_total.append(diff_ang)
    #     print("a")
    #     print(np.round(a,4))
    #     print("b")
    #     print(np.round(b,4))
    #     print(f"sum diff distance = {diff_dist}   |   sum diff angle = {diff_ang}")
    # et = time.time()
    # print(f"{et-st} seconds")
    # # print median, avg, and max
    # print(f"Median diff distance: {np.median(diff_dist_total)}")
    # print(f"Average diff distance: {np.mean(diff_dist_total)}")
    # print(f"Maximum diff distance: {np.max(diff_dist_total)}")
    # print(f"Median diff angle: {np.median(diff_ang_total)}")
    # print(f"Average diff angle: {np.mean(diff_ang_total)}")
    # print(f"Maximum diff angle: {np.max(diff_ang_total)}")
# else:
#     setup_module()