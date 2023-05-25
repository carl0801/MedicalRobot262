import daniel_kine as dk
import pickle
import os
import time
import numpy as np
import utility as util
import np_ik as ik
import timeit
import trajectory_interpolation as ntrp
from Jaco2_erobot import KinovaJaco2
import plotter


# TODO: Perform N searches and select the solution which has the lowest distance in q values from the current q anglesa
robot = KinovaJaco2()

cart_via = np.load("cart_trajectory_via.npy")
#cart_trajec = np.load("cart_trajectory.npy")
joint_via = np.load("joint_trajectory_via.npy")
joint_traj = np.load("move_traj_j_pos.npy")
joint_traj = joint_traj[:,:6]

plotter.plot_trajectory(cart_via, type="cartesian", title="Cartesian space via points")
plotter.plot_trajectory(joint_via, type="joint", title="Joint space via points")
plotter.plot_trajectory(joint_traj, type="joint", title="Joint space final trajectory")
