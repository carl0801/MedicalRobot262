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

dk.setup_module()

robot = KinovaJaco2()
random = np.random.default_rng(seed=2)

N = 1000
#q = random.uniform(1, 2*np.pi, size=(6))
q = np.deg2rad(np.array([152.3,  215.8,   87.2,  333.5,   57.7, -870.8]))
pose = dk.fkine(q)
coords = util.mat2pos(pose)

start = time.time()
ik_solution, info = ik.ik_simple(pose)
end = time.time() - start

diff = q - ik_solution
print(f"diff between q and computed ikine q: {diff}")
print(f"diff between cartesian poses: {np.linalg.norm(pose - dk.fkine(ik_solution))}")


xd, yd, yz = 0.1, 0.0, 0.0
pose_delta = np.hstack([np.eye(4,3), np.array([xd, yd, yz,1]).reshape(4,1)])
target = pose.copy()
target[:3,3] = target[:3,3] + np.array([xd, yd, yz])
print(f"q: {q}")
print(f"start pose: \n{pose}")
print(f"target pose: \n{target}")
move_traj, j_vel_traj = ntrp.move_trajectory(q, target, debug=True)

print(f"move_traj shape: {move_traj.shape}")
print("\nMove traj[0]: \n", move_traj[0], "\nmove_traj[50]\n",move_traj[50],"\nmove_traj[-1]: \n", move_traj[-1])
plotter.plot_trajectory(move_traj, type="joint", title="Joint space trajectory")
#cart_traj = np.load("cart_trajectory_via.npy")
#print(cart_traj)
#plotter.plot_trajectory(cart_traj, type="cartesian", title="Loaded cartesian space trajectory")
robot.plot(move_traj)


# draw a line between pose and target



# for i in range(N):
#     q = np.random.uniform(0, 2*np.pi, size=(6))
#     pose = dk.fkine(q)
#     # Output: solution, [E, i, j, search, qlim_err, iter_err]
#     solution, info = ik.ik_simple(pose, slim=1000, ilim=50, tol=1e-7)
#     print("solution joint angles: ", solution)
#     print(f"searches performed: {info[3]}")
#     print(f"iterations on final search: {info[1]}")
#     print(f"Error E (quadratic) on solution: {info[0]}")
#     # print(f"Solutions found that violates joint limits: {info[4]}")
#     # print(f"Number of searches abandoned due to reaching iteration limit: {info[5]}")
#     # print()
#     final_pose = dk.fkine(solution)
#     # print(f"final pose: \n{final_pose}")
#     # print(f"desired pose: \n{pose}")
#     # print()
#     print(f"pose difference: \n{pose - final_pose}")
#     # print(f"pose difference norm: \n{np.linalg.norm(pose - final_pose)}")
#     print(f"final xyzrpy: \n{util.mat2pos(final_pose)}")
#     print(f"desired xyzrpy: \n{util.mat2pos(pose)}")
#     print()

