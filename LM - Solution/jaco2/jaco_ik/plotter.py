import matplotlib.pyplot as plt
import jaco_ik.utility as util
import numpy as np

def plot_trajectory(tensor, goal=None, type='cartesian', title='Cartesian Trajectory'):
    fig, axs = plt.subplots(2, 3, figsize=(15, 10))
    if type == 'cartesian':
        labels = ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']
        pos_list = []
        for i in range(tensor.shape[0]):
            pos_list.append(util.mat2pos(tensor[i]))
        tensor = np.array(pos_list)
            
    elif type == 'joint':
        labels = ['theta_1', 'theta_2', 'theta_3', 'theta_4', 'theta_5', 'theta_6']
    else:
        raise ValueError('type must be either cartesian or joint')
    
    colors = ['b', 'g', 'r', 'c', 'm', 'y']
    
    for i in range(tensor.shape[1]):
        axs[i//3][i%3].plot(tensor[:,i], color=colors[i], label=labels[i])
        if goal is not None:
            axs[i//3][i%3].plot([0, tensor.shape[0]], [goal[i], goal[i]], color='k', linestyle='--', label='Goal')
        axs[i//3][i%3].set_xlabel('Step')
        axs[i//3][i%3].set_ylabel(labels[i])
        axs[i//3][i%3].legend()
        fig.suptitle(title)
    plt.show()
    
    