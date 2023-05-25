import numpy as np
import math

#########################################################################
###################          Error Functions         ####################
#########################################################################

def rotmat_error(rot_e, rot_d):
    
    # Get the difference rotation matrix between the desired and current rotation matrices
    dist_rot_mat = np.matmul(rot_d, np.transpose(rot_e, axes=(0, 2, 1)))
    
    return dist_rot_mat

def angle_of_rotation(rot_mat):
    # Compute the angle of rotation from the rotation matrix
    angle = np.arccos((np.trace(rot_mat) - 1) / 2)
    
    return angle


def ep_eo_scalars(current_mat, target_mat):
    # current_mat: Nx4x4 tensor of current pose(4x4) for each branch
    rot_e = current_mat[:, :3, :3]
    rot_d = target_mat[:, :3, :3]
    rot_err = rotmat_error(rot_e, rot_d)
    e_o = np.arccos((np.trace(rot_err) - 1) / 2)
    e_p = np.linalg.norm(target_mat[:, :3, 3] - current_mat[:, :3, 3])
    
    return e_p, e_o


def scalar_error(current_mat, target_mat, w_p=1., w_o=1.):
    e_p, e_o = ep_eo_scalars(current_mat, target_mat)
    error = e_p*w_p + e_o*w_o
    return error



def pose_error(Te, Td):
    
    # Extract the translation vectors from the transformation matrices
    pos_e = Te[:, :3, 3]
    pos_d = Td[:, :3, 3]

    # Calculate the position error
    pos_error = pos_d - pos_e
    
    # Extract the rotation matrices from the transformation matrices
    rot_e = Te[:, :3, :3]
    rot_d = Td[:, :3, :3]
    
    # Calculate the rotation error using axis-angle representation
    axis_angle_error = np.zeros((Te.shape[0], 3))
    for i in range(Te.shape[0]):
        rot_error = rot_d[i].dot(rot_e[i].T)
        angle_error = 0.5 * np.arccos((np.trace(rot_error) - 1.0))
        if angle_error > 0.0:
            axis_error = 1.0 / (2.0 * np.sin(angle_error)) * np.array([
                rot_error[2, 1] - rot_error[1, 2],
                rot_error[0, 2] - rot_error[2, 0],
                rot_error[1, 0] - rot_error[0, 1]
            ])
        else:
            axis_error = np.zeros(3)
        axis_angle_error[i] = axis_error
    
    error = np.concatenate((pos_error, axis_angle_error), axis=1).squeeze()
    return error


def angle_axis(Te, Tep):
    e = np.zeros(6)
    li = np.zeros(3)
    
    e[:3] = Tep[:3, 3] - Te[:3, 3]
    
    R = Tep[:3, :3] @ Te[:3, :3].T

    li[0] = R[2, 1] - R[1, 2]
    li[1] = R[0, 2] - R[2, 0]
    li[2] = R[1, 0] - R[0, 1]
    
    li_norm = np.linalg.norm(li)
    R_tr = np.trace(R)
    
    if li_norm < 1e-6:
        if R_tr > 0:
            e[3:] = 0
        else:
            e[3] = np.pi / 2 * (R[0, 0] + 1)
            e[4] = np.pi / 2 * (R[1, 1] + 1)
            e[5] = np.pi / 2 * (R[2, 2] + 1)
    else:
        ang = np.arctan2(li_norm, R_tr - 1)
        e[3:] = ang * li / li_norm
    
    return e

# def angle_axis(Te, Tep, device=np):
#     """Compute angle-axis error between tensors (batched) poses of shape (batch_size, 4, 4)

#     Args:
#         Te (tensor): Current pose
#         Tep (tensor): Desired pose
#         device (module, optional): Either 'np' or 'cp'-- whether to use Numpy or Cupy. Defaults to np.

#     Returns:
#         _type_: _description_
#     """
#     Te = Te.reshape((1, 4, 4))
#     Tep = Tep.reshape((1, 4, 4))
#     # Batched angle axis error
#     batch_size = Te.shape[0]
#     e = device.zeros((batch_size, 6))
#     li = device.zeros((batch_size, 3))
    
#     e[:, :3] = Tep[:, :3, 3] - Te[:, :3, 3]
    
#     R = device.matmul(Tep[:, :3, :3], Te[:, :3, :3].transpose((0, 2, 1)))
    
#     li[:, 0] = R[:, 2, 1] - R[:, 1, 2]
#     li[:, 1] = R[:, 0, 2] - R[:, 2, 0]
#     li[:, 2] = R[:, 1, 0] - R[:, 0, 1]
    
#     li_norm = device.linalg.norm(li, axis=1)
#     R_tr = device.trace(R, axis1=1, axis2=2)
    
#     mask = li_norm < 1e-6
#     mask_gt_0 = R_tr > 0
    
#     e[mask, 3:] = 0
#     e[~mask, 3:] = device.expand_dims(device.arctan2(li_norm[~mask], R_tr[~mask] - 1), axis=1) * li[~mask] / device.expand_dims(li_norm[~mask], axis=1)
    
#     e[mask & mask_gt_0, 3] = device.pi / 2 * (R[mask & mask_gt_0, 0, 0] + 1)
#     e[mask & mask_gt_0, 4] = device.pi / 2 * (R[mask & mask_gt_0, 1, 1] + 1)
#     e[mask & mask_gt_0, 5] = device.pi / 2 * (R[mask & mask_gt_0, 2, 2] + 1)
    
#     return e


def b_angle_axis(Te, Tep, device=np):
    """Compute angle-axis error between tensors (batched) poses of shape (batch_size, 4, 4)

    Args:
        Te (tensor): Current pose
        Tep (tensor): Desired pose
        device (module, optional): Either 'np' or 'cp'-- whether to use Numpy or Cupy. Defaults to np.

    Returns:
        _type_: _description_
    """

    # Batched angle axis error
    batch_size = Te.shape[0]
    e = device.zeros((batch_size, 6))
    li = device.zeros((batch_size, 3))
    
    e[:, :3] = Tep[:, :3, 3] - Te[:, :3, 3]
    
    R = device.matmul(Tep[:, :3, :3], Te[:, :3, :3].transpose((0, 2, 1)))
    
    li[:, 0] = R[:, 2, 1] - R[:, 1, 2]
    li[:, 1] = R[:, 0, 2] - R[:, 2, 0]
    li[:, 2] = R[:, 1, 0] - R[:, 0, 1]
    
    li_norm = device.linalg.norm(li, axis=1)
    R_tr = device.trace(R, axis1=1, axis2=2)
    
    mask = li_norm < 1e-6
    mask_gt_0 = R_tr > 0
    
    e[mask, 3:] = 0
    e[~mask, 3:] = device.expand_dims(device.arctan2(li_norm[~mask], R_tr[~mask] - 1), axis=1) * li[~mask] / device.expand_dims(li_norm[~mask], axis=1)
    
    e[mask & mask_gt_0, 3] = device.pi / 2 * (R[mask & mask_gt_0, 0, 0] + 1)
    e[mask & mask_gt_0, 4] = device.pi / 2 * (R[mask & mask_gt_0, 1, 1] + 1)
    e[mask & mask_gt_0, 5] = device.pi / 2 * (R[mask & mask_gt_0, 2, 2] + 1)
    
    return e