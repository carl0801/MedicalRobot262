import numpy as np
import jaco_ik.daniel_kine as dk
import jaco_ik.error_functions as ef
import jaco_ik.utility as util
dk.setup_module()


def step(q0, Td, We, k):
    # Implementation of LM algorithm from paper by Corke and Haviland:
    # See: https://arxiv.org/pdf/2207.01796.pdf
    # page numbers and equations refer to this paper.
    
    Te = dk.fkine(q0)
    
    e = ef.angle_axis(Te, Td)
    # quadratic error E:
    E = (0.5 * e) @ We @ e # p. 9, equation (44)

    # Setting damping matrix using Chan's method
    Wn = k * E * np.eye(q0.shape[-1]) # p. 10, equation (50)

    J = dk.jacob0(q0)
    g = J.T @ We @ e # p. 9, equation (47)
    
    q0 += np.linalg.inv(J.T @ We @ J + Wn) @ g # p. 9, equation (48) and (49)

    return q0, E


## todo - add input for how many batches to run? Or just base it on the size of the input?
def ik_simple(
        Td,             # 4x4 tensor describing the desired end-effector pose.
        q = None,       # matrix of joint angles, each row corresponds to a pose. If None, random values are used.
        ilim = 30,      # maximum number of iterations
        slim = 100,     # maximum number of searches
        tol = 1e-7,     # tolerance on error norm
        qlim = [1],     # joint limits
        we=1.0,         # weight for orientation error
        k=0.1,          # gain for Chan's method (0.1 recommended by P.Corke and default)
        num_j = 6,      # number of joints
        dev = np        # module to use, np or cp. np (NumPy) for cpu, cp (CuPy) for gpu
        ):

    
    if qlim[0] != 1:
        qlim = qlim
    else:
        qlim = dev.deg2rad(dev.array([[0, 50, 19, 0, 0, 0], [720, 310, 341, 720, 720, 720]])) # default Jaco2 limits

    # Set we. if not given: default is identity matrix (weight of 1)
    We = dev.eye(num_j) * we


    if q is None:
        q = dev.random.uniform(qlim[0], qlim[1], [slim, num_j])   
    else:
        q0 = np.resize(q.copy(), [5, 6]) #dev.random.uniform(qlim[0], qlim[1], num_j)
        rand_q = dev.random.uniform(qlim[0], qlim[1], [slim-5, num_j])
        q  = dev.vstack([q0, rand_q])
        
    qlim_err = 0 # number of solutions rejected due to joint limit violation
    iter_err = 0 # number of solutions rejected due to iteration limit
    
    i     = 0   # iteration counter of individual searches
    search = 0  # number of current search (index of q and stop condition for while loop)

    # Initialise variables
    e = 0.0

    for j in range(slim):
        q0 = q[j]   # initial joint angles for this search
        i = 0       # reset iteration counter
        search += 1

        while i < ilim:
            i += 1
            # take a step
            q0, e = step(q0, Td, We, k)
            # check if goal is reached
            if e <= tol:
                # scale q to be within 360 degrees
                q0 = q0  % (2 * np.pi)

                # Check if we have violated joint limits
                qlim_check = util.check_qlim(q0, qlim)

                if not qlim_check:
                    # joint limits are violated, so solution is not valid
                    qlim_err += 1
                    break
                else:
                    return q0, [e, i, j, search, qlim_err, iter_err]
        

    # If nothing has been returned, the search has failed
    raise Exception("Failed to converge, search limit and iteration limit reached.")
    


    



    
