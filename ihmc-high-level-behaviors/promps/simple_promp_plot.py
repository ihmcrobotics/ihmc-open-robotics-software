import numpy as np
import math
import sys
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, PathPatch
import mpl_toolkits.mplot3d.art3d as art3d

def covariance_to_ellipse(cov):
    a = cov[0,0]
    b = cov[0,1]
    c = cov[1,1]

    w, V = np.linalg.eig(cov)

    idx = 0 if w[0] > w[1] else 1
    idx = 1

    V[:, idx] = V[:, idx] * w[idx]
    V[:, 1-idx] = V[:, 1-idx] * w[1-idx]

    l1 = np.linalg.norm(V[:, 1-idx])
    l2 = np.linalg.norm(V[:, idx])
    angle = math.atan2(V[1, 1-idx], V[0, 1-idx])

    return (l1, l2, angle)

#----------------SIMPLE PROMP------------------------------------------

trajectories = np.genfromtxt('../build/trajectories.csv', delimiter=',', dtype=float)
generated = np.genfromtxt('../build/generated.csv', delimiter=',', dtype=float)
variance = np.genfromtxt('../build/variance.csv', delimiter=',', dtype=float)
# covariance = np.genfromtxt('../build/covariance.csv', delimiter=',', dtype=float)

generated_c = np.genfromtxt('../build/generated_conditioned.csv', delimiter=',', dtype=float)
variance_c = np.genfromtxt('../build/variance_conditioned.csv', delimiter=',', dtype=float)
# covariance_c = np.genfromtxt('../build/covariance_conditioned.csv', delimiter=',', dtype=float)

dims = 1 if len(generated.shape) == 1 else generated.shape[1]
motions = int(trajectories.shape[1] / dims)
timesteps = generated.shape[0]

if dims == 1:
    generated = np.expand_dims(generated, axis=1)
    variance = np.expand_dims(variance, axis=1)
    generated_c = np.expand_dims(generated_c, axis=1)
    variance_c = np.expand_dims(variance_c, axis=1)

# plot 2d of trajectori and covariance
if dims == 2:

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    z_idxs = list(range(timesteps))
    for m in range(motions):
        ax.plot(trajectories[:,m*dims + 0], trajectories[:,m*dims + 1], zs=z_idxs, zdir='x')
        for t in range(timesteps):
            if t % 5 == 0:
                (w, h, a) = covariance_to_ellipse(covariance[2*t:2*t+2, :])
                ellipse = Ellipse(generated[t, 0:2], 2*math.sqrt(w), 2*math.sqrt(h), a, fc=(0.2,0.6,0.5,0.011))
                ax.add_patch(ellipse)
                art3d.pathpatch_2d_to_3d(ellipse, z=t, zdir="x")
    ax.plot(generated[:,0], generated[:,1], zs=z_idxs, zdir='x')

    plt.grid()
    plt.show()
    plt.savefig('../build/promp_with_cov_d12.png')
    plt.close()

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    z_idxs = list(range(timesteps))
    for m in range(motions):
        ax.plot(trajectories[:,m*dims + 0], trajectories[:,m*dims + 1], zs=z_idxs, zdir='x')
        for t in range(timesteps):
            if t % 5 == 0:
                (w, h, a) = covariance_to_ellipse(covariance_c[2*t:2*t+2, :])
                print("angle", a)
                ellipse = Ellipse(generated_c[t, 0:2], 2*math.sqrt(w), 2*math.sqrt(h), a, fc=(0.2,0.6,0.5,0.011))
                ax.add_patch(ellipse)
                art3d.pathpatch_2d_to_3d(ellipse, z=t, zdir="x")
    ax.plot(generated_c[:,0], generated_c[:,1], zs=z_idxs, zdir='x')

    plt.grid()
    plt.show()
    plt.savefig('../build/promp_with_cov_d12_conditioned.png')
    plt.close()

for i in range(dims):
    
    plt.figure()
    for m in range(motions):
        plt.plot(trajectories[:,m*dims + i], 'black')
    plt.plot(generated[:,i], "green")
    plt.grid()

    lower_bound = generated[:,i] - variance[:,i]
    upper_bound = generated[:,i] + variance[:,i]

    plt.fill_between(np.linspace(0,timesteps-1, timesteps), lower_bound, upper_bound, color='green', alpha=0.2)
    plt.savefig('../build/promp_'+str(i)+'.png')
    plt.close()
    
    # conditioned
    plt.figure()
    for m in range(motions):
        plt.plot(trajectories[:,m*dims + i], 'black')
    plt.plot(generated_c[:,i], "green")
    plt.grid()
    
    lower_bound = generated_c[:,i] - variance_c[:,i]
    upper_bound = generated_c[:,i] + variance_c[:,i]

    plt.fill_between(np.linspace(0,timesteps-1, timesteps), lower_bound, upper_bound, color='green', alpha=0.2)
    plt.savefig('../build/promp_conditioned_'+str(i)+'.png')
    plt.close()