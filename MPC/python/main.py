import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from pylab import *
from mpc_solver import mpc_solver
from get_ReferenceTrajectory import get_ReferenceTrajectory

def main():

    px_0 = 0
    py_0 = 8
    pz_0 = 20
    step_n = 2000
    K = 20
    a = 0.08
    h = 20
    r = 10

    x, y, z = get_ReferenceTrajectory(K,a,h,r,step_n)

    lim_v_xy = [-6, 6]
    lim_v_z = [-1, 6]
    lim_a_xy = [-3, 3]
    lim_a_z = [-1, 3]
    lim_j_xy = [-3, 3]
    lim_j_z = [-2, 2]


    log_px, log_vx, log_ax, log_jx = mpc_solver(px_0, lim_v_xy[1], lim_v_xy[0], lim_a_xy[1],
                                            lim_a_xy[0], lim_j_xy[1], lim_j_xy[0], x, step_n)
    log_py, log_vy, log_ay, log_jy = mpc_solver(py_0, lim_v_xy[1], lim_v_xy[0], lim_a_xy[1],
                                            lim_a_xy[0], lim_j_xy[1], lim_j_xy[0], y, step_n)
    log_pz, log_vz, log_az, log_jz = mpc_solver(pz_0, lim_v_z[1], lim_v_z[0], lim_a_z[1],
                                            lim_a_z[0], lim_j_z[1], lim_j_z[0], z, step_n)


    fig1 = plt.figure()
    ax = fig1.gca(projection='3d')
    ax.plot(x, y, z, ':', linewidth=4, color='green', label="reference trajectory")
    ax.plot(log_px, log_py, log_pz, '-', linewidth=2, color='red', label="real trajectory")
    ax.legend()

    # fig2 = plt.figure()
    fig2 = plt.figure(figsize=(10,8))
    ax1 = fig2.add_subplot(221)  # top left
    ax2 = fig2.add_subplot(222)  # top right
    ax3 = fig2.add_subplot(223)  # bottom left
    ax4 = fig2.add_subplot(224)
    # fig2, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)
    # fig2.suptitle('state')

    t = np.linspace(0,len(log_vx),num=len(log_vx))
    ax1.plot(t, log_vx, label="Vx")
    ax1.plot(t, log_vy, '-.', label="Vy")
    ax1.plot(t, log_vz, ':', label="Vz")
    ax1.set_title('Velocity')
    ax1.legend()

    ax2.plot(t, log_ax, label="Ax")
    ax2.plot(t, log_ay, '-.', label="Ay")
    ax2.plot(t, log_az, ':', label="Az")
    ax2.set_title('Acceleration')
    ax2.legend()

    ax3.plot(t, log_jx, label="Jx")
    ax3.plot(t, log_jy, '-.', label="Jy")
    ax3.plot(t, log_jz, ':', label="Jz")
    ax3.set_title('Jerk')
    ax3.legend()

    ax4.plot(t, log_px-np.asarray(x[:len(log_vx)]), label="Error of Px")
    ax4.plot(t, log_py-np.asarray(y[:len(log_vx)]), '-.', label="Error of Py")
    ax4.plot(t, log_pz-np.asarray(z[:len(log_vx)]), ':', label="Error of Pz")
    ax4.set_title('Error of Position')
    ax4.legend()

    plt.tight_layout()


    fig1.savefig("Trajectory.png")
    fig2.savefig("State.png")
    plt.show()


if __name__ == '__main__':
    main()
