import scipy.linalg
import numpy as np
import scipy.sparse as sparse
import osqp
from get_PredictionMatrix import getPredictionMatrix

def mpc_solver(p_0, max_v, min_v, max_a, min_a, max_j, min_j, Target_p, step_n):
    v_0 = 0
    a_0 = 0
    j_0 = 0
    K = 20
    dt = 0.2
    w1 = 10
    w2 = 1
    w3 = 1
    w4 = 5
    w5 = 1e8
    count = 0
    log_p = []
    log_v = []
    log_a = []
    log_j = []
    conc_with_identity = lambda x, y: np.concatenate([x, y * np.ones([K, K])], axis=1)
    for i in range(step_n-1):
        Tp, Tv, Ta, Bp, Bv, Ba = getPredictionMatrix(K, dt, p_0, v_0, a_0)

        Target_p_K = np.asarray(Target_p[count:count + K]).reshape(1, -1)

        H = scipy.linalg.block_diag(w4 * np.ones([K, K]) + w1 * (np.matmul(Tp.transpose(), Tp)),
                                    w5 * np.ones([K, K]))

        F = np.concatenate([2 * w1 * (np.matmul(Bp.transpose(), Tp) - np.matmul(Target_p_K, Tp)),
                            np.zeros([1, K])], axis=1)

        A = np.concatenate([conc_with_identity(Tv, 0),
                            conc_with_identity(-Tv, -1),
                            conc_with_identity(Ta, 0),
                            conc_with_identity(-Ta, -1),
                            conc_with_identity(np.ones([K, K]), 0),
                            conc_with_identity(-np.ones([K, K]), -1),
                            conc_with_identity(np.zeros_like(Ta), -1)], axis=0)

        b = np.concatenate([np.ones([K, 1]) * max_v - Bv,
                            -np.ones([K, 1]) * min_v + Bv,
                            np.ones([K, 1]) * max_a - Ba,
                            -np.ones([K, 1]) * min_a + Ba,
                            np.ones([K, 1]) * max_j,
                            -np.ones([K, 1]) * min_j,
                            np.zeros([K, 1])], axis=0)

        P = sparse.csc_matrix(H)
        q = np.array(F.T)
        A_ = sparse.csc_matrix(A)
        l = np.array(-np.inf * np.ones_like(b))
        # l = None
        u = np.array(b)

        # Create an OSQP object
        prob = osqp.OSQP()

        # Setup workspace and change alpha parameter
        prob.setup(P, q, A_, l, u)

        # Solve problem
        res = prob.solve()

        j = res.x[0]

        p_0 = p_0 + v_0 * dt + 0.5 * a_0 * dt ** 2 + 1 / 6 * j * dt ** 3
        v_0 = v_0 + a_0 * dt + 0.5 * j * dt ** 2
        a_0 = a_0 + j * dt
        j_0 = j

        log_p.append(p_0)
        log_v.append(v_0)
        log_a.append(a_0)
        log_j.append(j_0)
        count += 1

    return log_p, log_v, log_a, log_j