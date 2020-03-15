import numpy as np
def getPredictionMatrix(K, dt, p_0, v_0, a_0):

    Tp = np.zeros([K, K])
    Tv = np.zeros([K, K])
    Ta = np.zeros([K, K])

    for i in range(K):
        Ta[i, :i] = np.ones([1, i]) * dt
        for j in range(i):
            Tv[i, j] = (i - j + 0.5) * dt ** 2
            Tp[i, j] = ((i - j + 1) * (i - j) / 2 + 1 / 6) * dt ** 3

    Ba = np.ones([K, 1]) * a_0
    Bv = np.ones([K, 1]) * v_0
    Bp = np.ones([K, 1]) * p_0

    for i in range(K):
        Bv[i] = Bv[i] + i * dt * a_0
        Bp[i] = Bp[i] + i * dt * v_0 + i ** 2 / 2 * a_0 * dt ** 2

    return Tp, Tv, Ta, Bp, Bv, Ba