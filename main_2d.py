import numpy as np

measurements = [1, 2, 3]

x = np.mat([[0.],[0.]])   # initial state (location & velocity)
P = np.mat([[1000., 0.],[0., 1000]]) # initial uncertainty
u = np.mat([[0.],[0.]])   # external motion
F = np.mat([[1., 1.],[0., 1.]])   # next state function
H = np.mat([[1., 0.]])   # measurement function
R = np.mat([[1.]])        # measurement uncertainty
I = np.mat([[1., 0.],[0., 1.]])   # identity matrix

for n in range(len(measurements)):
    Z = np.mat([[measurements[n]]])
    y = Z - (np.dot(H,x))
    S = np.dot(np.dot(H,P),H.T) + R
    K = np.dot(np.dot(P,H.T), S.I)
    x = x + np.dot(K, y)

    P = np.dot(I - np.dot(K, H), P)

    x = np.dot(F, x) + u
    P = np.dot(np.dot(F, P), F.T)
    print 'x = ', x
    print 'P = ', P