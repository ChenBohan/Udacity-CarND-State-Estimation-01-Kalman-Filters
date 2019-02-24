import matrix as m

measurements = [1, 2, 3]

x = m.matrix([[0.],[0.]])   # initial state (location & velocity)
P = m.matrix([[1000., 0.],[0., 1000]]) # covariance matrix (initial uncertainty)
u = m.matrix([[0.],[0.]])   # external motion
F = m.matrix([[1., 1.],[0., 1.]])   # transition (next state) function
H = m.matrix([[1., 0.]])   # measurement function
R = m.matrix([[1.]])        # measurement uncertainty
I = m.matrix([[1., 0.],[0., 1.]])   # identity matrix

def kalman_filter(x, P, measurements):

    for n in range(len(measurements)):

        # measurement
        Z = m.matrix([[measurements[n]]])   # measurement matrix of nth measurement
        y = Z - (H * x)                     # arrow calculation
        S = H * P * H.transpose() + R
        K = P * H.transpose() * S.inverse() # kalman gain
        x = x + K * y                       # next prediction
        P = (I - (K * H)) * P               # measurement update

        # prediction
        x = (F * x) + u
        P = F * P * F.transpose()

        print 'x = '
        x.show()
        print 'P = '
        P.show()

    return x, P

kalman_filter(x, P, measurements)