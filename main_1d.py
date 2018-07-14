import kalman_filter_1d as kf_1d

measurements = [5., 6., 7., 9., 10.]
motion = [1., 1., 2., 1., 1.]
measurement_sig = 4.
motion_sig = 2.
mu =0
sig = 10000.

for n in range(len(measurements)):
    [mu, sig] = kf_1d.update(mu, sig, measurements[n], measurement_sig)
    print 'update:',[mu, sig]
    [mu, sig] = kf_1d.predict(mu, sig, motion[n], motion_sig)
    print 'predict:',[mu, sig]