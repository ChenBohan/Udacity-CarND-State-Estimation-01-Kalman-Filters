import kalman_filter_md
import matrix as m

measurements = [1, 2, 3]
x = m.matrix([[0.],[0.]])   # initial state (location & velocity)
P = m.matrix([[1000., 0.],[0., 1000]]) # initial uncertainty

kalman_filter_md.kalman_filter(x, P, measurements)
