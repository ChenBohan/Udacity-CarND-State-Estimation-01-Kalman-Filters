def update(mean1, var1, mean2, var2):
    """
    mean1, var1: prior measurement probability
    mean2, var2: new measurement probability
    """
    new_mean = (var2 * mean1 + var1 * mean2) / (var1 + var2)
    new_var = 1 / (1 / var1 + 1  / var2)
    return [new_mean, new_var]

def predict(mean1, var1, mean2, var2):
    """
    mean1: current estimate
    var1: current variance
    mean2: motion
    var2: motion's uncertainty
    """
    new_mean = mean1 + mean2
    new_var = var1 + var2
    return [new_mean, new_var]

# Test data
measurements = [5.0, 6.0, 7.0, 9.0, 10.0]
motions = [1.0, 1.0, 2.0, 1.0, 1.0]
measurement_sig = 4.0
motion_sig = 2.0
mu = 0.0 # initial mu
sig = 1000.0  # initial sig

# KF
for n in range(len(measurements)):
    [mu, sig] = update(mu, sig, measurements[n], measurement_sig)
    print 'update: ', [mu, sig]
    [mu, sig] = predict(mu, sig, motions[n], motion_sig)
    print 'predict: ', [mu, sig]
