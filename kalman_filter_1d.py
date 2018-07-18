def update(mean1, var1, mean2, var2):
    new_mean = (var2 * mean1 + var1* mean2)
    new_var = 1 / (1 / var1 + 1  / var2)
    return [new_mean, new_var]

def predict(mean1, var1, mean2, var2):
    # mean1: current estimate
    # var1: current variance
    # mean2: motion
    # var2: motion's uncertainty
    new_mean = mean1 + mean2
    new_var = var1 + var2
    return [new_mean, new_var]
