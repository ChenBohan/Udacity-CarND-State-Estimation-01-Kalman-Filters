# Robotics-Sensor-Fusion-01-Kalman-Filters
Udacity Self-Driving Car Engineer Nanodegree: Kalman Filters

## Content of this repository
- `kalman_filter_1d.py` functions for 1d kalman filter.
- `main_1d.py` a 1d kalman filter example.
- `kalman_filter_md.py` functions for multi-dimension kalman filter.
- `main_md.py` a multi-dimension kalman filter example.
- `matrix.py` matrix lib used in `kalman_filter_md.py`.

## Basic concept

### Kalman filter
- a technique for **estimating the state** of a system
- estimate a **continue** state
    - (monte carlo localization -> **discrete** state)
    - (particle filter -> **continue** state)
- result: a **uni-model distribution**
    - (monte carlo localization -> **multi-model distribution**)
    - (particle filter -> **multi-model distribution** state)
- two main cycles:
    - **Measurement Update** (bayes rule, multiplication)
    - **Motion Update, prediction** (total probability, addition)
    
<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-01-Kalman-Filters/blob/master/readme_img/flow.png" width = "50%" height = "50%" div align=center />

### Gaussian distribution
- continue function
- two parameters:
    - mean: μ 
    - variance: σ^2
- <img src="https://gss3.bdstatic.com/7Po3dSag_xI4khGkpoWK1HF6hhy/baike/s%3D205/sign=2abf505a42166d223c77129473220945/342ac65c1038534384b650b09213b07eca808822.jpg" width = "20%" height = "20%" div align=center />
```python
def gussian(mu, sigma2, x)
    return 1 / sqrt(2.0 * math.pi * sigma2) * exp(-0.5 * (x - mu)**2 / sigma2)
```

### Ref:

[How a Kalman filter works, in pictures.](http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/)


## 1D Kalman Filter

### Measurement Update

Parameter Update of two gaussian

<img src="https://github.com/ChenBohan/Auto-Car-Perception-01-Kalman-Filters/blob/master/readme_img

- ``mean1`` ``var1`` prior measurement probability
- ``mean2`` ``var2`` new measurement probability

```python
def update(mean1, var1, mean2, var2):
    new_mean = (var2 * mean1 + var1* mean2) / (var1 + var2)
    new_var = 1 / (1 / var1 + 1  / var2)
    return [new_mean, new_var]
```

### Motion Update, Prediction

<img src="https://github.com/ChenBohan/Auto-Car-Perception-01-Kalman-Filters/blob/master/readme_img/gaussian_motion.png" width = "50%" height = "50%" div align=center />

- ``mean1`` current estimate
- ``var1`` current variance
- ``mean2`` motion
- ``var2`` motion's uncertainty

```python
def predict(mean1, var1, mean2, var2):
    new_mean = mean1 + mean2
    new_var = var1 + var2
    return [new_mean, new_var]
```

### Kalman Filter

```python
for n in range(len(measurements)):
    [mu, sig] = update(mu, sig, measurements[n], measurement_sig)
    print 'update: ', [mu, sig]
    [mu, sig] = predict(mu, sig, motions[n], motion_sig)
    print 'predict: ', [mu, sig]
```

### Test data & result

```
measurements = [5.0, 6.0, 7.0, 9.0, 10.0]
motions = [1.0, 1.0, 2.0, 1.0, 1.0]
measurement_sig = 4.0
motion_sig = 2.0
mu = 0.0 # initial mu
sig = 1000.0  # initial sig
```

```
update:  [4.9800796812749, 3.9840637450199203]
predict:  [5.9800796812749, 5.98406374501992]
update:  [5.992019154030327, 2.3974461292897047]
predict:  [6.992019154030327, 4.397446129289705]
update:  [6.996198441360958, 2.094658810112146]
predict:  [8.996198441360958, 4.094658810112146]
update:  [8.99812144836331, 2.0233879678767672]
predict:  [9.99812144836331, 4.023387967876767]
update:  [9.99906346214631, 2.0058299481392163]
predict:  [10.99906346214631, 4.005829948139216]

```

## Multi-dimension Kalman Filter

### Update

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-01-Kalman-Filters/blob/master/readme_img/update2.png" width = "30%" height = "30%" div align=center />

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-01-Kalman-Filters/blob/master/readme_img/update.png" width = "30%" height = "30%" div align=center />

- ``Z`` measurement matrix of nth measurement.
- ``y`` compare the prediction with sensor measurement.
- ``H`` measurement function.
- ``K`` Kalman filter gain, combines the uncertainty of the prediction with the uncertainty of the sensor measurement. 

```python
Z = m.matrix([[measurements[n]]])   
y = Z - (H * x)                     
S = H * P * H.transpose() + R
K = P * H.transpose() * S.inverse() 
x = x + K * y                       
P = (I - (K * H)) * P
```

### Predict
Predicted (a prior) state estimate:

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-01-Kalman-Filters/blob/master/readme_img/predict.png" width = "30%" height = "30%" div align=center />

- ``x`` initial state (location & velocity)
- ``F`` next state function
- ``u`` external motion

```python
x = (F * x) + u
```

- ``P`` initial uncertainty
```python
P = F * P * F.transpose()
```

PS: x′=Fx+Bu+ν. But then Bu was crossed out, B is a matrix called the control input matrix and uu is the control vector.

(For example, if we were in an autonomous vehicle tracking a bicycle, pedestrian or another car, we would not be able to model the internal forces of the other object; hence, we do not know for certain what the other object's acceleration is. Instead, we will set Bu = 0 and represent acceleration as a random noise with mean)
