# CarND-10-Sensor-Fusion-Kalman-Filters
Udacity Self-Driving Car Engineer Nanodegree: Kalman Filters

## Content of this repository
- `kalman_filter_1d.py` 1D kalman filter with python.
- `kalman_filter_1d.cpp` 1D kalman filter with c++.
- `kalman_filter_md.py` multi-dimension kalman filter.
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

[Chinese version](https://blog.csdn.net/luoshi006/article/details/52134323)

[Noise in KF](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/3612b91d-9c33-47ad-8067-a572a6c93837/concepts/df73ce8f-f476-49ed-a7d0-b67a657616fa)


## 1D Kalman Filter

### Measurement Update

Parameter Update of two gaussian

<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-01-Kalman-Filters/blob/master/readme_img/measurement_update.png" width = "50%" height = "50%" div align=center />

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

### Result

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

## Kalman Filter Design

- state
    - state transition function
- measurement
    - measurement function
    

<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-01-Kalman-Filters/blob/master/readme_img/Kalman%20Filter%20Design.png" width = "60%" height = "60%" div align=center />
    
<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-01-Kalman-Filters/blob/master/readme_img/kf_formula.png" width = "60%" height = "60%" div align=center />

## Multi-dimension Kalman Filter

<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-01-Kalman-Filters/blob/master/readme_img/final_formula.png" width = "80%" height = "80%" div align=center />

### Measurement Update

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-01-Kalman-Filters/blob/master/readme_img/update2.png" width = "30%" height = "30%" div align=center />

<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-01-Kalman-Filters/blob/master/readme_img/Combining%20Gaussians.png" width = "80%" height = "80%" div align=center />

<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-01-Kalman-Filters/blob/master/readme_img/put_it_togeteher" width = "80%" height = "80%" div align=center />

- ``Z`` measurement matrix of nth measurement.
- ``y`` compare the prediction with sensor measurement.
- ``H`` measurement function.
- ``K`` Kalman filter gain, combines the uncertainty of the prediction with the uncertainty of the sensor measurement. 

```python
Z = m.matrix([[measurements[n]]])   # measurement matrix of nth measurement
y = Z - (H * x)                     # arrow calculation
S = H * P * H.transpose() + R
K = P * H.transpose() * S.inverse() # kalman gain
x = x + K * y                       # next prediction
P = (I - (K * H)) * P               # measurement update
```

The ``K`` matrix, often called the **Kalman filter gain**, combines the uncertainty of where we think we are ``P'`` with the uncertainty of our sensor measurement ``R``. If our sensor measurements are very uncertain (``R`` is high relative to ``P'``), then the Kalman filter will give more weight to where we think we are: x′x'x′. If where we think we are is uncertain (``P'`` is high relative to ``R``), the Kalman filter will put more weight on the sensor measurement: ``z``. 

### Predict

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-01-Kalman-Filters/blob/master/readme_img/predict.png" width = "30%" height = "30%" div align=center />

- ``x`` initial state (location & velocity)
- ``F`` next state function
- ``u`` external motion
- ``P`` initial uncertainty

```python
x = (F * x) + u
P = F * P * F.transpose()
```

### Result

run the filter with these 3 measurement, we can estimate the velocity

<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-01-Kalman-Filters/blob/master/readme_img/predict_unmeasurement.png" width = "60%" height = "60%" div align=center />

```python
measurements = [1, 2, 3]

x = m.matrix([[0.],[0.]])   # initial state (location & velocity)
P = m.matrix([[1000., 0.],[0., 1000]]) # initial uncertainty
u = m.matrix([[0.],[0.]])   # external motion
F = m.matrix([[1., 1.],[0., 1.]])   # next state function
H = m.matrix([[1., 0.]])   # measurement function
R = m.matrix([[1.]])        # measurement uncertainty
I = m.matrix([[1., 0.],[0., 1.]])   # identity matrix
```

```
x = 
[0.9990009990009988]
[0.0]
 
P = 
[1000.9990009990012, 1000.0]
[1000.0, 1000.0]
 
x = 
[2.998002993017953]
[0.9990019950129659]
 
P = 
[4.990024935169789, 2.9930179531228447]
[2.9930179531228305, 1.9950129660888933]
 
x = 
[3.9996664447958645]
[0.9999998335552873]
 
P = 
[2.3318904241194827, 0.9991676099921091]
[0.9991676099921067, 0.49950058263974184]

```
