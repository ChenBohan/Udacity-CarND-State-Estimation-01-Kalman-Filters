# Robotics-Sensor-Fusion-01-Kalman-Filters
Udacity Self-Driving Car Engineer Nanodegree: Kalman Filters

## Basic concept

### Kalman filter
- a technique for **estimating the state** of a system
- estimate a **continue** state
    - (monte carlo localization -> **discrete** state)
    - (particle filter -> **continue** state)
- result: a **uni-model distribution**
    - (monte carlo localization -> **multi-model distribution**)
    - (particle filter -> **multi-model distribution** state)

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
<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-01-Kalman-Filters/blob/master/readme_img/flow.png" width = "50%" height = "50%" div align=center />

Ref:

[How a Kalman filter works, in pictures.](http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/)

## Content of this repository
- `kalman_filter_1d.py` functions for 1d kalman filter.
- `main_1d.py` a 1d kalman filter example.
- `kalman_filter_md.py` functions for multi-dimension kalman filter.
- `main_md.py` a multi-dimension kalman filter example.
- `matrix.py` matrix lib used in `kalman_filter_md.py`.


## 1D Kalman Filter

### Update

- ``mean1`` ``var1`` prior measurement probability
- ``mean1`` ``var1`` new measurement probability
```python
def update(mean1, var1, mean2, var2):
    new_mean = (var2 * mean1 + var1* mean2)
    new_var = 1 / (1 / var1 + 1  / var2)
    return [new_mean, new_var]
```
<img src="https://github.com/ChenBohan/Auto-Car-Perception-01-Kalman-Filters/blob/master/readme_img/gaussian_motion.png" width = "50%" height = "50%" div align=center />

### Predict

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
<img src="https://github.com/ChenBohan/Auto-Car-Perception-01-Kalman-Filters/blob/master/readme_img/measurement_update.png" width = "50%" height = "50%" div align=center />

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
