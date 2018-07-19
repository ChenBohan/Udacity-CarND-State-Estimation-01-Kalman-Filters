# Auto-Car-Perception-01-Kalman-Filters
Udacity Self-Driving Car Engineer Nanodegree: Kalman Filters
## Content of this repository
- `kalman_filter_1d.py` functions for 1d kalman filter.
- `main_1d.py` a 1d kalman filter example.
- `kalman_filter_md.py` functions for multi-dimension kalman filter.
- `main_md.py` a multi-dimension kalman filter example.
- `matrix.py` matrix lib used in `kalman_filter_md.py`.

## Kalman Filter
Kalman filtering is an algorithm that uses a series of measurements observed over time, containing statistical noise and other inaccuracies, and produces estimates of unknown variables that tend to be more accurate than those based on a single measurement alone, by estimating a joint probability distribution over the variables for each timeframe.

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-01-Kalman-Filters/blob/master/readme_img/flow.png" width = "50%" height = "50%" div align=center />

Ref:

[How a Kalman filter works, in pictures.](http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/)

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

- ``Z`` measurement matrix of nth measurement
- ``y`` arrow calculation
- ``H`` measurement function
- ``K`` kalman gain

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
