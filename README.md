# Auto-Car-Perception-01-Kalman-Filters
Udacity Self-Driving Car Engineer Nanodegree: Kalman Filters
## Kalman Filter
Kalman filtering is an algorithm that uses a series of measurements observed over time, containing statistical noise and other inaccuracies, and produces estimates of unknown variables that tend to be more accurate than those based on a single measurement alone, by estimating a joint probability distribution over the variables for each timeframe.
## Implementation
<img src="https://github.com/ChenBohan/Auto-Car-Perception-01-Kalman-Filters/blob/master/readme_img/formula.jpg" width = "70%" height = "70%" div align=center />

### Predict
Predicted (a priori) state estimate:
```python
# x is initial state (location & velocity)
# F is next state function
# u is external motion
x = np.dot(F, x) + u
```

```python
# P is initial uncertainty
P = np.dot(np.dot(F, P), F.T)
```
