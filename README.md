# Auto-Car-Perception-01-Kalman-Filters
Udacity Self-Driving Car Engineer Nanodegree: Kalman Filters
## Content of this repo
- `kalman_filter_1d.py` functions for 1d kalman filter.
- `main_1d.py` a 1d kalman filter example.
- `kalman_filter_md.py` functions for multi-dimension kalman filter.
- `main_md.py` a multi-dimension kalman filter example.
- `matrix.py` matrix lib used in `kalman_filter_md.py`.

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
x = (F * x) + u
```

```python
# P is initial uncertainty
P = F * P * F.transpose()
```
