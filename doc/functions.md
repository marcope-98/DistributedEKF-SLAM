# Functions Documentation
Table of contents:
- [cvt_rb_to_xy](#cvt_rb_to_xy) 
- [cvt_xy_to_rb](#cvt_xy_to_rb)
- [find_transformation](#find_transformation)
- [load_config](#load_config)


## cvt_rb_to_xy

`xy = cvt_rb_to_xy(rb, pose)`
	
	The utility function `cvt_rb_to_xy` converts the position of a range and bearing measurement (r and b) wrt a pose (x, y, theta) to the corresponding x and y coordinates.

The transformation is performed as follows:

$$
\begin{align*}
x &= range \cdot cos(bearing + \theta_{pose}) + x_{pose}\\
y &= range \cdot sin(bearing + \theta_{pose}) + y_{pose}
\end{align*}
$$

## cvt_xy_to_rb

`rb = cvt_xy_to_rb(xy, pose)`
	
	The utility function `cvt_xy_to_rb` converts the position of a point in the plane (x and y) wrt a pose (x, y, theta) to a range and bearing.


Let

$$
\delta x = x_{xy} - x_{pose} \qquad\qquad \delta y = y_{xy} - y_{pose}
$$

The resulting range and bearing output is computed as follows:

$$
\begin{align*}
range &= \sqrt{\delta x^2 + \delta y^2}\\
bearing &= atan2(\delta y, \delta x) - \theta_{pose}
\end{align*}
$$

## find_transformation

`[R, t] = find_transformation(x1, x2)`

	The find transormation helper functions finds the rotation matrix and translation vector that aligns the 2d points in x1 to the 2d points in x2.

The two input arrays `x1` and `x2` represents the same point in two different coordinate systems.

The resulting rotation matrix `R` and the translation vector `t` between the coordinate system of `x1` and the coordinate system of `x2`.


## load_config

`[profile, Robots] = load_config(config)`

	The `load_config` helper function parses the config struct containing the information for the simulation and returns the interpolated Robots' groundtruth, measurements and controls as well as a struct `profile` containing start and end of the simulation as well as the number of robots.