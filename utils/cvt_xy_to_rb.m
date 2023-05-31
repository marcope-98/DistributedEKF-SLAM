%% Utility: conversion from x-y coordinates to range-bearing
% @param  xy    vector (2x1) containing x and y coordinates of the
%               landmark
% @param  pose  vector (3x1) containing the pose in the form [x y theta]
%
% @out    rb    vector (2x1) containing range and bearing wrt to the
%               current pose
function rb = cvt_xy_to_rb(xy, pose)
    dx = xy(1) - pose(1);
    dy = xy(2) - pose(2);
            
    rb = [sqrt(dx^2 + dy^2);
          wrapToPi(atan2(dy, dx) - pose(3))];
end