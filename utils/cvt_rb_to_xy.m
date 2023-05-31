%% Utility: conversion from range-bearing to x-y coordinates
% @param rb    vector (2x1) containing range and bearing wrt to the
%              current pose
% @param pose  vector (3x1) containing the pose in the form [x y theta]
%
% @out   xy    vector (2x1) containing x and y coordinates of the
%              landmark
function xy = cvt_rb_to_xy(rb, pose)
    ranges = rb(1,:);
    angles = rb(2,:);
    xy = [0; 0];
    xy(1) = ranges .* cos(angles + pose(3)) + pose(1);
    xy(2) = ranges .* sin(angles + pose(3)) + pose(2);
end