clc;
clear;
close all;

% flags
use_distributed = true;

% initialize simulation parameters
init;

% waitbar to track progress
total = profile.sim.end - profile.sim.start;
f = waitbar(0, "Simulating C-SLAM");
%% SLAM + consensus
for i = profile.sim.start : profile.sim.end
    if rem(i,10000) == 0
        waitbar((i - profile.sim.start) / total,f);
    end
    
    % perform ekf slam prediction and correction step + send information
    for j = 1:profile.sim.nRobots
        Agents(j).step();
    end
    
    % fetch information and merge local maps using consensus
    for j = 1:profile.sim.nRobots
        Agents(j).consensus();
    end

    % flush the server (deletes all messages in the inbox)
    server.flush();
end
close(f);

%% Simulation result
% convert Agents state for animation
Robots = cell([profile.sim.nRobots,1]);
for j = 1:profile.sim.nRobots
    Robots{j} = Agents(j).cvt_to_Robot();
end

%% Animate the simulation results
display_simulation(Robots, profile);

%% Plotting results
pose_error_plot(Robots, profile);
landmark_position_error_plot(Robots, profile);



clear i j total f server