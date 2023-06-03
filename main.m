clc;
clear;
close all;
init;
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
%         Agents(j).fetch();
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

%% Find optimal transformation
x_GT = profile.info.Landmarks(:,2:3)';
for j = 1:profile.sim.nRobots
    x_EST = Agents(j).get_landmarks();
    [R,t] = find_transformation(x_EST, x_GT);
    th = atan2(R(2,1), R(1,1));    
    Robots{j}.Est(:,2:3) = (R * (Robots{j}.Est(:,2:3)') + t)';
    Robots{j}.Est(:,4) = wrapToPi(Robots{j}.Est(:,4) + th);
end


%% 
% animate the simulation results
animateMRCLAMdataSet(Robots, profile.info.Landmarks, profile.sim.end, profile.sim.dt)

clear i j total f th R t x_EST x_GT