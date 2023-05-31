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
        Agents(j).fetch();
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
x_GT = profile.info.Landmarks(:,2:3);
mu_GT = mean(profile.info.Landmarks(:,2:3));
x_GT = x_GT - mu_GT;
for j = 1:profile.sim.nRobots
    x_EST = Agents(j).get_landmarks()';
    mu_EST = mean(x_EST);
    x_EST = x_EST - mu_EST;
    W = [0,0;0,0];
    for i = 1:15
        W = W + x_EST(i,:)' * x_GT(i,:);
    end
    [U, S, V] = svd(W);
    R = V * U';
    t = mu_GT' - R * mu_EST';
    for i = 1:numel(Robots{j}.Est(:,1))
        T = (R * Robots{j}.Est(i, 2:3)') + t;
        Robots{j}.Est(i, 2:3) = T';
    end
end


%%

% animate the simulation results
animateMRCLAMdataSet(Robots, profile.info.Landmarks, profile.sim.end, profile.sim.dt)

clear i j total f