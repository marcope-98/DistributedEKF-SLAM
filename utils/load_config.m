% Copyright (C) 2023  Marco "marcope-98" Peressutti
% See end of file for extended copyright information

%% load_config
% @param config         user defined configuration structure

% @out profile          struct containg simulation metadata
% @out Robots           cell array containing robots' information

function [profile, Robots] = load_config(config)
    %% Extract variables for readability: this must be provided
    DATASET  = config.Dataset;
    N_ROBOTS = config.nRobots;
    DT       = config.dt;
    
    %% Process simulation config
    % check if dataset number is correct
    if DATASET < 1 || DATASET > 9
        error('[ERROR]: invalid dataset number. config.Dataset must be in the range [1-9]');
    end
    % check if number of robots is correct
    if N_ROBOTS < 1 || N_ROBOTS > 5
        error("[ERROR]: invalid number of robots. config.nRobots must be in the range [1-5]");
    end
    
    % retrieve path to selected dataset
    filename = strcat('assets/MRCLAM/MRCLAM_Dataset', num2str(DATASET));
    % load dataset
    [Barcodes, Landmark_GT, Robots] = loadMRCLAMdataSet(filename, N_ROBOTS);
    % resample dataset
    [Robots, timesteps]             = sampleMRCLAMdataSet(Robots, DT);
    
    %% Generate default simulation settings
    % Simulation start default setting
    if isnan(config.sim.start)
        temp = zeros(N_ROBOTS, 1);
        for i = 1:N_ROBOTS
            temp(i) = min(Robots{i}.M(:,1));
        end
        config.sim.start = round(max(temp) / DT, 0);
        config.sim.start = max(config.sim.start, 1);
    end
    % Simulation end default setting
    if isnan(config.sim.end)
        config.sim.end = timesteps;
    end
    % check if range is valid
    if config.sim.start <= 0         && ...
       config.sim.end   >  timesteps && ...
       config.sim.start >  config.sim.end
        error('[ERROR]: selected simulation interval is not feasibile.');
    end
    
    % Generate default ekf settings
    if isnan(config.ekf.Q)
        config.ekf.Q =  ([0.45, 0; 0, deg2rad(6)]).^2;
%         config.ekf.Q =  ([11.8, 0; 0, 0.18]);
    end
    
    if isnan(config.ekf.A)
         config.ekf.A = [.1, .01, .18, .08];
    end
    
    %% Correct Measurement data to start at config.sim.start
    for i = 1:N_ROBOTS
        cond = round(Robots{i}.M(:,1) / DT, 0) >= config.sim.start;
        Robots{i}.M = Robots{i}.M(cond, :);
    end
    
    %% Apply to profile
    profile = struct;
    profile.ekf            = struct;
    profile.ekf.A          = config.ekf.A;
    profile.ekf.Q          = config.ekf.Q;
    profile.ekf.N          = config.ekf.N;
    
    profile.sim            = struct;
    profile.sim.start      = config.sim.start;
    profile.sim.end        = config.sim.end;
    profile.sim.dt         = DT;
    profile.sim.nRobots    = N_ROBOTS;

    profile.info           = struct;
    profile.info.dataset   = strcat("MRCLAM_Dataset", num2str(config.Dataset));
    profile.info.Landmarks = Landmark_GT;
    profile.info.Barcodes  = Barcodes;
    
end

%%
%     Distributed EKF SLAM with known correspondence    
%     Copyright (C) 2023  Marco "marcope-98" Peressutti
% 
%     This file is part of DSMA-Assignment
% 
%     This program is free software: you can redistribute it and/or modify
%     it under the terms of the GNU General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
% 
%     This program is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU General Public License for more details.
% 
%     You should have received a copy of the GNU General Public License
%     along with this program.  If not, see <https://www.gnu.org/licenses/>.