% Copyright (C) 2023  Marco "marcope-98" Peressutti
% See end of file for extended copyright information

%%
classdef AgentEKF < handle
    properties
        ekf;        % EKF SLAM object
        sensor;     % Sensor   object
        actuator;   % Actuator object
        
        id;         % id of the robot [1 - 5]
        sim;        % Simulation details
        info;       % Robot Ground Truth
        Est;        % Estimated robot pose history
        LandmarkPositionHistory;
        server;     % Communication server
    end
    
    methods
        function obj = AgentEKF(Robot, codeDict, server, params)
            obj.id       = params.id;   % robot id
            obj.sim      = params.sim;  % simulation parameters
            obj.info     = Robot.G;     % ground truth info
            
            state_0 = [0;0;0];
            obj.ekf      = ekfSLAM(state_0, params.ekf);
            obj.sensor   = Sensor(codeDict, Robot.M);
            obj.actuator = Actuator(obj.sim.start, Robot.O);
            
            obj.Est      = -10 * ones(numel(obj.info(:,1)), 4);
            obj.Est(:,1) = obj.info(:,1);
            obj.LandmarkPositionHistory = zeros(2, params.ekf.N, numel(obj.info(:,1)));
            obj.server   = server;
        end
        
        function step(obj)
            obj.Est(obj.actuator.current, 2:4) = obj.get_state()';
            obj.LandmarkPositionHistory(:,:, obj.actuator.current) = obj.get_landmarks();
            obj.LandmarkPositionHistory(:, ~obj.ekf.observed, obj.actuator.current) = NaN;
            % get controls
            [t, u] = obj.actuator.control();
            % prediction step
            obj.ekf.predict(u);
            % get observations
            [measurement, ~] = obj.sensor.sense(t);
            % correction step
            obj.ekf.correct(measurement);
        end
        
        %% Communication
        function consensus(obj)
        end
        
        %% Utils
        function Robot = cvt_to_Robot(obj)
            Robot = struct;
            Robot.O = obj.actuator.Odometry;
            Robot.M = obj.sensor.Measurement;
            Robot.G = obj.info;
            Robot.Est = obj.Est;
            Robot.LHistory = obj.LandmarkPositionHistory;
        end
        
        function state = get_state(obj)
            state = obj.ekf.get_pose();
        end
        
        function landmarks = get_landmarks(obj)
            landmarks = obj.ekf.get_landmarks();
        end
        
    end
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