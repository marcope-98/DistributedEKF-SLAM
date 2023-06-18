% Copyright (C) 2023  Marco "marcope-98" Peressutti
% See end of file for extended copyright information

%%
classdef AgentDEKF < handle
    properties
        ekf;        % EKF SLAM object
        sensor;     % Sensor   object
        actuator;   % Actuator object
        
        id;         % id of the robot [1 - 5]
        sim;        % Simulation details
        info;       % Robot Ground Truth
        server;     % Communication server
        
        
        Est;                     % Estimated robot pose history
        LandmarkPositionHistory; % Estimated Landmark Position History
    end
    
    methods
        function obj = AgentDEKF(Robot, codeDict, server, params)
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
            
            obj.server = server;
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
            [measurement, others] = obj.sensor.sense(t);
            % correction step
            obj.ekf.correct(measurement);
            % send Fisher information matrix and information vector
            obj.broadcast(others);
        end
        
        %% Communication
        function broadcast(obj, receipients)
            % if no robot are seen return
            if isempty(receipients)
                return;
            end
            % prepare content of message
            content     = struct;
            content.landmarks = obj.get_landmarks();
            content.cov       = zeros(2,2,obj.ekf.N);
            for i = 1 : obj.ekf.N
                lID = 3 + 2 * i;
                content.cov(:, :, i) = obj.ekf.cov(lID-1:lID,lID-1:lID);
            end
            content.obs = obj.ekf.observed;
            % for each receipient send state
            R = numel(receipients(1, :));
            for i = 1 : R
                message = Message(content);
                send(obj.server, obj.id, receipients(3, i), message);
            end
        end
        
        
        function [count, y, Y, observed] = fetch(obj)
            inbox = obj.server.inbox{obj.id};
            obs = obj.ekf.observed;
            count = 0;
            for i = 1:numel(inbox)
                message = inbox(i);
                if ~isempty(message)
                    common = obs & message.content.obs;
                    if sum(common) > 1
                        count = count + 1;
                    end
                end
            end
            
            if count == 0
                y = [];
                Y = [];
                observed = [];
                return;
            end
            
            landmarks = obj.get_landmarks();
            y         = zeros(2, obj.ekf.N, count);
            Y         = zeros(2 , 2, obj.ekf.N, count);
            observed  = zeros(1, obj.ekf.N, count, 'logical');
            count = 0;
            for i = 1:numel(inbox)
                message = inbox(i);
                content = message.content;
                if ~isempty(message)
                    common = obs & content.obs;
                    if sum(common) > 1
                        L = content.landmarks;
                        count = count + 1;
                        [R, t] = find_transformation(L(:, common), landmarks(:, common));
                        L = R * L + t;
                        for j = 1 : obj.ekf.N
                            if content.obs(j)
                                state = L(:,j);
                                cov = R * content.cov(:,:,j) * R';
                                y(:, j, count) = cov \ state;
                                Y(:,:,j,count) = inv(cov);
                            else
                                y(:, j, count) = [0; 0];
                                Y(:,:,j,count) = zeros(2);
                            end
                        end
                        observed(1, :, count) = content.obs;
                    end
                end
            end
        end
        
        function consensus(obj)
            [count, y2, Y2, observed] = obj.fetch();
            if count == 0
                return;
            end
            
            Y = zeros(2,2,obj.ekf.N);
            y = zeros(2,obj.ekf.N);

            for i = 1:obj.ekf.N
                lID = 3 + 2*i;
                cov = obj.ekf.cov(lID-1:lID,lID-1:lID);
                state = obj.ekf.state(lID-1:lID);
                Y(:,:,i) = inv(cov);
                y(:,i)   = cov \ state;
            end
            obs = obj.ekf.observed;
            d = obs;
            for i = 1 : count
                d = d + observed(1,:,i);
                common = obs & observed(:,:,i);
                for j = 1:obj.ekf.N
                    if common(j)
                        Y(:,:,j) = Y(:,:,j) + Y2(:,:,j,i);
                        y(:,j) = y(:,j) + y2(:,j,i);
                    end
                end
            end
            
            for i = 1 : obj.ekf.N
                if obs(i)
                    den = 1 / (d(i));
                    Y(:,:,i) = Y(:,:,i) * den;
                    y(:, i) = y(:,i) * den;
                end
            end
            
            
            obj.ekf.cov(1:3,4:end) = 0;
            obj.ekf.cov(4:end, 1:3) = 0;
            obj.ekf.cov(4:end, 4:end) = 0;
            % for each landmark apply the merged map and covariance
            for i = 1:obj.ekf.N
                    lID = 3 + 2*i;
                    obj.ekf.cov(lID-1:lID, lID-1:lID) = inv(Y(:,:,i));
                    obj.ekf.state(lID-1:lID) = Y(:,:,i) \ y(:,i);
            end
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