% Copyright (C) 2023  Marco "marcope-98" Peressutti
% See end of file for extended copyright information

%%
classdef Agent < handle
    properties
        ekf;        % EKF SLAM object
        sensor;     % Sensor   object
        actuator;   % Actuator object
        
        id;         % id of the robot [1 - 5]
        sim;        % Simulation details
        info;       % Robot Ground Truth
        Est;        % Estimated robot pose history
        server;     % Communication server
    end
    
    methods
        function obj = Agent(Robot, codeDict, server, params)
            obj.id       = params.id;   % robot id
            obj.sim      = params.sim;  % simulation parameters
            obj.info     = Robot.G;     % ground truth info
            
            state_0 = obj.info(obj.sim.start, 2:4)';
            
            obj.ekf      = ekfSLAM(state_0, params.ekf);
            obj.sensor   = Sensor(codeDict, Robot.M);
            obj.actuator = Actuator(obj.sim.start, Robot.O);
            
            obj.Est      = -10 * ones(numel(obj.info(:,1)), 4);
            obj.Est(:,1) = obj.info(:,1);
            
            obj.server = server;
        end
        
        function step(obj)
            obj.Est(obj.actuator.current, 2:4) = obj.get_state()';
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
            content.Y = zeros(2,2,obj.ekf.N);
            content.y = zeros(2, obj.ekf.N);
            for i = 1:obj.ekf.N
                lID = 3 + 2*i;
                cov = obj.ekf.cov(lID-1:lID,lID-1:lID);
                state = obj.ekf.state(lID-1:lID);
                content.Y(:,:,i) = inv(cov);
                content.y(:,i)   = cov \ state;
            end
            content.obs = obj.ekf.observed;
            
            % for each receipient send state
            R = numel(receipients);
            for i = 1:R
                message = Message(content);
                send(obj.server, obj.id, receipients(i), message);
            end
        end
        
        function fetch(obj)
            % fetch messages from inbox
            inbox = obj.server.inbox{obj.id};
            % Retrieve self Fisher information matrix and information
            % vector
            Y = zeros(2,2,obj.ekf.N);
            y = zeros(2, obj.ekf.N);
            for i = 1:obj.ekf.N
                lID = 3 + 2*i;
                cov = obj.ekf.cov(lID-1:lID,lID-1:lID);
                state = obj.ekf.state(lID-1:lID);
                Y(:,:,i) = inv(cov);
                y(:,i)   = cov \ state;
            end
            obs = obj.ekf.observed;
            % perform consensus
            d = 1; % used of iterative mean computation
            % for each message in the inbox
            for i = 1:numel(inbox)
                % fetch content of the message
                message = inbox(i);
                content = message.content;
                % if the message is not empty
                if ~isempty(message)
                    % compute factor for iterative mean computation
                    d = d + 1;
                    fact1 = 1 / d;
                    fact2 = (d - 1) * fact1;
                    % merge seen observed landmark
                    obs = obs | content.obs;
                    % for each landmark perform information consensus
                    for j = 1:obj.ekf.N
                        Yj = content.Y(:,:,j);
                        yj = content.y(:,j);

                        y(:,j)   = fact2 * y(:,j)   + fact1 * yj ;
                        Y(:,:,j) = fact2 * Y(:,:,j) + fact1 * Yj ;
                    end
                end
            end

            % if no message where present in the inbox return
            if d == 1
                return;
            end
            % we cannot assume correlation after map merging
            obj.ekf.cov(1:3,4:end) = 0;
            obj.ekf.cov(4:end, 1:3) = 0;
            obj.ekf.cov(4:end, 4:end) = 0;
            % for each landmark apply the merged map and covariance
            for i = 1:obj.ekf.N
                lID = 3 + 2*i;
                obj.ekf.cov(lID-1:lID, lID-1:lID) = inv(Y(:,:,i));
                obj.ekf.state(lID-1:lID) = Y(:,:,i) \ y(:,i);
            end
            obj.ekf.observed     = obs;
        end
        
        %% Utils
        
        function Robot = cvt_to_Robot(obj)
            Robot = struct;
            Robot.O = obj.actuator.Odometry;
            Robot.M = obj.sensor.Measurement;
            Robot.G = obj.info;
            Robot.Est = obj.Est;
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