% Copyright (C) 2023  Marco "marcope-98" Peressutti
% See end of file for extended copyright information

%%
classdef ekfSLAM < handle
    % ekfSLAM Extended Kalman Filter Simultaneous Localization and Mapping 
    %   The class is responsible for the correction and prediction steps in
    %   the EKF SLAM algorithm with known correspondence. 
    %   The class assumes:
    %   - a velocity based motion model 
    %   - a range and bearing measurement model
    %   - a fixed number of landmarks
    properties
        Q;        % measurement noise matrix
        A;        % parameters of control noise matrix
        
        state;    % state      vector
        cov;      % covariance matrix
        N;        % number of landmarks
        observed; % bool array of landmark
    end
    
    methods
        %% Contructor
        % @param state_0  vector (3x1) containing the initial 2d pose of
        %                 the robot [x_0, y_0, theta_0]
        % @param params   struct containing three fiels:
        %                  - params.N: Number of landmarks
        %                  - params.Q: (2x2) measurement noise matrix
        %                  - params.A: (4x1) parameters of control noise
        %                              matrix (@see ekfSLAM.predict())
        function obj = ekfSLAM(state_0, params)
            obj.Q     = params.Q;
            obj.A     = params.A;
            obj.N     = params.N;
            % initialize state vector with NaNs
            obj.state = zeros(3 + 2 * obj.N, 1);
            % initialize estimate of robot pose with groundtruth
            obj.state(1:3) = state_0;
            % initialize covariance matrix with zeros
            obj.cov          = 1e10 * eye(3 + 2 * obj.N);
            obj.cov(1:3,1:3) = 1e-3 * eye(3);
            % initialize observed landmarks to false
            obj.observed     = zeros(1, obj.N, 'logical');
        end
        
        %% EKF prediction step
        % @param u   vector (3x1) in the form [v, w, dt]:
        %            v  : linear  velocity
        %            w  : angular velocity
        %            dt : timestep
        function predict(obj, u)
            % derive velocity-based motion model using the controls
            [g, G, V] = obj.velocity_motion_model(u);
            v = u(1); % linear  velocity
            w = u(2); % angular velocity
            
            % compute the process noise matrix R
            % R = [ a_1 * |v| + a_2 * |w| ,           0           ]^2
            %     [           0           , a_3 * |v| + a_4 * |w| ]
            u1 = (obj.A(1) * abs(v) + obj.A(2) * abs(w))^2;
            u2 = (obj.A(3) * abs(v) + obj.A(4) * abs(w))^2;
            U = [u1, 0; 0, u2];
            R = V * U * V'; 
            
            % compute state prediction
            obj.state(1:3)      = obj.state(1:3) + g;
            obj.state(3)        = wrapToPi(obj.state(3));
            % compute covariance prediction
            sigma_xm            = G * obj.cov(1:3, 4:end);
            obj.cov(1:3, 1:3)   = G * obj.cov(1:3, 1:3) * G' + R;
            obj.cov(1:3, 4:end) = sigma_xm;
            obj.cov(4:end, 1:3) = sigma_xm';
        end
        
        %% Velocity based motion model
        % @param u   vector (3x1) in the form [v, w, dt]:
        %            v  : linear  velocity
        %            w  : angular velocity
        %            dt : timestep
        %
        % @out   g   vector (3x1) velocity motion model
        % @out   G   matrix (3x3) jacobian of the motion model wrt state   
        %            (dg/dstate)
        % @out   V   matrix (3x2) jacobian of the motion model wrt control 
        %            (dg/du)
        function [g, G, V] = velocity_motion_model(obj, u)
            th = obj.state(3);
            v  = u(1);
            w  = u(2);
            dt = u(3);
            
            G = eye(3);
            % singularity at w -> 0: solved using limits
            if abs(w) < 1e-4
                c = cos(th);
                s = sin(th);
                
                g = [v * dt * c;
                     v * dt * s;
                     0];
                
                G(1,3) = - g(2);
                G(2,3) =   g(1);
                
                V = [dt * c, - g(2) * dt / 2;
                     dt * s,   g(1) * dt / 2;
                          0,          dt];
                      
            else % normal case
                r = v / w;
                
                c1 = cos(th);
                s1 = sin(th);
                c2 = cos(th + w*dt);
                s2 = sin(th + w*dt);
                dc = c1 - c2;
                ds = s1 - s2;
                
                g = [- r * ds;
                       r * dc;
                       w * dt];
                               
                G(1,3) = - g(2);
                G(2,3) =   g(1);
                
                V = [- ds / w, r * ( (ds / w) + c2 * dt);
                       dc / w, r * (-(dc / w) + s2 * dt);
                            0,                       dt];
            end
        end
        
        %% Measurement model
        % @param landmark   vector (2x1) containing the x and y coordinates
        %                   of the landmark
        %
        % @out   zHat       vector (2x1) estimated observation containing range and bearing
        function zHat = h(obj, landmark)
            zHat = cvt_xy_to_rb(landmark, obj.state(1:3));
        end
        
        %% Jacobian of the measurement model
        % @param landmark   vector (2x1) containing the x and y coordiantes
        %                   of the landmark
        %
        % @out   H3         matrix (2x3) containing the partial derivative
        %                   of the observation model wrt to the state
        function H3 = dh_dstate(obj, landmark)
            dx = landmark(1) - obj.state(1);
            dy = landmark(2) - obj.state(2);
            q  = dx^2 + dy^2;
            sq = sqrt(q);
            
            H3 = zeros(2,3);
            H3(1,1) = - dx / sq;
            H3(1,2) = - dy / sq;
            H3(2,1) =   dy /  q;
            H3(2,2) = - dx /  q;
            H3(2,3) = - 1; 
        end
        
        %% EKF correction step
        % @param measurement  vector (3xM) containing the measurements 
        %                     Each column contains the range, bearing and
        %                     landmarkID of the observed landmark
        function correct(obj, measurement)
            % if no measurement provided return
            if isempty(measurement)
                return;
            end
            I          = eye(3 + 2*obj.N);
            bias       = [0.055; 0];              % radius of the landmark
            L          = numel(measurement(1,:)); % number of observations
            
            % helper variables to stack state and covariance update
            mu_update  = zeros(3 + 2 * obj.N, 1); 
            cov_update = zeros(3 + 2 * obj.N);
            
            % for each observed landmark / for each observaction
            for i = 1:L
                rb = measurement(1:2, i) + bias;  % add bias to measurement
                liD   = measurement(3, i) - 5;    % retrieve landmark id
                land_pos = 3 + 2 * liD;           % retrieve index of landmark in state vector
                landmark = obj.state(land_pos-1:land_pos); % retrieve landmark
                % if landmark was not observed before, initialize the landmark
                if ~obj.observed(liD)
                    obj.observed(liD) = true;
                    % retrieve x and y coordinate of the landmark from
                    % current pose estimate and measurement
                    landmark = cvt_rb_to_xy(rb, obj.state(1:3));
                    % add coordinates to state vector
                    obj.state(land_pos-1:land_pos) = landmark; 
                end
                % retrieve the Jacobian of the measurement model wrt the state
                H3 = obj.dh_dstate(landmark);
                % build the H matrix
                H  = zeros(2, 3 + 2*obj.N);
                H(:, 1:3) = H3;
                H(:, land_pos-1:land_pos) = - H3(:,1:2);
                % Kalman gain
                K = (obj.cov * H') / ((H * obj.cov * H') + obj.Q);
                % compute innovation (z - zHat)
                innovation = rb - obj.h(landmark);
                innovation(2) = wrapToPi(innovation(2));
                % stack state and covariance updates
                mu_update = mu_update + K * innovation;
                cov_update = cov_update + K * H;
            end
            % apply state and covariance update
            obj.state = obj.state + mu_update;
            obj.state(3) = wrapToPi(obj.state(3));
            obj.cov = (I - cov_update) * obj.cov;
            
        end
        
        %% Utility: getter to retrieve the current estimated pose of the robot
        % @out    pose vector (3x1) containing x y and theta 
        function pose = get_pose(obj)
            pose = obj.state(1:3);
        end
        
        %% Utility: getter to retrieve position of landmarks
        function landmarks = get_landmarks(obj)
            landmarks = zeros(2, obj.N);
            landmarks(1,:) = obj.state(4:2:4+2*(obj.N)-1);
            landmarks(2,:) = obj.state(5:2:4+2*(obj.N)-1);
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