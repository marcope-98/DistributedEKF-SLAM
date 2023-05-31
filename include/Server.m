% Copyright (C) 2023  Marco "marcope-98" Peressutti
% See end of file for extended copyright information

%% This needs to be a handle class for the love of god
classdef Server < handle 
    properties
        N           = 0;  % number of nodes
        inbox       = {}; % NxN cell array of messages to receive
    end
    
    
    methods
        function obj = Server(nRobots)
            obj.N = nRobots;
            % Initialize empty inbox
            temp(1, obj.N) = Message();
            obj.inbox = cell([1, obj.N]);
            for i = 1:obj.N
                obj.inbox{i} = temp;
            end
        end
        
        % send message to server from agent
        function send(obj, from, to, message)
            if to > obj.N
                return;
            end
            obj.inbox{to}(from) = message;
        end

        % reset the inbox
        function flush(obj)
            temp(1, obj.N) = Message();
            for i = 1 : obj.N
                obj.inbox{i} = temp;
            end
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