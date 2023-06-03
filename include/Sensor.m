% Copyright (C) 2023  Marco "marcope-98" Peressutti
% See end of file for extended copyright information

%% Class Definition
classdef Sensor < handle
    properties
        current     = 1;
        Measurement = [];
        codeDict    = [];
    end
    
    methods
        function obj = Sensor(codeDict, Measurement)
            obj.codeDict    = codeDict;
            obj.Measurement = Measurement;
        end
        
        function varargout = sense(obj, t)
            measurement = [];
            receipients = [];
            i    = obj.current;
            last = numel(obj.Measurement(:,1));
            while (obj.Measurement(i, 1) - t < .005) && (i < last)
                barcode = obj.Measurement(i, 2);
                if (obj.codeDict.isKey(barcode))
                    landmarkID = obj.codeDict(barcode);
                else
                    landmarkID = 0;
                end
                
                if landmarkID > 0 && landmarkID <= 5
                    z = [obj.Measurement(i, 3:4)'; landmarkID];
                    receipients = [receipients, z];
                end
                if landmarkID > 5 && landmarkID < 21
                    z = [obj.Measurement(i, 3:4)'; landmarkID];
                    measurement = [measurement, z];
                end
                
                i = i + 1;
            
            end
            
            varargout{1} = measurement;
            if nargout == 2
                varargout{2} = receipients;
            end
            obj.current = i;
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