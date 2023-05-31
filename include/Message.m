%% Dont make this a handle class for the love of god

classdef Message
    properties
        content = struct([]);
    end
    methods
        function obj = Message(varargin)
            if(nargin > 0)
                obj.content = varargin{1};
            end
        end
        
        function flag = isempty(obj)
            flag = isempty(obj.content);
        end
    end
end