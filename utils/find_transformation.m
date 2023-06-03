
% finds transformation that aligns landmarks from L1 into landmarks to L2
function varargout = find_transformation(varargin)

    x1 = varargin{1};
    x2 = varargin{2};
    N = numel(x1(1,:));
    
    if nargin == 2
        mu_1 = mean(x1, 2);
        mu_2 = mean(x2, 2);
    end
    
    if nargin == 3
        wrt = varargin{3};
        mu_1 = x1(:,wrt);
        mu_2 = x2(:,wrt);
    end
    
    W = zeros(2);
    
    x1 = x1 - mu_1;
    x2 = x2 - mu_2;
    
    for i = 1: N
        W = W + x1(:, i) * x2(:,i)';
    end
    [U, ~, V] = svd(W);
    R = V * U';
    t = mu_2 - R * mu_1;
    if nargout == 1
        varargout{1} = R; 
    else
        varargout{1} = R;
        varargout{2} = t;
    end
end