
% finds transformation that aligns landmarks from L1 into landmarks to L2
function [R, t] = find_transformation(x1, x2)

    N = numel(x1(1,:));
    mu_1 = mean(x1, 2);
    mu_2 = mean(x2, 2);
    W = zeros(2);
    
    x1 = x1 - mu_1;
    x2 = x2 - mu_2;
    
    for i = 1: N
        W = W + x1(:, i) * x2(:,i)';
    end
    [U, ~, V] = svd(W);
    D = [1, 0; 0 sign(det(W))];
    R = V * D * U';
    t = mu_2 - R * mu_1;
end