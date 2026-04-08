function L_score = computeLocalLinearity3D(successNodes, epsVal)
% 基于最近成功节点计算局部线性度
% successNodes: N x 3
% L_score = lambda1 / (lambda2 + eps)

if nargin < 2
    epsVal = 1e-6;
end

N = size(successNodes, 1);

% 样本太少时，不可信
if N < 3
    L_score = NaN;
    return;
end

X = successNodes - mean(successNodes, 1);

% 协方差矩阵
C = (X' * X) / max(N - 1, 1);

% 特征值
eigVals = eig(C);
eigVals = sort(real(eigVals), 'descend');

if numel(eigVals) < 2
    L_score = NaN;
    return;
end

lambda1 = eigVals(1);
lambda2 = eigVals(2);

L_score = lambda1 / (lambda2 + epsVal);

end