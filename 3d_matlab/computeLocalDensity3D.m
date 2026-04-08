function rho_local = computeLocalDensity3D(successNodes, refPoint, radius)
% 计算局部节点密度
% successNodes: 最近成功节点
% refPoint    : 当前参考点
% radius      : 局部统计半径

if nargin < 3
    radius = 10.0;
end

if isempty(successNodes)
    rho_local = 0;
    return;
end

diffs = successNodes - refPoint;
dists = sqrt(sum(diffs.^2, 2));

countInRadius = sum(dists <= radius);

% 归一化成较平滑的密度值
rho_local = countInRadius / max(size(successNodes,1), 1);

end