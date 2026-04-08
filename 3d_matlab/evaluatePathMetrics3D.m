function metrics = evaluatePathMetrics3D(path, scene)
% 统一评估路径质量
%
% 输出字段：
%   nodeCount
%   length
%   minClearance
%   meanClearance

metrics = struct( ...
    'nodeCount', 0, ...
    'length', inf, ...
    'minClearance', inf, ...
    'meanClearance', inf);

if isempty(path)
    return;
end

metrics.nodeCount = size(path, 1);
metrics.length = computePathLength3D(path);

[minClr, meanClr, ~] = computePathClearance3D(path, scene);
metrics.minClearance = minClr;
metrics.meanClearance = meanClr;

end