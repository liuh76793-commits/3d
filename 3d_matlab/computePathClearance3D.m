function [minClearance, meanClearance, clearances, sampledPoints] = computePathClearance3D(path, scene, sampleStep)
% 沿路径线段采样计算安全裕度
%
% 输入：
%   path       : N x 3 路径点
%   scene      : 场景
%   sampleStep : 线段采样步长，默认 1.0
%
% 输出：
%   minClearance  : 路径采样点上的最小安全裕度
%   meanClearance : 路径采样点上的平均安全裕度
%   clearances    : 每个采样点的安全裕度
%   sampledPoints : 实际采样点

if nargin < 3
    sampleStep = 1.0;
end

if isempty(path)
    minClearance = inf;
    meanClearance = inf;
    clearances = [];
    sampledPoints = [];
    return;
end

if size(path,1) == 1
    sampledPoints = path;
    clearances = estimateClearance3D(path(1,:), scene);
    minClearance = clearances;
    meanClearance = clearances;
    return;
end

sampledPoints = [];

for i = 1:size(path,1)-1
    p1 = path(i,:);
    p2 = path(i+1,:);
    segLen = norm(p2 - p1);

    if segLen < 1e-10
        if isempty(sampledPoints) || norm(sampledPoints(end,:) - p1) > 1e-10
            sampledPoints = [sampledPoints; p1]; %#ok<AGROW>
        end
        continue;
    end

    nSample = max(ceil(segLen / sampleStep), 1);

    % 为避免重复点，除第一段外跳过 t=0
    if i == 1
        tVals = linspace(0, 1, nSample + 1);
    else
        tVals = linspace(0, 1, nSample + 1);
        tVals = tVals(2:end);
    end

    for t = tVals
        pt = p1 + t * (p2 - p1);
        sampledPoints = [sampledPoints; pt]; %#ok<AGROW>
    end
end

clearances = zeros(size(sampledPoints,1), 1);
for k = 1:size(sampledPoints,1)
    clearances(k) = estimateClearance3D(sampledPoints(k,:), scene);
end

minClearance = min(clearances);
meanClearance = mean(clearances);

end