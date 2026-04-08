function improveM = computeImprovementMetrics3D(rawM, pruneM, seedM)
% 计算路径后处理前后的改善指标
%
% 输入：
%   rawM   : 原始路径指标结构体
%   pruneM : 剪枝后路径指标结构体
%   seedM  : 平滑 seed 路径指标结构体
%
% 输出字段：
%   pruneLengthImproveRate
%   seedLengthImproveRate
%   pruneMinClearImproveRate
%   seedMinClearImproveRate
%   pruneMeanClearImproveRate
%   seedMeanClearImproveRate
%   seedSafetyLossFromPruneRate

improveM = struct( ...
    'pruneLengthImproveRate', 0, ...
    'seedLengthImproveRate', 0, ...
    'pruneMinClearImproveRate', 0, ...
    'seedMinClearImproveRate', 0, ...
    'pruneMeanClearImproveRate', 0, ...
    'seedMeanClearImproveRate', 0, ...
    'seedSafetyLossFromPruneRate', 0);

% ---------- 长度改善率 ----------
if isfinite(rawM.length) && rawM.length > 1e-10
    improveM.pruneLengthImproveRate = 100 * (rawM.length - pruneM.length) / rawM.length;
    improveM.seedLengthImproveRate  = 100 * (rawM.length - seedM.length)  / rawM.length;
end

% ---------- 最小安全裕度改善率 ----------
if isfinite(rawM.minClearance) && rawM.minClearance > 1e-10
    improveM.pruneMinClearImproveRate = 100 * (pruneM.minClearance - rawM.minClearance) / rawM.minClearance;
    improveM.seedMinClearImproveRate  = 100 * (seedM.minClearance  - rawM.minClearance) / rawM.minClearance;
end

% ---------- 平均安全裕度改善率 ----------
if isfinite(rawM.meanClearance) && rawM.meanClearance > 1e-10
    improveM.pruneMeanClearImproveRate = 100 * (pruneM.meanClearance - rawM.meanClearance) / rawM.meanClearance;
    improveM.seedMeanClearImproveRate  = 100 * (seedM.meanClearance  - rawM.meanClearance) / rawM.meanClearance;
end

% ---------- 平滑相对剪枝的安全损失率 ----------
% 若 seed 的最小安全裕度低于 prune，则为负值；高于 prune 则为正值
if isfinite(pruneM.minClearance) && pruneM.minClearance > 1e-10
    improveM.seedSafetyLossFromPruneRate = ...
        100 * (seedM.minClearance - pruneM.minClearance) / pruneM.minClearance;
end

end