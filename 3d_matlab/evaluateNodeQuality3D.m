function [J, accepted] = evaluateNodeQuality3D(deltaProg, clearanceVal, rho_local, params)
% 节点质量评价（修正版）
%
% 1) 对过小 clearance 做硬拒绝
% 2) clearance 做软饱和，避免极大值主导
% 3) 继续惩罚局部堆点

% 硬约束：太贴障的节点直接拒绝
if clearanceVal < params.minAcceptClearance
    J = -inf;
    accepted = false;
    return;
end

% soft cap
clearEff = min(clearanceVal, params.clearanceSoftRef);

J = params.qualityWeightProg    * deltaProg + ...
    params.qualityWeightClear   * clearEff  - ...
    params.qualityWeightDensity * rho_local;

accepted = (J >= params.qualityThreshold);
end