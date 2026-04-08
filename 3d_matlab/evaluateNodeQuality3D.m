function [J, accepted] = evaluateNodeQuality3D(deltaProg, clearanceVal, rho_local, params)
% 节点质量评价（改进版）
%
% 目标：
% 1) 不是只鼓励推进，还要显著鼓励远离障碍
% 2) 对“极小安全裕度”的节点做硬拒绝
% 3) 对局部堆点区域继续惩罚
%
% 参数新增建议：
% params.minAcceptClearance
% params.clearanceSoftRef

% ---- 硬约束：太贴障的节点直接拒绝 ----
if clearanceVal < params.minAcceptClearance
    J = -inf;
    accepted = false;
    return;
end

% ---- clearance 饱和上限，避免超大 clearance 失真 ----
clearEff = min(clearanceVal, params.clearanceSoftRef);

% ---- 质量函数 ----
J = params.qualityWeightProg    * deltaProg + ...
    params.qualityWeightClear   * clearEff  - ...
    params.qualityWeightDensity * rho_local;

accepted = (J >= params.qualityThreshold);
end