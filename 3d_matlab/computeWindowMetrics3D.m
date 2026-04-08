function [metrics, win] = computeWindowMetrics3D(win, refPoint, params)
% 从滑动窗口中计算局部状态统计量
% 修正版：
% 1) R_succ 用全部窗口样本
% 2) E_prog / dbar / dmin 仅对成功扩展样本统计
% 3) L_score 仅在成功样本足够时更新，否则冻结
% 4) L_score 增加上限裁剪，避免异常共线导致比值爆炸
% 5) rho_local 基于成功节点统计

metrics = struct( ...
    'R_succ', 0, ...
    'E_prog', 0, ...
    'dbar', inf, ...
    'dmin', inf, ...
    'L_score', win.lastLscore, ...
    'rho_local', 0, ...
    'N_succ', 0);

if isempty(win.success)
    win.metrics = metrics;
    return;
end

W_eff = numel(win.success);
succMask = logical(win.success(:));

% 1. 局部成功率：窗口内所有样本
metrics.R_succ = sum(succMask) / W_eff;

% 2. 成功样本数
metrics.N_succ = sum(succMask);

% 3. 仅对成功样本统计推进量和安全裕度
if metrics.N_succ > 0
    progSucc = win.deltaProg(succMask);
    clrSucc  = win.clearance(succMask);

    metrics.E_prog = mean(progSucc);
    metrics.dbar   = mean(clrSucc);
    metrics.dmin   = min(clrSucc);
else
    metrics.E_prog = 0;
    metrics.dbar   = inf;
    metrics.dmin   = inf;
end

% 4. 局部线性度：仅在成功节点足够时更新
if metrics.N_succ >= params.N_succ_min_for_pca && size(win.successNodes,1) >= params.N_succ_min_for_pca
    L_score = computeLocalLinearity3D(win.successNodes, 1e-6);

    if ~isnan(L_score) && ~isinf(L_score)
        % ===== 新增：线性度上限裁剪 =====
        L_score = min(L_score, 20);
        metrics.L_score = L_score;
        win.lastLscore = L_score;
    else
        metrics.L_score = win.lastLscore;
    end
else
    metrics.L_score = win.lastLscore;
end

% 5. 局部节点密度：基于成功节点
metrics.rho_local = computeLocalDensity3D(win.successNodes, refPoint, params.localDensityRadius);

win.metrics = metrics;

end