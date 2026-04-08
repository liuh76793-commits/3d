clc; clear; close all;

%% 1. 场景
scene = createSceneS1();

%% 2. 基础参数
params.maxIter            = 4200;
params.baseStepLen        = 3.2;
params.minStepLen         = 1.0;
params.maxStepLen         = 3.9;
params.goalSampleRate     = 0.02;
params.connectThresh      = 7.2;
params.collisionStep      = 0.4;
params.randSeed           = 1;

%% 3. 滑动窗口参数
params.windowSize              = 20;
params.N_succ_min_for_pca      = 4;
params.localDensityRadius      = 9.0;

%% 4. 迟滞状态机参数
% ---- Free：保持适中 ----
params.free_enter_Rsucc        = 0.76;
params.free_exit_Rsucc         = 0.60;
params.free_enter_dbar         = 6.2;
params.free_exit_dbar          = 4.8;

% ---- Narrow：进一步收紧，避免它吞掉中后段 ----
params.narrow_enter_dbar       = 3.7;
params.narrow_exit_dbar        = 4.9;
params.narrow_enter_Lscore     = 3.2;
params.narrow_exit_Lscore      = 2.4;
params.narrow_enter_Rsucc_low  = 0.22;
params.narrow_enter_Rsucc_high = 0.88;

% ---- Blocked：继续削弱，只在明显停滞且风险很高时触发 ----
params.blocked_enter_Rsucc     = 0.08;
params.blocked_exit_Rsucc      = 0.18;
params.blocked_enter_Eprog     = 0.10;
params.blocked_exit_Eprog      = 0.35;
params.blocked_enter_dmin      = 0.10;
params.blocked_exit_dmin       = 0.35;
params.blockedFailConnTrigger  = 22;

% ---- Approach：比上一版更容易在中后段接管 ----
params.approach_enter_distGoal = 16.0;
params.approach_exit_distGoal  = 20.0;
params.approach_enter_distTree = 13.5;
params.approach_exit_distTree  = 16.5;
params.connFailThreshold       = 8;

% ---- 驻留时间：减少粘滞 ----
params.dwell_free     = 2;
params.dwell_narrow   = 1;
params.dwell_blocked  = 1;
params.dwell_approach = 2;

%% 5. 自适应步长参数
% 更重视远离障碍，但不想再出现大外绕
params.stepWeightClear = 0.60;
params.stepWeightProg  = 0.10;
params.stepWeightDens  = 1.10;

%% 6. Narrow 模式：PCA 定向采样参数
% 继续降低横向噪声，专门压贴边碎步
params.narrowDirectionalNoise = 0.10;
params.narrowLookAheadFactor  = 1.35;

%% 7. Blocked 模式：祖先回撤参数
% 保留轻量脱困，但避免过于发散
params.blockedBackSteps       = 1;
params.blockedLateralNoise    = 0.20;

%% 8. 节点质量评价参数
% 显著强调 clearance，让擦障节点更难被接受
params.qualityWeightProg      = 0.30;
params.qualityWeightClear     = 1.90;
params.qualityWeightDensity   = 1.05;
params.qualityThreshold       = 0.26;

%% 9. 平滑 seed 安全门槛
params.seedMinSafeClearance   = 1.2;
params.seedSmoothIters        = 3;
params.pruneMinSafeClearance  = 0.35;
params.pruneKeepClearRatio    = 0.85;

%% 10. 运行
tic;
fprintf('===== Active Key Params =====\n');
fprintf('free_enter_dbar         = %.2f\n', params.free_enter_dbar);
fprintf('narrow_enter_dbar       = %.2f\n', params.narrow_enter_dbar);
fprintf('narrow_enter_Lscore     = %.2f\n', params.narrow_enter_Lscore);
fprintf('blocked_enter_Rsucc     = %.2f\n', params.blocked_enter_Rsucc);
fprintf('blocked_enter_Eprog     = %.2f\n', params.blocked_enter_Eprog);
fprintf('blocked_enter_dmin      = %.2f\n', params.blocked_enter_dmin);
fprintf('blockedFailConnTrigger  = %d\n',  params.blockedFailConnTrigger);
fprintf('approach_enter_distGoal = %.2f\n', params.approach_enter_distGoal);
fprintf('approach_enter_distTree = %.2f\n', params.approach_enter_distTree);
fprintf('connectThresh           = %.2f\n', params.connectThresh);
fprintf('goalSampleRate          = %.2f\n', params.goalSampleRate);
fprintf('stepWeightClear         = %.2f\n', params.stepWeightClear);
fprintf('qualityWeightClear      = %.2f\n', params.qualityWeightClear);
fprintf('qualityThreshold        = %.2f\n', params.qualityThreshold);
fprintf('============================\n');

result = smhBiRRTConnect3D(scene, params);
planTime = toc;

%% 11. 后处理
if result.success
    rawPath = result.path;
    prunePath = shortcutPrunePath3D(rawPath, scene, params.collisionStep, params);
    seedPath  = smoothSeedPath3D(prunePath, scene, params.collisionStep, ...
                                 params.seedSmoothIters, params.seedMinSafeClearance);

    rawM   = evaluatePathMetrics3D(rawPath, scene);
    pruneM = evaluatePathMetrics3D(prunePath, scene);
    seedM  = evaluatePathMetrics3D(seedPath, scene);

    improveM = computeImprovementMetrics3D(rawM, pruneM, seedM);

    result.prunePath      = prunePath;
    result.seedPath       = seedPath;
    result.rawMetrics     = rawM;
    result.pruneMetrics   = pruneM;
    result.seedMetrics    = seedM;
    result.improveMetrics = improveM;
else
    rawPath   = [];
    prunePath = [];
    seedPath  = [];
end

%% 12. 绘图：原始路径
figure('Color','w','Name','SMH-BiRRTConnect 3D - Raw Path');
drawScene3D(scene, 'perspective');
hold on;

plotTree3DConnect(result.treeA, [0.15 0.45 1.0]);
plotTree3DConnect(result.treeB, [1.00 0.35 0.15]);

if result.success
    plotPath3D(rawPath, [1 0 0], 3.0);
    title(sprintf('SMH-BiRRTConnect Raw | Iter = %d | Nodes = %d | Length = %.2f', ...
        result.iterUsed, size(rawPath,1), result.rawMetrics.length));
else
    title(sprintf('SMH-BiRRTConnect Failed | Iter = %d', result.iterUsed));
end

%% 13. 绘图：后处理对比
if result.success
    figure('Color','w','Name','SMH-BiRRTConnect 3D - Postprocess');
    drawScene3D(scene, 'perspective');
    hold on;

    plotPath3D(rawPath,   [1 0 0],   2.5);
    plotPath3D(prunePath, [0 0.6 0], 2.5);
    plotPath3D(seedPath,  [0 0 0],   2.0);

    legend({'Raw Path','Pruned Path','Smooth Seed'}, 'Location','best');
    title(sprintf('Postprocess | Raw = %.2f | Pruned = %.2f | Seed = %.2f', ...
        result.rawMetrics.length, result.pruneMetrics.length, result.seedMetrics.length));
end

%% 14. 输出
fprintf('\n========== SMH-BiRRTConnect 3D Result ==========\n');
fprintf('是否成功: %d\n', result.success);
fprintf('规划时间/s: %.4f\n', planTime);
fprintf('使用迭代次数: %d\n', result.iterUsed);
fprintf('Tree A 节点数: %d\n', size(result.treeA.nodes,1));
fprintf('Tree B 节点数: %d\n', size(result.treeB.nodes,1));
fprintf('总节点数: %d\n', size(result.treeA.nodes,1) + size(result.treeB.nodes,1));

if result.success
    fprintf('\n原始路径节点数: %d\n', result.rawMetrics.nodeCount);
    fprintf('原始路径长度: %.4f\n', result.rawMetrics.length);
    fprintf('原始最小安全裕度: %.4f\n', result.rawMetrics.minClearance);
    fprintf('原始平均安全裕度: %.4f\n', result.rawMetrics.meanClearance);

    fprintf('\n剪枝后路径节点数: %d\n', result.pruneMetrics.nodeCount);
    fprintf('剪枝后路径长度: %.4f\n', result.pruneMetrics.length);
    fprintf('剪枝后最小安全裕度: %.4f\n', result.pruneMetrics.minClearance);
    fprintf('剪枝后平均安全裕度: %.4f\n', result.pruneMetrics.meanClearance);

    fprintf('\n平滑seed节点数: %d\n', result.seedMetrics.nodeCount);
    fprintf('平滑seed路径长度: %.4f\n', result.seedMetrics.length);
    fprintf('平滑seed最小安全裕度: %.4f\n', result.seedMetrics.minClearance);
    fprintf('平滑seed平均安全裕度: %.4f\n', result.seedMetrics.meanClearance);

    fprintf('\n剪枝长度改善率: %.2f %%\n', result.improveMetrics.pruneLengthImproveRate);
    fprintf('seed长度改善率: %.2f %%\n', result.improveMetrics.seedLengthImproveRate);

    fprintf('剪枝最小安全裕度改善率: %.2f %%\n', result.improveMetrics.pruneMinClearImproveRate);
    fprintf('seed最小安全裕度改善率: %.2f %%\n', result.improveMetrics.seedMinClearImproveRate);

    fprintf('剪枝平均安全裕度改善率: %.2f %%\n', result.improveMetrics.pruneMeanClearImproveRate);
    fprintf('seed平均安全裕度改善率: %.2f %%\n', result.improveMetrics.seedMeanClearImproveRate);

    fprintf('平滑安全损失率(相对剪枝最小安全裕度): %.2f %%\n', ...
        result.improveMetrics.seedSafetyLossFromPruneRate);
else
    fprintf('未找到可行路径。\n');
end

fprintf('\n---- 模式统计（扩展成功次数）----\n');
summarizeModeStats3D(result.stats);

fprintf('\n---- Tree A 窗口统计 ----\n');
disp(result.winA.metrics);

fprintf('\n---- Tree B 窗口统计 ----\n');
disp(result.winB.metrics);

fprintf('\n---- Tree A 状态 ----\n');
fprintf('state = %s, dwellCount = %d, failConn = %d\n', ...
    string(result.winA.state), result.winA.dwellCount, result.winA.failConn);

fprintf('\n---- Tree B 状态 ----\n');
fprintf('state = %s, dwellCount = %d, failConn = %d\n', ...
    string(result.winB.state), result.winB.dwellCount, result.winB.failConn);

isAStart = norm(result.treeA.nodes(1,:) - scene.start) < 1e-9;
if isAStart
    fprintf('Tree A root = start\n');
    fprintf('Tree B root = goal\n');
else
    fprintf('Tree A root = goal\n');
    fprintf('Tree B root = start\n');
end

fprintf('===============================================\n');