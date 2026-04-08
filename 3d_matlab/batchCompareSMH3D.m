clc; clear; close all;

%% 场景
scene = createSceneS1();

%% 参数（与主程序保持一致）
params.maxIter            = 4000;
params.baseStepLen        = 4.0;
params.minStepLen         = 1.8;
params.maxStepLen         = 5.0;
params.goalSampleRate     = 0.05;
params.connectThresh      = 3.5;
params.collisionStep      = 1.0;

params.windowSize              = 20;
params.N_succ_min_for_pca      = 4;
params.localDensityRadius      = 10.0;

params.free_enter_Rsucc        = 0.80;
params.free_exit_Rsucc         = 0.65;
params.free_enter_dbar         = 7.5;
params.free_exit_dbar          = 5.5;

params.narrow_enter_dbar       = 5.0;
params.narrow_exit_dbar        = 6.5;
params.narrow_enter_Lscore     = 2.2;
params.narrow_exit_Lscore      = 1.6;
params.narrow_enter_Rsucc_low  = 0.20;
params.narrow_enter_Rsucc_high = 0.90;

params.blocked_enter_Rsucc     = 0.45;
params.blocked_exit_Rsucc      = 0.55;
params.blocked_enter_Eprog     = 2.0;
params.blocked_exit_Eprog      = 2.6;
params.blocked_enter_dmin      = 3.0;
params.blocked_exit_dmin       = 4.0;

params.approach_enter_distGoal = 10.0;
params.approach_exit_distGoal  = 13.0;
params.approach_enter_distTree = 8.0;
params.approach_exit_distTree  = 11.0;
params.connFailThreshold       = 2;

params.dwell_free     = 2;
params.dwell_narrow   = 3;
params.dwell_blocked  = 3;
params.dwell_approach = 2;

params.stepWeightClear = 0.20;
params.stepWeightProg  = 0.35;
params.stepWeightDens  = 1.50;

params.narrowDirectionalNoise = 1.2;
params.narrowLookAheadFactor  = 2.0;

params.blockedBackSteps       = 2;
params.blockedLateralNoise    = 1.8;

params.qualityWeightProg      = 1.0;
params.qualityWeightClear     = 0.35;
params.qualityWeightDensity   = 0.8;
params.qualityThreshold       = -0.05;

%% 批量次数
numRuns = 20;

% 结果记录
successArr = zeros(numRuns,1);
timeArr = nan(numRuns,1);

rawLenArr   = nan(numRuns,1);
pruneLenArr = nan(numRuns,1);
seedLenArr  = nan(numRuns,1);

rawNodeArr   = nan(numRuns,1);
pruneNodeArr = nan(numRuns,1);
seedNodeArr  = nan(numRuns,1);

rawMinClrArr   = nan(numRuns,1);
pruneMinClrArr = nan(numRuns,1);
seedMinClrArr  = nan(numRuns,1);

rawMeanClrArr   = nan(numRuns,1);
pruneMeanClrArr = nan(numRuns,1);
seedMeanClrArr  = nan(numRuns,1);

treeNodeArr = nan(numRuns,1);

freeArr     = nan(numRuns,1);
narrowArr   = nan(numRuns,1);
blockedArr  = nan(numRuns,1);
approachArr = nan(numRuns,1);
switchArr   = nan(numRuns,1);

%% 批量运行
for k = 1:numRuns
    params.randSeed = k;

    tic;
    result = smhBiRRTConnect3D(scene, params);
    tElapsed = toc;

    successArr(k) = result.success;
    timeArr(k) = tElapsed;

    if result.success
        rawPath = result.path;
        prunePath = shortcutPrunePath3D(rawPath, scene, params.collisionStep);
        seedPath = smoothSeedPath3D(prunePath, scene, params.collisionStep, 3);

        rawM   = evaluatePathMetrics3D(rawPath, scene);
        pruneM = evaluatePathMetrics3D(prunePath, scene);
        seedM  = evaluatePathMetrics3D(seedPath, scene);

        rawLenArr(k)   = rawM.length;
        pruneLenArr(k) = pruneM.length;
        seedLenArr(k)  = seedM.length;

        rawNodeArr(k)   = rawM.nodeCount;
        pruneNodeArr(k) = pruneM.nodeCount;
        seedNodeArr(k)  = seedM.nodeCount;

        rawMinClrArr(k)   = rawM.minClearance;
        pruneMinClrArr(k) = pruneM.minClearance;
        seedMinClrArr(k)  = seedM.minClearance;

        rawMeanClrArr(k)   = rawM.meanClearance;
        pruneMeanClrArr(k) = pruneM.meanClearance;
        seedMeanClrArr(k)  = seedM.meanClearance;

        treeNodeArr(k) = size(result.treeA.nodes,1) + size(result.treeB.nodes,1);

        freeArr(k)     = result.stats.free;
        narrowArr(k)   = result.stats.narrow;
        blockedArr(k)  = result.stats.blocked;
        approachArr(k) = result.stats.approach;
        switchArr(k)   = result.stats.switches;
    end

    fprintf('Run %02d/%02d | success = %d | time = %.4f s\n', ...
        k, numRuns, result.success, tElapsed);
end

%% 成功样本筛选
succMask = successArr == 1;

fprintf('\n========== SMH-BiRRTConnect Batch Summary ==========\n');
fprintf('总次数: %d\n', numRuns);
fprintf('成功次数: %d\n', sum(succMask));
fprintf('成功率: %.2f %%\n', 100 * mean(successArr));

if any(succMask)
    fprintf('\n---- 成功样本均值 ± 标准差 ----\n');

    printMeanStd('规划时间/s', timeArr(succMask));
    printMeanStd('原始路径长度', rawLenArr(succMask));
    printMeanStd('剪枝后路径长度', pruneLenArr(succMask));
    printMeanStd('平滑seed路径长度', seedLenArr(succMask));

    printMeanStd('原始路径点数', rawNodeArr(succMask));
    printMeanStd('剪枝后路径点数', pruneNodeArr(succMask));
    printMeanStd('平滑seed点数', seedNodeArr(succMask));

    printMeanStd('原始最小安全裕度', rawMinClrArr(succMask));
    printMeanStd('剪枝后最小安全裕度', pruneMinClrArr(succMask));
    printMeanStd('平滑seed最小安全裕度', seedMinClrArr(succMask));

    printMeanStd('原始平均安全裕度', rawMeanClrArr(succMask));
    printMeanStd('剪枝后平均安全裕度', pruneMeanClrArr(succMask));
    printMeanStd('平滑seed平均安全裕度', seedMeanClrArr(succMask));

    printMeanStd('搜索树总节点数', treeNodeArr(succMask));

    printMeanStd('Free 次数', freeArr(succMask));
    printMeanStd('Narrow 次数', narrowArr(succMask));
    printMeanStd('Blocked 次数', blockedArr(succMask));
    printMeanStd('Approach 次数', approachArr(succMask));
    printMeanStd('Switches 次数', switchArr(succMask));
end

fprintf('===================================================\n');

%% ===== 辅助函数 =====
function printMeanStd(nameStr, arr)
fprintf('%s: %.4f ± %.4f\n', nameStr, mean(arr), std(arr));
end