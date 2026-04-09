function result = smhBiRRTConnect3D(scene, params)
% 三维 SMH-BiRRTConnect
% 这版保持与当前仓库其余辅助函数兼容，并修正：
% 1) updateHysteresisState3D 的参数顺序
% 2) Blocked / Approach 的判定逻辑
% 3) 与 extendByMode3D / connectTreeByMode3D 的旧接口完全对齐

rng(params.randSeed);

%% 初始化双树
treeA.nodes = scene.start;
treeA.parent = 0;
treeA.mode = {"root"};

treeB.nodes = scene.goal;
treeB.parent = 0;
treeB.mode = {"root"};

failCountA = 0;
failCountB = 0;

winA = initWindowState3D(params.windowSize);
winB = initWindowState3D(params.windowSize);

stats.free = 0;
stats.narrow = 0;
stats.blocked = 0;
stats.approach = 0;
stats.switches = 0;

success = false;
meetIdxA = -1;
meetIdxB = -1;

for iter = 1:params.maxIter
    refA = treeA.nodes(end, :);
    refB = treeB.nodes(end, :);

    [metricsA, winA] = computeWindowMetrics3D(winA, refA, params);
    [metricsB, winB] = computeWindowMetrics3D(winB, refB, params);

    distTreeAB = norm(refA - refB);

    isStartRootA = (norm(treeA.nodes(1,:) - scene.start) < 1e-9);
    isStartRootB = (norm(treeB.nodes(1,:) - scene.start) < 1e-9);

    [winA, switchIncA] = updateHysteresisState3D(winA, metricsA, refA, scene.goal, distTreeAB, isStartRootA, params);
    [winB, switchIncB] = updateHysteresisState3D(winB, metricsB, refB, scene.goal, distTreeAB, isStartRootB, params);
    stats.switches = stats.switches + switchIncA + switchIncB;

    %% 采样
    q_rand = sampleBiased3D(scene, params, treeA, treeB);

    %% treeA 扩展一步
    [treeA, reachedA, newIdxA, ~, failCountA, stats, ~, deltaProgA, clearA, qualityA] = ...
        extendByMode3D(treeA, q_rand, scene, params, failCountA, stats, 'A', winA.state, metricsA, winA);

    if reachedA
        q_newA = treeA.nodes(newIdxA, :);
        winA = updateWindowState3D(winA, 1, deltaProgA, clearA, q_newA, qualityA);
    else
        winA = updateWindowState3D(winA, 0, 0, inf, [], 0);
    end

    %% 对向树 connect
    if reachedA
        q_newA = treeA.nodes(newIdxA, :);
        nNodesB_before = size(treeB.nodes, 1);

        [treeB, connected, newIdxB, failCountB, stats, ~, deltaProgB, clearB, qualityB] = ...
            connectTreeByMode3D(treeB, q_newA, scene, params, failCountB, stats, 'B', winB.state, metricsB);

        nNodesB_after = size(treeB.nodes, 1);

        if connected
            meetIdxA = newIdxA;
            meetIdxB = newIdxB;

            q_meetB = treeB.nodes(newIdxB, :);
            winB = updateWindowState3D(winB, 1, deltaProgB, clearB, q_meetB, qualityB);
            winB.failConn = 0;

            success = true;
            break;
        else
            if nNodesB_after > nNodesB_before
                lastIdxB = nNodesB_after;
                q_lastB = treeB.nodes(lastIdxB, :);
                winB = updateWindowState3D(winB, 1, deltaProgB, clearB, q_lastB, qualityB);
                winB.failConn = 0;
            else
                winB = updateWindowState3D(winB, 0, 0, inf, [], 0);
                winB.failConn = winB.failConn + 1;
            end
        end
    end

    %% 交换两树角色
    tmpTree = treeA; treeA = treeB; treeB = tmpTree;
    tmpFail = failCountA; failCountA = failCountB; failCountB = tmpFail;
    tmpWin  = winA; winA = winB; winB = tmpWin;
end

%% 输出整理
result.success = success;
result.iterUsed = iter;
result.treeA = treeA;
result.treeB = treeB;
result.stats = stats;
result.winA = winA;
result.winB = winB;

if success
    rootA = treeA.nodes(1,:);

    if norm(rootA - scene.start) < 1e-9
        startTree = treeA;
        startMeet = meetIdxA;
        goalTree = treeB;
        goalMeet = meetIdxB;
    else
        startTree = treeB;
        startMeet = meetIdxB;
        goalTree = treeA;
        goalMeet = meetIdxA;
    end

    pathStart = extractPathFromTree3D(startTree, startMeet);
    pathGoal  = extractPathFromTree3D(goalTree, goalMeet);
    pathGoal  = flipud(pathGoal);

    if ~isempty(pathStart) && ~isempty(pathGoal)
        if norm(pathStart(end,:) - pathGoal(1,:)) < 1e-9
            path = [pathStart; pathGoal(2:end,:)];
        else
            path = [pathStart; pathGoal];
        end
    else
        path = [];
    end

    result.path = path;
    result.pathLength = computePathLength3D(path);
else
    result.path = [];
    result.pathLength = inf;
end

end

% =====================================================================

function [win, switchInc] = updateHysteresisState3D(win, metrics, refPoint, goalPoint, distTree, isStartRoot, params)
% 参数顺序必须与主程序调用一致：
% (..., distTree, isStartRoot, params)

switchInc = 0;

prevState = string(win.state);
newState = prevState;
distGoal = norm(refPoint - goalPoint);

% ---------------- 1. 数据不足保护 ----------------
minSuccForState = max(2, params.N_succ_min_for_pca - 1);
if metrics.N_succ < minSuccForState
    win.freezeCount = win.freezeCount + 1;
    win.metrics = metrics;
    return;
end

% ---------------- 2. Approach 最高优先级 ----------------
if isStartRoot
    approachEnter = (distGoal <= params.approach_enter_distGoal) || ...
                    (distTree <= params.approach_enter_distTree);
else
    approachEnter = (distTree <= params.approach_enter_distTree);
end

if prevState ~= "approach" && approachEnter
    win.lastState = prevState;
    win.state = "approach";
    win.dwellCount = 1;
    win.metrics = metrics;
    switchInc = 1;
    return;
end

% ---------------- 3. 当前状态是否允许离开 ----------------
canLeave = canLeaveState(prevState, win.dwellCount, params);

switch prevState
    case "approach"
        if isStartRoot
            stayCond = (distGoal <= params.approach_exit_distGoal) || ...
                       (distTree <= params.approach_exit_distTree);
        else
            stayCond = (distTree <= params.approach_exit_distTree);
        end

        stayCond = stayCond && (win.failConn < params.connFailThreshold);

        if stayCond || ~canLeave
            win.dwellCount = win.dwellCount + 1;
            win.metrics = metrics;
            return;
        end

    case "blocked"
        enoughRecovery = ...
            (metrics.R_succ >= params.blocked_exit_Rsucc) && ...
            (metrics.E_prog >= params.blocked_exit_Eprog) && ...
            (metrics.dmin   >= params.blocked_exit_dmin);

        if ~enoughRecovery || ~canLeave
            win.dwellCount = win.dwellCount + 1;
            win.metrics = metrics;
            return;
        end

    case "narrow"
        stayCond = ...
            (metrics.dbar    <= params.narrow_exit_dbar) && ...
            (metrics.L_score >= params.narrow_exit_Lscore) && ...
            (win.failConn    <  params.blockedFailConnTrigger);

        if stayCond || ~canLeave
            win.dwellCount = win.dwellCount + 1;
            win.metrics = metrics;
            return;
        end

    case "free"
        stayCond = ...
            (metrics.R_succ >= params.free_exit_Rsucc) && ...
            (metrics.dbar   >= params.free_exit_dbar);

        if stayCond || ~canLeave
            win.dwellCount = win.dwellCount + 1;
            win.metrics = metrics;
            return;
        end
end

% ---------------- 4. 重新判定新状态 ----------------
blockedByMetrics = ...
    (metrics.R_succ <= params.blocked_enter_Rsucc) && ...
    (metrics.E_prog <= params.blocked_enter_Eprog) && ...
    (metrics.dmin   <= params.blocked_enter_dmin);

blockedByFail = ...
    (win.failConn >= params.blockedFailConnTrigger) && ...
    (metrics.dmin <= params.blocked_exit_dmin);

if blockedByMetrics || blockedByFail
    newState = "blocked";

elseif (metrics.dbar <= params.narrow_enter_dbar) && ...
       (metrics.L_score >= params.narrow_enter_Lscore) && ...
       (metrics.R_succ >= params.narrow_enter_Rsucc_low) && ...
       (metrics.R_succ <= params.narrow_enter_Rsucc_high)
    newState = "narrow";

elseif (metrics.R_succ >= params.free_enter_Rsucc) && ...
       (metrics.dbar   >= params.free_enter_dbar)
    newState = "free";

else
    newState = "free";
end

% ---------------- 5. 写回 ----------------
if newState ~= prevState
    win.lastState = prevState;
    win.state = newState;
    win.dwellCount = 1;
    switchInc = 1;
else
    win.dwellCount = win.dwellCount + 1;
end

win.metrics = metrics;

end

function ok = canLeaveState(stateName, dwellCount, params)
switch string(stateName)
    case "free"
        ok = dwellCount >= params.dwell_free;
    case "narrow"
        ok = dwellCount >= params.dwell_narrow;
    case "blocked"
        ok = dwellCount >= params.dwell_blocked;
    case "approach"
        ok = dwellCount >= params.dwell_approach;
    otherwise
        ok = true;
end
end
