function [tree, reached, newIdx, modeName, failCount, stats, stepLenUsed, deltaProg, clearVal, qualityVal] = ...
    extendByMode3D(tree, q_target, scene, params, failCount, stats, treeTag, forcedState, metrics, win)

% 保持与当前 smhBiRRTConnect3D.m 的旧接口完全兼容
% 输出顺序：
% [tree, reached, newIdx, modeName, failCount, stats, stepLenUsed, deltaProg, clearVal, qualityVal]

reached = false;
newIdx = -1;
stepLenUsed = params.baseStepLen;
deltaProg = 0;
clearVal = inf;
qualityVal = 0;
modeName = string(forcedState);

idxNear = findNearestNode3D(tree.nodes, q_target);
q_near  = tree.nodes(idxNear, :);
idxParent = idxNear;
q_from = q_near;

% ---------- 自适应步长 ----------
stepLenBase = params.baseStepLen ...
    + params.stepWeightClear * clipFinite(metrics.dbar, 0, 20) ...
    + params.stepWeightProg  * clipFinite(metrics.E_prog, 0, 10) ...
    - params.stepWeightDens  * clipFinite(metrics.rho_local, 0, 1);

idxExpand = idxNear;
q_from = q_near;
q_bias = q_target;
stepLen = params.baseStepLen;

switch modeName
    case "free"
        stepLen = min(max(stepLenBase, 0.90 * params.baseStepLen), params.maxStepLen);
        q_bias = q_target;

    case "narrow"
        stepLen = min(max(stepLenBase, params.minStepLen), 0.75 * params.baseStepLen);

        dirVec = computePcaDirection3D(win.successNodes);
        if isempty(dirVec)
            q_bias = q_target;
        else
            toTarget = q_target - q_from;
            if dot(dirVec, toTarget) < 0
                dirVec = -dirVec;
            end
            q_bias = q_from + params.narrowLookAheadFactor * stepLen * dirVec ...
                             + params.narrowDirectionalNoise * randn(1,3);
        end

    case "blocked"
    stepLen = min(max(stepLenBase, params.minStepLen), 0.70 * params.baseStepLen);

    [idxParent, q_from] = selectAncestorNode3D(tree, idxNear, params.blockedBackSteps);

    dirToTarget = q_target - q_from;
    if norm(dirToTarget) > 1e-10
        dirToTarget = dirToTarget / norm(dirToTarget);
    else
        dirToTarget = randn(1,3);
        dirToTarget = dirToTarget / max(norm(dirToTarget), 1e-10);
    end

    q_bias = q_from + stepLen * dirToTarget + params.blockedLateralNoise * randn(1,3);

    case "approach"
        stepLen = min(max(stepLenBase, params.minStepLen), 0.65 * params.baseStepLen);
        q_bias = q_target;

    otherwise
        stepLen = params.baseStepLen;
        q_bias = q_target;
end

stepLen = min(max(stepLen, params.minStepLen), params.maxStepLen);
stepLenUsed = stepLen;

for k = 1:3
    q_bias(k) = min(max(q_bias(k), scene.boundary(k,1)), scene.boundary(k,2));
end

q_new = steer3D(q_from, q_bias, stepLen);

if norm(q_new - q_from) < 1e-10
    failCount = failCount + 1;
    return;
end

if checkPointCollision3D(q_new, scene)
    failCount = failCount + 1;
    return;
end

if checkSegmentCollision3D(q_from, q_new, scene, params.collisionStep)
    failCount = failCount + 1;
    return;
end

deltaProg = norm(q_target - q_from) - norm(q_target - q_new);
clearVal = estimateSegmentClearance3D( ...
    q_from, q_new, scene, max(0.8, params.collisionStep));

[qualityVal, accepted] = evaluateNodeQuality3D(deltaProg, clearVal, metrics.rho_local, params);
if ~accepted
    failCount = failCount + 1;
    return;
end

% 添加 q_new
tree.nodes(end+1, :)  = q_new;
tree.parent(end+1,1) = idxParent;
if isfield(tree, 'mode')
    tree.mode{end+1, 1} = char(modeName);
end

newIdx = size(tree.nodes, 1);
reached = true;
failCount = 0;

if isfield(stats, char(modeName))
    stats.(char(modeName)) = stats.(char(modeName)) + 1;
end

% 只在 Approach 下保留一次额外直连尝试
if modeName == "approach"
    if norm(q_target - q_new) > 1e-10
        if ~checkPointCollision3D(q_target, scene) && ...
           ~checkSegmentCollision3D(q_new, q_target, scene, params.collisionStep)

            clearVal2 = estimateSegmentClearance3D(q_new, q_target, scene, max(0.25, params.collisionStep/2));
            deltaProg2 = norm(q_target - q_new);

            [qualityVal2, accepted2] = evaluateNodeQuality3D(deltaProg2, clearVal2, metrics.rho_local, params);
            if accepted2
                tree.nodes(end+1, :)  = q_target;
                tree.parent(end+1, 1) = newIdx;
                if isfield(tree, 'mode')
                    tree.mode{end+1, 1} = char(modeName);
                end
                newIdx = size(tree.nodes, 1);
                clearVal = min(clearVal, clearVal2);
                qualityVal = max(qualityVal, qualityVal2);

                if isfield(stats, char(modeName))
                    stats.(char(modeName)) = stats.(char(modeName)) + 1;
                end
            end
        end
    end
end

end

% ==================== 辅助函数 ====================

function x = clipFinite(x, lo, hi)
if ~isfinite(x)
    x = lo;
end
x = min(max(x, lo), hi);
end
