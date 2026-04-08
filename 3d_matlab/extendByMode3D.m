function [tree, reached, newIdx, modeName, failCount, stats, stepLenUsed, deltaProg, clearVal, qualityVal] = ...
    extendByMode3D(tree, q_target, scene, params, failCount, stats, treeTag, forcedState, metrics, win)

reached = false;
newIdx = -1;
stepLenUsed = params.baseStepLen;
deltaProg = 0;
clearVal = inf;
qualityVal = 0;

idxNear = findNearestNode3D(tree.nodes, q_target);
q_near  = tree.nodes(idxNear, :);

modeName = string(forcedState);

% ---------- 自适应步长 ----------
stepLenBase = params.baseStepLen ...
    + params.stepWeightClear * clipFinite(metrics.dbar, 0, 20) ...
    + params.stepWeightProg  * clipFinite(metrics.E_prog, 0, 10) ...
    - params.stepWeightDens  * clipFinite(metrics.rho_local, 0, 1);

switch modeName
    case "free"
        stepLen = min(max(stepLenBase, 0.90 * params.baseStepLen), params.maxStepLen);
        q_bias  = q_target;

    case "narrow"
        stepLen = min(max(stepLenBase, params.minStepLen), 0.75 * params.baseStepLen);

        dirVec = computePcaDirection3D(win.successNodes);
        if isempty(dirVec)
            q_bias = q_target;
        else
            toTarget = q_target - q_near;
            if dot(dirVec, toTarget) < 0
                dirVec = -dirVec;
            end

            q_bias = q_near + params.narrowLookAheadFactor * stepLen * dirVec ...
                   + params.narrowDirectionalNoise * randn(1,3);
        end

    case "blocked"
        stepLen = min(max(stepLenBase, params.minStepLen), 0.70 * params.baseStepLen);

        [~, ancestorNode] = selectAncestorNode3D(tree, idxNear, params.blockedBackSteps);

        dirToTarget = q_target - ancestorNode;
        if norm(dirToTarget) > 1e-10
            dirToTarget = dirToTarget / norm(dirToTarget);
        else
            dirToTarget = randn(1,3);
            dirToTarget = dirToTarget / norm(dirToTarget);
        end

        q_bias = ancestorNode + stepLen * dirToTarget + params.blockedLateralNoise * randn(1,3);

    case "approach"
        stepLen = min(max(stepLenBase, params.minStepLen), 0.65 * params.baseStepLen);
        q_bias  = q_target;

    otherwise
        stepLen = params.baseStepLen;
        q_bias  = q_target;
end

stepLen = min(max(stepLen, params.minStepLen), params.maxStepLen);
stepLenUsed = stepLen;

for k = 1:3
    q_bias(k) = min(max(q_bias(k), scene.boundary(k,1)), scene.boundary(k,2));
end

q_new = steer3D(q_near, q_bias, stepLen);

if norm(q_new - q_near) < 1e-10
    failCount = failCount + 1;
    return;
end

if checkPointCollision3D(q_new, scene)
    failCount = failCount + 1;
    return;
end

if checkSegmentCollision3D(q_near, q_new, scene, params.collisionStep)
    failCount = failCount + 1;
    return;
end

deltaProg = norm(q_target - q_near) - norm(q_target - q_new);

% 关键：用整段边最小 clearance，而不是只看 q_new 点
clearVal = estimateSegmentClearance3D(q_near, q_new, scene, max(0.25, params.collisionStep/2));

[qualityVal, accepted] = evaluateNodeQuality3D(deltaProg, clearVal, metrics.rho_local, params);

if ~accepted
    failCount = failCount + 1;
    return;
end

tree.nodes(end+1, :) = q_new;
tree.parent(end+1,1) = idxNear;
tree.mode{end+1,1}   = char(modeName);

newIdx = size(tree.nodes, 1);
reached = true;
failCount = 0;

stats.(char(modeName)) = stats.(char(modeName)) + 1;

% ==========================================================
% 只在 Approach 下保留强化直接连通
% Free 不再做这个额外的大跨度直连
% ==========================================================
if modeName == "approach"
    if norm(q_target - q_new) > 1e-10
        if ~checkPointCollision3D(q_target, scene) && ...
           ~checkSegmentCollision3D(q_new, q_target, scene, params.collisionStep)

            deltaProg2 = norm(q_target - q_new);
            clearVal2  = estimateClearance3D(q_target, scene);

            [qualityVal2, accepted2] = evaluateNodeQuality3D(deltaProg2, clearVal2, metrics.rho_local, params);

            if accepted2
                tree.nodes(end+1,:)  = q_target;
                tree.parent(end+1,1) = newIdx;
                tree.mode{end+1,1}   = char(modeName);

                newIdx = size(tree.nodes,1);

                deltaProg  = deltaProg2;
                clearVal   = clearVal2;
                qualityVal = qualityVal2;

                stats.(char(modeName)) = stats.(char(modeName)) + 1;
            end
        end
    end
end

end

function v = clipFinite(x, lo, hi)
if isinf(x) || isnan(x)
    v = lo;
else
    v = min(max(x, lo), hi);
end
end