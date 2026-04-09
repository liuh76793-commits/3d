function [tree, connected, newIdx, failCount, stats, stepLenUsed, deltaProg, clearVal, qualityVal] = ...
    connectTreeByMode3D(tree, q_target, scene, params, failCount, stats, treeTag, forcedState, metrics)

% 单次 bounded connect：
% 1) 只找一次最近邻
% 2) 最多扩展一步
% 3) 若已接近目标，再尝试一次短距离直连
% 这样避免原来 while true + 全树重复最近邻搜索导致的超慢问题

connected = false;
newIdx = -1;
stepLenUsed = params.baseStepLen;
deltaProg = 0;
clearVal = inf;
qualityVal = 0;

modeName = string(forcedState);

% ---------- 1. 找最近节点（只做一次） ----------
idxNear = findNearestNode3D(tree.nodes, q_target);
q_near  = tree.nodes(idxNear, :);

idxParent = idxNear;
q_from = q_near;
q_bias = q_target;

% ---------- 2. 自适应步长 ----------
stepLenBase = params.baseStepLen ...
    + params.stepWeightClear * clipFinite(metrics.dbar, 0, 20) ...
    + params.stepWeightProg  * clipFinite(metrics.E_prog, 0, 10) ...
    - params.stepWeightDens  * clipFinite(metrics.rho_local, 0, 1);

switch modeName
    case "free"
        stepLen = min(max(stepLenBase, params.baseStepLen), params.maxStepLen);
        q_bias = q_target;

    case "narrow"
        stepLen = min(max(stepLenBase, params.minStepLen), 0.70 * params.baseStepLen);
        q_bias = q_target;

    case "blocked"
        stepLen = min(max(stepLenBase, params.minStepLen), 0.65 * params.baseStepLen);

        % 真正从祖先节点重新出发
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
        stepLen = min(max(stepLenBase, params.minStepLen), 0.60 * params.baseStepLen);
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

distToTarget = norm(q_target - q_from);

% ---------- 3. 仅在真正接近时才试一次直连 ----------
if (modeName == "approach") && (distToTarget <= 1.2 * params.connectThresh)
    if ~checkSegmentCollision3D(q_from, q_target, scene, params.collisionStep)
        deltaProgTry = distToTarget;

        % 改成看整段 clearance，而不是只看终点
        clearValTry = estimateSegmentClearance3D( ...
            q_from, q_target, scene, max(0.8, params.collisionStep));

        [qualityTry, acceptedTry] = evaluateNodeQuality3D( ...
            deltaProgTry, clearValTry, metrics.rho_local, params);

        if acceptedTry && (clearValTry >= params.minAcceptClearance)
            tree.nodes(end+1,:) = q_target;
            tree.parent(end+1,1) = idxParent;
            if isfield(tree, 'mode')
                tree.mode{end+1,1} = char(modeName);
            end

            newIdx = size(tree.nodes,1);
            connected = true;
            failCount = 0;
            deltaProg = deltaProgTry;
            clearVal = clearValTry;
            qualityVal = qualityTry;

            if isfield(stats, char(modeName))
                stats.(char(modeName)) = stats.(char(modeName)) + 1;
            end
            return;
        end
    end
end

% ---------- 4. 普通一步扩展 ----------
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

% 用点 clearance，先把速度恢复
clearVal = estimateSegmentClearance3D( ...
    q_from, q_new, scene, max(0.8, params.collisionStep));

[qualityVal, accepted] = evaluateNodeQuality3D( ...
    deltaProg, clearVal, metrics.rho_local, params);

if ~accepted
    failCount = failCount + 1;
    return;
end

tree.nodes(end+1,:) = q_new;
tree.parent(end+1,1) = idxParent;
if isfield(tree, 'mode')
    tree.mode{end+1,1} = char(modeName);
end

newIdx = size(tree.nodes,1);
failCount = 0;

if isfield(stats, char(modeName))
    stats.(char(modeName)) = stats.(char(modeName)) + 1;
end

% ---------- 5. 走完一步后，若已靠近，再补一次直连 ----------
if norm(q_new - q_target) <= params.connectThresh
    if ~checkSegmentCollision3D(q_new, q_target, scene, params.collisionStep)
        deltaProg2 = norm(q_target - q_new);
        clearVal2 = estimateSegmentClearance3D( ...
    q_new, q_target, scene, max(0.8, params.collisionStep));

        [qualityVal2, accepted2] = evaluateNodeQuality3D( ...
            deltaProg2, clearVal2, metrics.rho_local, params);

        if accepted2
            tree.nodes(end+1,:) = q_target;
            tree.parent(end+1,1) = newIdx;
            if isfield(tree, 'mode')
                tree.mode{end+1,1} = char(modeName);
            end

            newIdx = size(tree.nodes,1);
            connected = true;
            deltaProg = deltaProg2;
            clearVal = clearVal2;
            qualityVal = qualityVal2;

            if isfield(stats, char(modeName))
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