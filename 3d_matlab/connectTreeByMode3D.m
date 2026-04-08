function [tree, connected, newIdx, failCount, stats, stepLenUsed, deltaProg, clearVal, qualityVal] = ...
    connectTreeByMode3D(tree, q_target, scene, params, failCount, stats, treeTag, forcedState, metrics)
% connectTreeByMode3D
% 按当前模式执行 connect：
% 1) Free / Narrow / Blocked / Approach 使用不同的步长与偏置策略
% 2) 只在 Approach 下启用“强化直连”
% 3) 节点质量评价不再只看终点 clearance，而看整段边的最小 clearance

connected   = false;
newIdx      = -1;
stepLenUsed = params.baseStepLen;
deltaProg   = 0;
clearVal    = inf;
qualityVal  = 0;

modeName = string(forcedState);

% 只在 Approach 下启用强化直连
aggressiveConnect = (modeName == "approach");

while true
    idxNear = findNearestNode3D(tree.nodes, q_target);
    q_near  = tree.nodes(idxNear, :);

    % 自适应基础步长
    stepLenBase = params.baseStepLen ...
        + params.stepWeightClear * clipFinite(metrics.dbar,      0, 20) ...
        + params.stepWeightProg  * clipFinite(metrics.E_prog,    0, 10) ...
        - params.stepWeightDens  * clipFinite(metrics.rho_local, 0,  1);

    switch modeName
        case "free"
            stepLen = min(max(stepLenBase, params.baseStepLen), params.maxStepLen);
            q_bias  = q_target;

        case "narrow"
            % Narrow 里 connect 仍然以目标点为偏置，但步长更小
            stepLen = min(max(stepLenBase, params.minStepLen), 0.70 * params.baseStepLen);
            q_bias  = q_target;

        case "blocked"
            stepLen = min(max(stepLenBase, params.minStepLen), 0.65 * params.baseStepLen);

            [~, ancestorNode] = selectAncestorNode3D(tree, idxNear, params.blockedBackSteps);
            dirToTarget = q_target - ancestorNode;
            if norm(dirToTarget) > 1e-10
                dirToTarget = dirToTarget / norm(dirToTarget);
            else
                dirToTarget = randn(1,3);
                dirToTarget = dirToTarget / max(norm(dirToTarget), 1e-10);
            end

            q_bias = ancestorNode ...
                + stepLen * dirToTarget ...
                + params.blockedLateralNoise * randn(1,3);

        case "approach"
            stepLen = min(max(stepLenBase, params.minStepLen), 0.60 * params.baseStepLen);
            q_bias  = q_target;

        otherwise
            stepLen = params.baseStepLen;
            q_bias  = q_target;
    end

    stepLen = min(max(stepLen, params.minStepLen), params.maxStepLen);
    stepLenUsed = stepLen;

    % 裁剪到边界内
    for k = 1:3
        q_bias(k) = min(max(q_bias(k), scene.boundary(k,1)), scene.boundary(k,2));
    end

    distToTarget = norm(q_target - q_near);

    % ==========================================================
    % 1) 只在 Approach 下，优先尝试整段直连 q_target
    % ==========================================================
    if aggressiveConnect
        if ~checkSegmentCollision3D(q_near, q_target, scene, params.collisionStep)
            deltaProgTry = distToTarget;

            % 关键修改：用整段最小 clearance，而不是只看 q_target 点
            clearValTry = estimateSegmentClearance3D( ...
                q_near, q_target, scene, max(0.25, params.collisionStep/2));

            [qualityTry, acceptedTry] = evaluateNodeQuality3D( ...
                deltaProgTry, clearValTry, metrics.rho_local, params);

            if acceptedTry
                tree.nodes(end+1,:)  = q_target;
                tree.parent(end+1,1) = idxNear;
                tree.mode{end+1,1}   = char(modeName);

                newIdx     = size(tree.nodes,1);
                connected  = true;
                failCount  = 0;
                deltaProg  = deltaProgTry;
                clearVal   = clearValTry;
                qualityVal = qualityTry;

                stats.(char(modeName)) = stats.(char(modeName)) + 1;
                return;
            end
        end
    end

    % ==========================================================
    % 2) 原有近距离直连
    % ==========================================================
    if distToTarget <= params.connectThresh
        if ~checkSegmentCollision3D(q_near, q_target, scene, params.collisionStep)
            deltaProg = distToTarget;

            clearVal = estimateSegmentClearance3D( ...
                q_near, q_target, scene, max(0.25, params.collisionStep/2));

            [qualityVal, accepted] = evaluateNodeQuality3D( ...
                deltaProg, clearVal, metrics.rho_local, params);

            if accepted
                tree.nodes(end+1,:)  = q_target;
                tree.parent(end+1,1) = idxNear;
                tree.mode{end+1,1}   = char(modeName);

                newIdx    = size(tree.nodes,1);
                connected = true;
                failCount = 0;

                stats.(char(modeName)) = stats.(char(modeName)) + 1;
            else
                failCount = failCount + 1;
            end
        else
            failCount = failCount + 1;
        end
        return;
    end

    % ==========================================================
    % 3) 普通一步扩展
    % ==========================================================
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

    % 关键修改：用整段边最小 clearance
    clearVal = estimateSegmentClearance3D( ...
        q_near, q_new, scene, max(0.25, params.collisionStep/2));

    [qualityVal, accepted] = evaluateNodeQuality3D( ...
        deltaProg, clearVal, metrics.rho_local, params);

    if ~accepted
        failCount = failCount + 1;
        return;
    end

    tree.nodes(end+1,:)  = q_new;
    tree.parent(end+1,1) = idxNear;
    tree.mode{end+1,1}   = char(modeName);

    newIdx = size(tree.nodes,1);
    stats.(char(modeName)) = stats.(char(modeName)) + 1;
    failCount = 0;

    % ==========================================================
    % 4) 只在 Approach 下，走一步后再尝试一次直接连 q_target
    % ==========================================================
    if aggressiveConnect
        if ~checkSegmentCollision3D(q_new, q_target, scene, params.collisionStep)
            deltaProgTry = norm(q_target - q_new);

            clearValTry = estimateSegmentClearance3D( ...
                q_new, q_target, scene, max(0.25, params.collisionStep/2));

            [qualityTry, acceptedTry] = evaluateNodeQuality3D( ...
                deltaProgTry, clearValTry, metrics.rho_local, params);

            if acceptedTry
                tree.nodes(end+1,:)  = q_target;
                tree.parent(end+1,1) = newIdx;
                tree.mode{end+1,1}   = char(modeName);

                newIdx     = size(tree.nodes,1);
                connected  = true;
                deltaProg  = deltaProgTry;
                clearVal   = clearValTry;
                qualityVal = qualityTry;

                stats.(char(modeName)) = stats.(char(modeName)) + 1;
                return;
            end
        end
    end

    % ==========================================================
    % 5) 原有 connectThresh 判定
    % ==========================================================
    if norm(q_new - q_target) <= params.connectThresh
        if ~checkSegmentCollision3D(q_new, q_target, scene, params.collisionStep)
            deltaProg2 = norm(q_target - q_new);

            clearVal2 = estimateSegmentClearance3D( ...
                q_new, q_target, scene, max(0.25, params.collisionStep/2));

            [qualityVal2, accepted2] = evaluateNodeQuality3D( ...
                deltaProg2, clearVal2, metrics.rho_local, params);

            if accepted2
                tree.nodes(end+1,:)  = q_target;
                tree.parent(end+1,1) = newIdx;
                tree.mode{end+1,1}   = char(modeName);

                newIdx     = size(tree.nodes,1);
                connected  = true;
                deltaProg  = deltaProg2;
                clearVal   = clearVal2;
                qualityVal = qualityVal2;

                stats.(char(modeName)) = stats.(char(modeName)) + 1;
            end
        end
        return;
    end

    % 如果还没连上，while true 继续下一步 connect
end

end

function v = clipFinite(x, lo, hi)
if isinf(x) || isnan(x)
    v = lo;
else
    v = min(max(x, lo), hi);
end
end