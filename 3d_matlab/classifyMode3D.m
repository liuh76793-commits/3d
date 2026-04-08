function modeName = classifyMode3D(q_near, q_target, scene, params, failCount, allowApproach)
% 修正版：
% 1) blocked 优先于 narrow
% 2) approach 只在允许时触发
% 3) approach 阈值更严格

clearance = estimateClearance3D(q_near, scene);
distToGoal = norm(q_near - scene.goal);

if failCount >= params.blockedTrialLimit
    modeName = "blocked";
    return;
end

if allowApproach && distToGoal <= params.approachDistTh && clearance <= params.freeClearanceTh
    modeName = "approach";
    return;
end

if clearance <= params.narrowClearanceTh
    modeName = "narrow";
    return;
end

modeName = "free";
end