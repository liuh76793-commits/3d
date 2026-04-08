function q_rand = sampleBiased3D(scene, params, treeA, treeB)
% sampleBiased3D
% 修正版：
% 1) 当前扩展树若根在 start，则偏向 goal
% 2) 当前扩展树若根在 goal，则偏向 start
% 3) 仍保留两树中间区域采样，利于会合

r = rand;

isTreeAStart = (norm(treeA.nodes(1,:) - scene.start) < 1e-9);

if isTreeAStart
    targetBias = scene.goal;
else
    targetBias = scene.start;
end

% 1) 树特异的终点偏置
if r < params.goalSampleRate
    q_rand = targetBias;
else
    % 2) 朝两树中间区域采样，促进会合
    if r < params.goalSampleRate + 0.15 && size(treeA.nodes,1) > 1 && size(treeB.nodes,1) > 1
        idxA = randi(size(treeA.nodes,1));
        idxB = randi(size(treeB.nodes,1));
        mid = 0.5 * (treeA.nodes(idxA,:) + treeB.nodes(idxB,:));
        noise = 4 * randn(1,3);
        q_rand = mid + noise;
    else
        q_rand = sampleFree3D(scene, 0.0, targetBias);
    end
end

% 裁剪到边界
for k = 1:3
    q_rand(k) = min(max(q_rand(k), scene.boundary(k,1)), scene.boundary(k,2));
end
end