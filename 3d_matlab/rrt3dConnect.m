function result = rrt3dConnect(scene, params)
% 三维 RRT-Connect
%
% tree.nodes(i,:) : 第 i 个节点的坐标
% tree.parent(i)  : 第 i 个节点的父节点索引
%
% treeA 从 start 生长
% treeB 从 goal 生长

rng(params.randSeed);

%% 初始化两棵树
treeA.nodes  = scene.start;
treeA.parent = 0;

treeB.nodes  = scene.goal;
treeB.parent = 0;

success = false;
meetIdxA = -1;
meetIdxB = -1;

for iter = 1:params.maxIter

    %% Step 1: 随机采样
    q_rand = sampleFree3D(scene, params.goalSampleRate, scene.goal);

    %% Step 2: 扩展 treeA 一步
    [treeA, reachedA, newIdxA] = extendTreeOnce(treeA, q_rand, scene, params);

    if reachedA
        q_newA = treeA.nodes(newIdxA, :);

        %% Step 3: 用 treeB 朝 q_newA 不断 CONNECT
        [treeB, connected, newIdxB] = connectTree(treeB, q_newA, scene, params);

        if connected
            meetIdxA = newIdxA;
            meetIdxB = newIdxB;
            success = true;
            break;
        end
    end

    %% Step 4: 交换两棵树角色
    tmp = treeA;
    treeA = treeB;
    treeB = tmp;
end

%% 输出结果
result.success  = success;
result.iterUsed = iter;
result.treeA    = treeA;
result.treeB    = treeB;

if success
    % 注意：由于循环里不断交换树，最终 treeA/treeB 不一定分别对应 start/goal
    % 所以要根据根节点来判断哪棵树从 start 开始，哪棵树从 goal 开始。
    rootA = treeA.nodes(1,:);
    rootB = treeB.nodes(1,:);

    if norm(rootA - scene.start) < 1e-9
        startTree = treeA; startMeet = meetIdxA;
        goalTree  = treeB; goalMeet  = meetIdxB;
    else
        startTree = treeB; startMeet = meetIdxB;
        goalTree  = treeA; goalMeet  = meetIdxA;
    end

    pathStart = extractPathFromTree3D(startTree, startMeet); % start -> meet
    pathGoal  = extractPathFromTree3D(goalTree,  goalMeet);  % goalRoot -> meet

    % pathGoal 当前是 goal -> meet，需要翻转成 meet -> goal
    pathGoal = flipud(pathGoal);

    % 拼接时去掉重复的会合点
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
    result.pathLength = computePathLength(path);
else
    result.path = [];
    result.pathLength = inf;
end

end

%% =========================================================
function [tree, reached, newIdx] = extendTreeOnce(tree, q_target, scene, params)
% 向 q_target 扩展一步

reached = false;
newIdx = -1;

idxNear = findNearestNode3D(tree.nodes, q_target);
q_near  = tree.nodes(idxNear, :);
q_new   = steer3D(q_near, q_target, params.stepLen);

% 与原点重合就不扩展
if norm(q_new - q_near) < 1e-10
    return;
end

% 点碰撞
if checkPointCollision3D(q_new, scene)
    return;
end

% 线段碰撞
if checkSegmentCollision3D(q_near, q_new, scene, params.collisionStep)
    return;
end

% 加入树
tree.nodes(end+1, :) = q_new;
tree.parent(end+1,1) = idxNear;
newIdx = size(tree.nodes, 1);
reached = true;

end

%% =========================================================
function [tree, connected, newIdx] = connectTree(tree, q_target, scene, params)
% 持续朝 q_target 方向扩展，直到：
% 1) 碰撞失败 -> trapped
% 2) 足够接近 q_target -> connected

connected = false;
newIdx = -1;

while true
    idxNear = findNearestNode3D(tree.nodes, q_target);
    q_near  = tree.nodes(idxNear, :);

    % 若已经很近，尝试直接连接
    distToTarget = norm(q_target - q_near);
    if distToTarget <= params.connectThresh
        if ~checkSegmentCollision3D(q_near, q_target, scene, params.collisionStep)
            tree.nodes(end+1, :) = q_target;
            tree.parent(end+1,1) = idxNear;
            newIdx = size(tree.nodes, 1);
            connected = true;
        end
        return;
    end

    % 朝目标扩展一步
    q_new = steer3D(q_near, q_target, params.stepLen);

    if norm(q_new - q_near) < 1e-10
        return;
    end

    % 点碰撞
    if checkPointCollision3D(q_new, scene)
        return;
    end

    % 线段碰撞
    if checkSegmentCollision3D(q_near, q_new, scene, params.collisionStep)
        return;
    end

    % 加入树
    tree.nodes(end+1, :) = q_new;
    tree.parent(end+1,1) = idxNear;
    newIdx = size(tree.nodes, 1);

    % 如果已经足够接近 q_target，尝试直接连接
    if norm(q_new - q_target) <= params.connectThresh
        if ~checkSegmentCollision3D(q_new, q_target, scene, params.collisionStep)
            tree.nodes(end+1, :) = q_target;
            tree.parent(end+1,1) = newIdx;
            newIdx = size(tree.nodes, 1);
            connected = true;
        end
        return;
    end
end

end

%% =========================================================
function L = computePathLength(path)
L = 0;
for i = 1:size(path,1)-1
    L = L + norm(path(i+1,:) - path(i,:));
end
end