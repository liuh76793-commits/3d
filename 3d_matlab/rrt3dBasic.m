function result = rrt3dBasic(scene, params)
% 基础版三维 RRT
%
% tree.nodes(i,:)   : 第 i 个节点坐标 [x y z]
% tree.parent(i)    : 第 i 个节点的父节点编号
%
% result.success    : 是否成功
% result.path       : 最终路径（从 start 到 goal）
% result.pathLength : 路径长度
% result.tree       : 搜索树
% result.iterUsed   : 实际使用迭代次数

rng(params.randSeed);

%% 初始化树
tree.nodes  = scene.start;
tree.parent = 0;   % 根节点没有父节点

success   = false;
goalIndex = -1;

for iter = 1:params.maxIter

    %% 1. 随机采样
    q_rand = sampleFree3D(scene, params.goalSampleRate, scene.goal);

    %% 2. 最近节点搜索
    idxNear = findNearestNode3D(tree.nodes, q_rand);
    q_near  = tree.nodes(idxNear, :);

    %% 3. 按固定步长扩展
    q_new = steer3D(q_near, q_rand, params.stepLen);

    %% 4. 新点碰撞检测
    if checkPointCollision3D(q_new, scene)
        continue;
    end

    %% 5. 线段碰撞检测
    if checkSegmentCollision3D(q_near, q_new, scene, params.collisionStep)
        continue;
    end

    %% 6. 加入树
    tree.nodes(end+1, :) = q_new;
    tree.parent(end+1,1) = idxNear;
    idxNew = size(tree.nodes, 1);

    %% 7. 判断是否接近目标
    if norm(q_new - scene.goal) <= params.goalThreshold

        % 尝试从 q_new 直接连接到 goal
        if ~checkSegmentCollision3D(q_new, scene.goal, scene, params.collisionStep)
            tree.nodes(end+1, :) = scene.goal;
            tree.parent(end+1,1) = idxNew;
            goalIndex = size(tree.nodes, 1);
            success = true;
            break;
        end
    end
end

%% 输出结果
result.tree     = tree;
result.success  = success;
result.iterUsed = iter;

if success
    path = extractPath3D(tree, goalIndex);
    result.path = path;
    result.pathLength = computePathLength(path);
else
    result.path = [];
    result.pathLength = inf;
end

end

%% =========================================
function L = computePathLength(path)
L = 0;
for i = 1:size(path,1)-1
    L = L + norm(path(i+1,:) - path(i,:));
end
end