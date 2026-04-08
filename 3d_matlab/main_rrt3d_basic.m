clc; clear; close all;

%% 1. 构建场景
scene = createSceneS1();

%% 2. RRT 参数设置
params.maxIter        = 5000;   % 最大迭代次数
params.stepLen        = 4.0;    % 单步扩展长度
params.goalSampleRate = 0.10;   % 目标偏置采样概率
params.goalThreshold  = 5.0;    % 判定接近目标的阈值
params.collisionStep  = 1.0;    % 线段碰撞检测离散步长
params.randSeed       = 1;      % 随机种子，便于复现

%% 3. 执行基础版 RRT
result = rrt3dBasic(scene, params);

%% 4. 绘图显示
figure('Color','w','Name','Basic RRT in S1');
drawScene3D(scene, 'perspective');
hold on;

% 画搜索树
plotTree3D(result.tree, [0.2 0.4 1.0]);

% 画最终路径
if result.success
    plotPath3D(result.path, [1 0 0], 3.0);
    title(sprintf('Basic RRT Success | Iter = %d | Path Nodes = %d | Path Length = %.2f', ...
        result.iterUsed, size(result.path,1), result.pathLength));
else
    title(sprintf('Basic RRT Failed | Iter = %d', result.iterUsed));
end

%% 5. 输出结果
fprintf('\n========== Basic RRT Result ==========\n');
fprintf('是否成功: %d\n', result.success);
fprintf('使用迭代次数: %d\n', result.iterUsed);
fprintf('树节点数: %d\n', size(result.tree.nodes,1));
if result.success
    fprintf('路径节点数: %d\n', size(result.path,1));
    fprintf('路径长度: %.4f\n', result.pathLength);
else
    fprintf('未找到可行路径。\n');
end
fprintf('======================================\n');