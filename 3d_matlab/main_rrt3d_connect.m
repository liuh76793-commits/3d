clc; clear; close all;

scene = createSceneS1();

params.maxIter        = 5000;
params.stepLen        = 3.2;
params.goalSampleRate = 0.05;
params.connectThresh  = 7.0;
params.collisionStep  = 0.40;
params.randSeed       = 1;

rng(params.randSeed);

tic;
result = rrt3dConnect(scene, params);
tPlan = toc;

fprintf('\n========== RRT-Connect 3D Result ==========\n');
fprintf('是否成功: %d\n', result.success);
fprintf('规划时间/s: %.4f\n', tPlan);

if isfield(result, 'iterUsed')
    fprintf('使用迭代次数: %d\n', result.iterUsed);
end
if isfield(result, 'treeA') && isfield(result.treeA, 'nodes')
    fprintf('Tree A 节点数: %d\n', size(result.treeA.nodes,1));
end
if isfield(result, 'treeB') && isfield(result.treeB, 'nodes')
    fprintf('Tree B 节点数: %d\n', size(result.treeB.nodes,1));
end

if result.success && isfield(result, 'path') && ~isempty(result.path)
    pathLen = 0;
    for i = 2:size(result.path,1)
        pathLen = pathLen + norm(result.path(i,:) - result.path(i-1,:));
    end
    fprintf('路径长度: %.4f\n', pathLen);

    minClr = inf;
    for i = 1:size(result.path,1)
        minClr = min(minClr, estimateClearance3D(result.path(i,:), scene));
    end
    fprintf('路径最小安全裕度: %.4f\n', minClr);
else
    fprintf('未找到可行路径。\n');
end

figure('Color','w'); hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('RRT-Connect 3D');

drawScene3D(scene, 'perspective');

if isfield(result, 'treeA') && isfield(result.treeA, 'nodes')
    drawTree3D(result.treeA, [0.5 0.7 1.0]);
end
if isfield(result, 'treeB') && isfield(result.treeB, 'nodes')
    drawTree3D(result.treeB, [1.0 0.7 0.5]);
end

if result.success && isfield(result, 'path') && ~isempty(result.path)
    plot3(result.path(:,1), result.path(:,2), result.path(:,3), 'k-', 'LineWidth', 2.2);
    scatter3(result.path(1,1), result.path(1,2), result.path(1,3), 60, 'g', 'filled');
    scatter3(result.path(end,1), result.path(end,2), result.path(end,3), 60, 'r', 'filled');
end

view(3);

% ===== 辅助函数 =====
function drawTree3D(tree, edgeColor)
for i = 2:size(tree.nodes,1)
    p = tree.parent(i);
    if p > 0
        a = tree.nodes(i,:);
        b = tree.nodes(p,:);
        plot3([a(1), b(1)], [a(2), b(2)], [a(3), b(3)], '-', ...
            'Color', edgeColor, 'LineWidth', 0.6);
    end
end
end
