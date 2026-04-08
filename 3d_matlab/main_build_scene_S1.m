clc; clear; close all;

%% 构建新版 S1 场景
scene = createSceneS1();

%% 透视图
figure('Color','w','Name','S1 Scene - Perspective');
drawScene3D(scene, 'perspective');

%% 俯视图
figure('Color','w','Name','S1 Scene - Top View');
drawScene3D(scene, 'top');

%% 侧视图
figure('Color','w','Name','S1 Scene - Side View');
drawScene3D(scene, 'side');

%% 简单碰撞测试
fprintf('--- 点碰撞测试 ---\n');
pt1 = scene.start;
pt2 = scene.goal;
pt3 = [50, 50, 50];
fprintf('start 是否碰撞: %d\n', checkPointCollision3D(pt1, scene));
fprintf('goal  是否碰撞: %d\n', checkPointCollision3D(pt2, scene));
fprintf('[50,50,50] 是否碰撞: %d\n', checkPointCollision3D(pt3, scene));

fprintf('--- 线段碰撞测试 ---\n');
flagSeg = checkSegmentCollision3D(scene.start, scene.goal, scene, 1.0);
fprintf('Start -> Goal 是否碰撞: %d\n', flagSeg);