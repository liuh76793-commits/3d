function seedPath = smoothSeedPath3D(path, scene, collisionStep, smoothIters, minSafeClearance)
% 生成安全感知平滑 seed
%
% 输入：
%   path             : 输入路径（通常是 prunePath）
%   scene            : 场景
%   collisionStep    : 碰撞检测步长
%   smoothIters      : 平滑迭代次数
%   minSafeClearance : 最小安全裕度门槛
%
% 输出：
%   seedPath         : 平滑后的 seed；若平滑后过于危险，则退回原输入路径

if nargin < 3
    collisionStep = 1.0;
end
if nargin < 4
    smoothIters = 3;
end
if nargin < 5
    minSafeClearance = 1.0;
end

if isempty(path) || size(path,1) <= 2
    seedPath = path;
    return;
end

% 保存输入路径，若全局安全检查失败则回退
fallbackPath = path;

% 先重采样，便于平滑
seedPath = resamplePath3D(path, 2.0);

for iter = 1:smoothIters
    newPath = seedPath;

    for i = 2:size(seedPath,1)-1
        pPrev = seedPath(i-1,:);
        pCurr = seedPath(i,:);
        pNext = seedPath(i+1,:);

        % 邻域平滑候选点
        candidate = 0.25 * pPrev + 0.50 * pCurr + 0.25 * pNext;

        % 1) 点本身不能碰撞
        if checkPointCollision3D(candidate, scene)
            continue;
        end

        % 2) 与前后点连接不能碰撞
        if checkSegmentCollision3D(pPrev, candidate, scene, collisionStep)
            continue;
        end
        if checkSegmentCollision3D(candidate, pNext, scene, collisionStep)
            continue;
        end

        % 3) 候选点安全裕度门槛
        candClear = estimateClearance3D(candidate, scene);
        if candClear < minSafeClearance
            continue;
        end

        % 4) 不允许比当前点安全裕度掉太多
        currClear = estimateClearance3D(pCurr, scene);
        if candClear < 0.75 * currClear
            continue;
        end

        newPath(i,:) = candidate;
    end

    seedPath = newPath;
end

% ===== 新增：全局安全检查 =====
[minSeedClr, ~, ~] = computePathClearance3D(seedPath, scene, collisionStep);

% 若平滑后整条路径安全裕度不达标，则退回剪枝路径
if minSeedClr < minSafeClearance
    seedPath = fallbackPath;
end

end