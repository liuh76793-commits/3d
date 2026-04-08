function prunedPath = shortcutPrunePath3D(path, scene, collisionStep, params)
% 安全感知 shortcut/prune
%
% 功能：
% 1) 尝试把当前点与后续更远点直接连接
% 2) 不仅要求无碰撞
% 3) 还要求新 shortcut 的最小安全裕度不能比被替代原局部路径差太多
%
% 输入：
%   path          : 原始路径，N x 3
%   scene         : 场景
%   collisionStep : 线段碰撞检测与安全采样步长
%   params        : 参数结构体，需包含：
%                   params.pruneMinSafeClearance
%                   params.pruneKeepClearRatio
%
% 输出：
%   prunedPath    : 安全感知剪枝后的路径

if nargin < 3 || isempty(collisionStep)
    collisionStep = 1.0;
end

% 允许旧调用方式，给默认值
if nargin < 4 || isempty(params)
    params.pruneMinSafeClearance = 0.30;
    params.pruneKeepClearRatio   = 0.80;
end

if ~isfield(params, 'pruneMinSafeClearance')
    params.pruneMinSafeClearance = 0.30;
end
if ~isfield(params, 'pruneKeepClearRatio')
    params.pruneKeepClearRatio = 0.80;
end

if isempty(path) || size(path,1) <= 2
    prunedPath = path;
    return;
end

prunedPath = path;
i = 1;

while i < size(prunedPath,1)-1
    shortcutFound = false;

    % 从远到近尝试，优先找能跨越最多中间节点的 shortcut
    for j = size(prunedPath,1):-1:(i+2)
        p1 = prunedPath(i,:);
        p2 = prunedPath(j,:);

        % ---------- 1. 先判无碰撞 ----------
        if checkSegmentCollision3D(p1, p2, scene, collisionStep)
            continue;
        end

        % ---------- 2. 计算候选 shortcut 的安全裕度 ----------
        shortcutSeg = [p1; p2];
        [newMinClr, ~] = computePathClearance3D(shortcutSeg, scene, collisionStep);

        % ---------- 3. 计算被替代的原局部路径安全裕度 ----------
        oldLocalPath = prunedPath(i:j, :);
        [oldMinClr, ~] = computePathClearance3D(oldLocalPath, scene, collisionStep);

        % ---------- 4. 安全感知接受条件 ----------
        % 新 shortcut 必须同时满足：
        % (a) 绝对安全门槛
        % (b) 相对原局部路径不能掉太多
        safeThresh = max(params.pruneMinSafeClearance, ...
                         params.pruneKeepClearRatio * oldMinClr);

        if newMinClr < safeThresh
            continue;
        end

        % ---------- 5. 接受 shortcut ----------
        prunedPath = [prunedPath(1:i,:); prunedPath(j:end,:)];
        shortcutFound = true;
        break;
    end

    if ~shortcutFound
        i = i + 1;
    end
end

end