function cmin = estimateSegmentClearance3D(p1, p2, scene, step)
% 快速版：只采样少量关键点估计线段最小安全裕度
% 目的：避免规划时因为 clearance 计算过密而显著变慢

if nargin < 4 || isempty(step)
    step = 0.5;
end

% 先做一次碰撞检测：若线段本身碰撞，直接返回 0
if checkSegmentCollision3D(p1, p2, scene, step)
    cmin = 0;
    return;
end

% 只取少量关键点，不再按很密的步长逐点扫描
% 这里取 5 个点：起点、1/4、1/2、3/4、终点
ts = [0, 0.25, 0.5, 0.75, 1.0];

cmin = inf;
for k = 1:numel(ts)
    t = ts(k);
    pt = p1 + t * (p2 - p1);
    c = estimateClearance3D(pt, scene);
    if c < cmin
        cmin = c;
    end
end

if ~isfinite(cmin)
    cmin = 0;
end
end