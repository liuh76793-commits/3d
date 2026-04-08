function cmin = estimateSegmentClearance3D(p1, p2, scene, step)
% 估计线段 p1->p2 上的最小安全裕度
% 若线段碰撞，直接返回 0

if nargin < 4 || isempty(step)
    step = 0.5;
end

if checkSegmentCollision3D(p1, p2, scene, step)
    cmin = 0;
    return;
end

dist = norm(p2 - p1);
nSample = max(ceil(dist / step), 2);

cmin = inf;
for i = 0:nSample
    t = i / nSample;
    pt = p1 + t * (p2 - p1);
    c = estimateClearance3D(pt, scene);
    cmin = min(cmin, c);
end

if isinf(cmin)
    cmin = 0;
end
end