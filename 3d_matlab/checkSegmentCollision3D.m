function isCollide = checkSegmentCollision3D(p1, p2, scene, step)

if nargin < 4
    step = 1.0;
end

dist = norm(p2 - p1);
nSample = max(ceil(dist / step), 1);

isCollide = false;

for i = 0:nSample
    t = i / nSample;
    pt = p1 + t * (p2 - p1);

    if checkPointCollision3D(pt, scene)
        isCollide = true;
        return;
    end
end

end