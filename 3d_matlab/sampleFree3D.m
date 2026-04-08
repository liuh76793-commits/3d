function q_rand = sampleFree3D(scene, goalSampleRate, goalPoint)
% 在边界内随机采样
% 以一定概率直接采样 goal，提高收敛效率

if rand < goalSampleRate
    q_rand = goalPoint;
    return;
end

xmin = scene.boundary(1,1); xmax = scene.boundary(1,2);
ymin = scene.boundary(2,1); ymax = scene.boundary(2,2);
zmin = scene.boundary(3,1); zmax = scene.boundary(3,2);

q_rand = [ ...
    xmin + rand * (xmax - xmin), ...
    ymin + rand * (ymax - ymin), ...
    zmin + rand * (zmax - zmin) ...
    ];
end