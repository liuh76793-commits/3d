function c = estimateClearance3D(pt, scene)
% 返回点到最近障碍表面的近似距离
% 若点已在障碍内，则返回 0

if checkPointCollision3D(pt, scene)
    c = 0;
    return;
end

minDist = inf;
obsList = scene.obstacles;

for i = 1:length(obsList)
    obs = obsList(i);

    switch lower(obs.type)
        case 'sphere'
            d = norm(pt - obs.center) - obs.radius;

        case 'cylinder'
            dx = pt(1) - obs.center(1);
            dy = pt(2) - obs.center(2);
            radial = sqrt(dx^2 + dy^2) - obs.radius;

            if pt(3) < obs.zmin
                dz = obs.zmin - pt(3);
            elseif pt(3) > obs.zmax
                dz = pt(3) - obs.zmax;
            else
                dz = 0;
            end

            if radial > 0 && dz > 0
                d = sqrt(radial^2 + dz^2);
            else
                d = max(radial, dz);
            end

        case 'cuboid'
            r = obs.range;
            dx = max([r(1,1)-pt(1), 0, pt(1)-r(1,2)]);
            dy = max([r(2,1)-pt(2), 0, pt(2)-r(2,2)]);
            dz = max([r(3,1)-pt(3), 0, pt(3)-r(3,2)]);
            d = norm([dx,dy,dz]);

        case 'u_shape'
            % 近似按外包盒距离估计
            x0 = obs.base(1); y0 = obs.base(2); z0 = obs.base(3);
            Lx = obs.size(1); Ly = obs.size(2); Lz = obs.size(3);
            r = [x0 x0+Lx; y0 y0+Ly; z0 z0+Lz];
            dx = max([r(1,1)-pt(1), 0, pt(1)-r(1,2)]);
            dy = max([r(2,1)-pt(2), 0, pt(2)-r(2,2)]);
            dz = max([r(3,1)-pt(3), 0, pt(3)-r(3,2)]);
            d = norm([dx,dy,dz]);

        otherwise
            d = inf;
    end

    minDist = min(minDist, d);
end

c = max(minDist, 0);
end