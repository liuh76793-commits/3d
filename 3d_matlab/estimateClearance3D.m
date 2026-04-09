function d = estimateClearance3D(pt, scene)
% estimateClearance3D
% 估计点 pt 到最近障碍物表面的距离
% pt: [1x3]
% 返回 d >= 0

d = inf;
obsList = scene.obstacles;

for i = 1:numel(obsList)
    obs = obsList(i);

    switch lower(obs.type)
        case 'sphere'
            c = obs.center;
            r = obs.radius;
            di = abs(norm(pt - c) - r);

        case 'cylinder'
            % 默认圆柱轴向为 z
            c = obs.center;
            r = obs.radius;

            if isfield(obs, 'height')
                zmin = c(3) - obs.height / 2;
                zmax = c(3) + obs.height / 2;
            elseif isfield(obs, 'zmin') && isfield(obs, 'zmax')
                zmin = obs.zmin;
                zmax = obs.zmax;
            else
                zmin = c(3) - 0.5;
                zmax = c(3) + 0.5;
            end

            dx = pt(1) - c(1);
            dy = pt(2) - c(2);
            dz = pt(3);

            radial = hypot(dx, dy);
            dr = max(radial - r, 0);
            dz_out = max([zmin - dz, 0, dz - zmax]);

            if radial <= r && dz >= zmin && dz <= zmax
                di = min([r - radial, dz - zmin, zmax - dz]);
            else
                di = norm([dr, dz_out]);
            end

        case 'cuboid'
            r = obs.range;  % 3x2: [xmin xmax; ymin ymax; zmin zmax]
            di = pointToCuboidDistance(pt, r);

        case 'u_shape'
            parts = decomposeUShapeToCuboids(obs);
            di = inf;
            for k = 1:numel(parts)
                rk = parts(k).range;
                di = min(di, pointToCuboidDistance(pt, rk));
            end

        otherwise
            continue;
    end

    d = min(d, di);
end

if ~isfinite(d)
    d = 1e3;
end

end

% ==================== 辅助函数 ====================

function d = pointToCuboidDistance(pt, r)
% r: [xmin xmax; ymin ymax; zmin zmax]

dx = max([r(1,1) - pt(1), 0, pt(1) - r(1,2)]);
dy = max([r(2,1) - pt(2), 0, pt(2) - r(2,2)]);
dz = max([r(3,1) - pt(3), 0, pt(3) - r(3,2)]);

outsideDist = norm([dx, dy, dz]);

insideX = (pt(1) >= r(1,1) && pt(1) <= r(1,2));
insideY = (pt(2) >= r(2,1) && pt(2) <= r(2,2));
insideZ = (pt(3) >= r(3,1) && pt(3) <= r(3,2));

if insideX && insideY && insideZ
    d = min([ ...
        pt(1) - r(1,1), r(1,2) - pt(1), ...
        pt(2) - r(2,1), r(2,2) - pt(2), ...
        pt(3) - r(3,1), r(3,2) - pt(3) ]);
else
    d = outsideDist;
end
end

function parts = decomposeUShapeToCuboids(obs)
% 将 U 型障碍拆成 3 个长方体实体，用于 clearance 计算
% 与 checkPointCollision3D 中 pointInUShapeSolid 逻辑保持一致

base = obs.base;
sz = obs.size;
t = obs.thickness;
opening = lower(obs.opening);

x0 = base(1); y0 = base(2); z0 = base(3);
Lx = sz(1);   Ly = sz(2);   Lz = sz(3);

parts = struct('range', {});

switch opening
    case '-y'
        % 左壁
        parts(1).range = [x0, x0+t; ...
                          y0, y0+Ly; ...
                          z0, z0+Lz];
        % 右壁
        parts(2).range = [x0+Lx-t, x0+Lx; ...
                          y0, y0+Ly; ...
                          z0, z0+Lz];
        % 后壁
        parts(3).range = [x0+t, x0+Lx-t; ...
                          y0+Ly-t, y0+Ly; ...
                          z0, z0+Lz];

    case '+y'
        % 左壁
        parts(1).range = [x0, x0+t; ...
                          y0, y0+Ly; ...
                          z0, z0+Lz];
        % 右壁
        parts(2).range = [x0+Lx-t, x0+Lx; ...
                          y0, y0+Ly; ...
                          z0, z0+Lz];
        % 前壁
        parts(3).range = [x0+t, x0+Lx-t; ...
                          y0, y0+t; ...
                          z0, z0+Lz];

    case '+x'
        % 下壁
        parts(1).range = [x0, x0+Lx; ...
                          y0, y0+t; ...
                          z0, z0+Lz];
        % 上壁
        parts(2).range = [x0, x0+Lx; ...
                          y0+Ly-t, y0+Ly; ...
                          z0, z0+Lz];
        % 左壁
        parts(3).range = [x0, x0+t; ...
                          y0+t, y0+Ly-t; ...
                          z0, z0+Lz];

    case '-x'
        % 下壁
        parts(1).range = [x0, x0+Lx; ...
                          y0, y0+t; ...
                          z0, z0+Lz];
        % 上壁
        parts(2).range = [x0, x0+Lx; ...
                          y0+Ly-t, y0+Ly; ...
                          z0, z0+Lz];
        % 右壁
        parts(3).range = [x0+Lx-t, x0+Lx; ...
                          y0+t, y0+Ly-t; ...
                          z0, z0+Lz];

    otherwise
        parts(1).range = [x0, x0+Lx; ...
                          y0, y0+Ly; ...
                          z0, z0+Lz];
end
end
