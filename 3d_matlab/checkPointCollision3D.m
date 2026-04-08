function isCollide = checkPointCollision3D(pt, scene)

x = pt(1); y = pt(2); z = pt(3);

% 越界视为碰撞
if x < scene.boundary(1,1) || x > scene.boundary(1,2) || ...
   y < scene.boundary(2,1) || y > scene.boundary(2,2) || ...
   z < scene.boundary(3,1) || z > scene.boundary(3,2)
    isCollide = true;
    return;
end

isCollide = false;
obsList = scene.obstacles;

for i = 1:length(obsList)
    obs = obsList(i);

    switch lower(obs.type)

        case 'cuboid'
            r = obs.range;
            if x >= r(1,1) && x <= r(1,2) && ...
               y >= r(2,1) && y <= r(2,2) && ...
               z >= r(3,1) && z <= r(3,2)
                isCollide = true;
                return;
            end

        case 'cylinder'
            dx = x - obs.center(1);
            dy = y - obs.center(2);
            if (dx^2 + dy^2 <= obs.radius^2) && ...
               (z >= obs.zmin && z <= obs.zmax)
                isCollide = true;
                return;
            end

        case 'sphere'
            d2 = sum((pt - obs.center).^2);
            if d2 <= obs.radius^2
                isCollide = true;
                return;
            end

        case 'u_shape'
            if pointInUShapeSolid(pt, obs)
                isCollide = true;
                return;
            end

        otherwise
            error('未知障碍类型: %s', obs.type);
    end
end

end

%% ==========================================================
function flag = pointInUShapeSolid(pt, obs)
% U 型板实体碰撞检测
% 逻辑：
% 1) 先判断 z 是否落在高度范围内
% 2) 再判断 (x,y) 是否落在 U 型二维实体区域内

x = pt(1); y = pt(2); z = pt(3);

x0 = obs.base(1); y0 = obs.base(2); z0 = obs.base(3);
Lx = obs.size(1); Ly = obs.size(2); Lz = obs.size(3);
t  = obs.thickness;

if z < z0 || z > z0 + Lz
    flag = false;
    return;
end

% 先判断是否在外包矩形内
if x < x0 || x > x0 + Lx || y < y0 || y > y0 + Ly
    flag = false;
    return;
end

% 再去掉中间空腔区域
switch obs.opening
    case '+y'
        inInnerVoid = (x > x0+t) && (x < x0+Lx-t) && ...
                      (y > y0+t) && (y < y0+Ly);

    case '-y'
        inInnerVoid = (x > x0+t) && (x < x0+Lx-t) && ...
                      (y > y0)   && (y < y0+Ly-t);

    case '+x'
        inInnerVoid = (x > x0+t) && (x < x0+Lx) && ...
                      (y > y0+t) && (y < y0+Ly-t);

    case '-x'
        inInnerVoid = (x > x0)   && (x < x0+Lx-t) && ...
                      (y > y0+t) && (y < y0+Ly-t);

    otherwise
        error('未知 U 型 opening 类型');
end

flag = ~inInnerVoid;
end