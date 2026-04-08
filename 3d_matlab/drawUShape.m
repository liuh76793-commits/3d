function drawUShape(base, sz, t, opening, colorRGB)
% 真正的 U 型板绘图（非 3 个长方体裸拼）
%
% base = [x0,y0,z0]
% sz   = [Lx,Ly,Lz]
% t    = thickness
% opening = '+y' '-y' '+x' '-x'
%
% 原理：
% 1) 先在局部平面定义一个 U 形截面（二维多边形）
% 2) 再沿 z 方向拉伸成三维实体
%
% 注：
% 这里只负责“画得好看”
% 碰撞检测仍在 checkPointCollision3D 中按 U 型区域解析判断

x0 = base(1); y0 = base(2); z0 = base(3);
Lx = sz(1);   Ly = sz(2);   Lz = sz(3);

if t <= 0
    error('U 型板 thickness 必须 > 0');
end

if 2*t >= Lx || 2*t >= Ly
    error('U 型板 thickness 过大，导致内部空腔不存在');
end

% 在局部坐标系中构造 U 形二维边界
switch opening
    case '+y'
        % 开口朝 +y，上方敞开
        poly2d = [
            0, 0;
            Lx, 0;
            Lx, Ly;
            Lx-t, Ly;
            Lx-t, t;
            t, t;
            t, Ly;
            0, Ly
            ];

    case '-y'
        % 开口朝 -y，下方敞开
        poly2d = [
            0, 0;
            t, 0;
            t, Ly-t;
            Lx-t, Ly-t;
            Lx-t, 0;
            Lx, 0;
            Lx, Ly;
            0, Ly
            ];

    case '+x'
        % 开口朝 +x，右侧敞开
        poly2d = [
            0, 0;
            Lx, 0;
            Lx, t;
            t, t;
            t, Ly-t;
            Lx, Ly-t;
            Lx, Ly;
            0, Ly
            ];

    case '-x'
        % 开口朝 -x，左侧敞开
        poly2d = [
            0, t;
            Lx-t, t;
            Lx-t, Ly-t;
            0, Ly-t;
            0, Ly;
            Lx, Ly;
            Lx, 0;
            0, 0
            ];

    otherwise
        error('未知 U 型 opening 类型');
end

% 平移到全局坐标
poly2d(:,1) = poly2d(:,1) + x0;
poly2d(:,2) = poly2d(:,2) + y0;

n = size(poly2d, 1);

% 上下面顶点
V_bottom = [poly2d, z0 * ones(n,1)];
V_top    = [poly2d, (z0 + Lz) * ones(n,1)];
V = [V_bottom; V_top];

% 上下表面
patch('Vertices', V, ...
      'Faces', 1:n, ...
      'FaceColor', colorRGB, ...
      'FaceAlpha', 0.94, ...
      'EdgeColor', [0.18 0.18 0.18], ...
      'LineWidth', 0.8);

patch('Vertices', V, ...
      'Faces', n+(1:n), ...
      'FaceColor', colorRGB, ...
      'FaceAlpha', 0.94, ...
      'EdgeColor', [0.18 0.18 0.18], ...
      'LineWidth', 0.8);

% 侧壁
for i = 1:n
    i2 = i + 1;
    if i == n
        i2 = 1;
    end

    faceIdx = [i, i2, n+i2, n+i];
    patch('Vertices', V, ...
          'Faces', faceIdx, ...
          'FaceColor', colorRGB, ...
          'FaceAlpha', 0.94, ...
          'EdgeColor', [0.18 0.18 0.18], ...
          'LineWidth', 0.8);
end

end