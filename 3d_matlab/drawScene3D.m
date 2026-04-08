function drawScene3D(scene, viewMode)

hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');

xlim(scene.boundary(1,:));
ylim(scene.boundary(2,:));
zlim(scene.boundary(3,:));

set(gca, 'FontSize', 11, 'LineWidth', 1.0);
box on;

% 绘制障碍物
for i = 1:length(scene.obstacles)
    obs = scene.obstacles(i);

    switch lower(obs.type)
        case 'cuboid'
            drawCuboid(obs.range, obs.color);

        case 'cylinder'
            drawCylinder(obs.center, obs.radius, obs.zmin, obs.zmax, obs.color);

        case 'sphere'
            drawSphereObs(obs.center, obs.radius, obs.color);

        case 'u_shape'
            drawUShape(obs.base, obs.size, obs.thickness, obs.opening, obs.color);

        otherwise
            error('未知障碍类型: %s', obs.type);
    end
end

% 起点终点
plot3(scene.start(1), scene.start(2), scene.start(3), 'go', ...
    'MarkerSize', 9, 'MarkerFaceColor', 'g', 'LineWidth', 1.2);
text(scene.start(1)+1.0, scene.start(2)+1.0, scene.start(3)+1.0, 'Start', ...
    'Color', [0 0.7 0], 'FontWeight', 'bold', 'FontSize', 11);

plot3(scene.goal(1), scene.goal(2), scene.goal(3), 'rp', ...
    'MarkerSize', 11, 'MarkerFaceColor', 'r', 'LineWidth', 1.2);
text(scene.goal(1)+1.0, scene.goal(2)+1.0, scene.goal(3)+1.0, 'Goal', ...
    'Color', 'r', 'FontWeight', 'bold', 'FontSize', 11);

camlight headlight;
camlight right;
lighting gouraud;

switch lower(viewMode)
    case 'perspective'
        view(38, 24);
        title('S1 三维主场景 - 透视图');
    case 'top'
        view(0, 90);
        title('S1 三维主场景 - 俯视图');
    case 'side'
        view(90, 0);
        title('S1 三维主场景 - 侧视图');
    otherwise
        view(3);
        title('S1 三维主场景');
end

end