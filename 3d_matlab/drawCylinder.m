function drawCylinder(centerXY, radius, zmin, zmax, colorRGB)

n = 48;
[X, Y, Z] = cylinder(radius, n);
Z = Z * (zmax - zmin) + zmin;
X = X + centerXY(1);
Y = Y + centerXY(2);

surf(X, Y, Z, ...
    'FaceColor', colorRGB, ...
    'FaceAlpha', 0.94, ...
    'EdgeColor', 'none');

theta = linspace(0, 2*pi, n+1);
x = centerXY(1) + radius*cos(theta);
y = centerXY(2) + radius*sin(theta);

patch(x, y, zmin*ones(size(x)), colorRGB, ...
    'FaceAlpha', 0.94, 'EdgeColor', 'none');
patch(x, y, zmax*ones(size(x)), colorRGB, ...
    'FaceAlpha', 0.94, 'EdgeColor', 'none');
end