function drawSphereObs(centerXYZ, radius, colorRGB)

n = 36;
[X, Y, Z] = sphere(n);
X = radius * X + centerXYZ(1);
Y = radius * Y + centerXYZ(2);
Z = radius * Z + centerXYZ(3);

surf(X, Y, Z, ...
    'FaceColor', colorRGB, ...
    'FaceAlpha', 0.94, ...
    'EdgeColor', 'none');
end