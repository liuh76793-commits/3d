function drawCuboid(rangeXYZ, colorRGB)
% rangeXYZ = [xmin xmax; ymin ymax; zmin zmax]

xmin = rangeXYZ(1,1); xmax = rangeXYZ(1,2);
ymin = rangeXYZ(2,1); ymax = rangeXYZ(2,2);
zmin = rangeXYZ(3,1); zmax = rangeXYZ(3,2);

V = [
    xmin ymin zmin
    xmax ymin zmin
    xmax ymax zmin
    xmin ymax zmin
    xmin ymin zmax
    xmax ymin zmax
    xmax ymax zmax
    xmin ymax zmax
    ];

F = [
    1 2 3 4
    5 6 7 8
    1 2 6 5
    2 3 7 6
    3 4 8 7
    4 1 5 8
    ];

patch('Vertices', V, 'Faces', F, ...
      'FaceColor', colorRGB, ...
      'FaceAlpha', 0.92, ...
      'EdgeColor', [0.18 0.18 0.18], ...
      'LineWidth', 0.8);
end