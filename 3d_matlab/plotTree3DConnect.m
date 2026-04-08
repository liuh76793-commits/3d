function plotTree3DConnect(tree, colorRGB)

nodes  = tree.nodes;
parent = tree.parent;

for i = 2:size(nodes,1)
    pIdx = parent(i);
    if pIdx <= 0
        continue;
    end

    p1 = nodes(i,:);
    p2 = nodes(pIdx,:);

    plot3([p1(1), p2(1)], ...
          [p1(2), p2(2)], ...
          [p1(3), p2(3)], ...
          '-', 'Color', colorRGB, 'LineWidth', 0.7);
end
end