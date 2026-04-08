function idx = findNearestNode3D(nodes, q)
% 在节点集合中寻找距离 q 最近的节点编号

diffs = nodes - q;
d2 = sum(diffs.^2, 2);
[~, idx] = min(d2);
end