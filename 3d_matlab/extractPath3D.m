function path = extractPath3D(tree, goalIndex)
% 从 goalIndex 沿 parent 回溯到根节点，再翻转成 start->goal

path = tree.nodes(goalIndex, :);
idx = goalIndex;

while tree.parent(idx) ~= 0
    idx = tree.parent(idx);
    path = [tree.nodes(idx, :); path]; %#ok<AGROW>
end
end