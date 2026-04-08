function path = extractPathFromTree3D(tree, idx)
% 从树中某节点回溯到根节点，并翻转成 root -> idx

path = tree.nodes(idx, :);

while tree.parent(idx) ~= 0
    idx = tree.parent(idx);
    path = [tree.nodes(idx, :); path]; %#ok<AGROW>
end
end