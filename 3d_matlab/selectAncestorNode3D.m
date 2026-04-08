function [ancestorIdx, ancestorNode] = selectAncestorNode3D(tree, idxNear, backSteps)
% 从当前最近节点向上回撤若干层祖先
% 若父链不够长，则返回能到达的最早祖先

ancestorIdx = idxNear;

for k = 1:backSteps
    p = tree.parent(ancestorIdx);
    if p <= 0
        break;
    end
    ancestorIdx = p;
end

ancestorNode = tree.nodes(ancestorIdx, :);

end