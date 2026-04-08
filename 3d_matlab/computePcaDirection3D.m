function dirVec = computePcaDirection3D(successNodes)
% 基于成功节点计算 PCA 主方向
% 若样本不足，则返回空

dirVec = [];

N = size(successNodes, 1);
if N < 3
    return;
end

X = successNodes - mean(successNodes, 1);
C = (X' * X) / max(N - 1, 1);

[V, D] = eig(C);
eigVals = diag(D);
[~, idx] = sort(real(eigVals), 'descend');

mainVec = V(:, idx(1));
mainVec = real(mainVec(:)');

if norm(mainVec) < 1e-10
    return;
end

dirVec = mainVec / norm(mainVec);

end