function newPath = resamplePath3D(path, stepLen)
% 将路径按固定弧长间隔重采样
% stepLen: 重采样间距

if nargin < 2
    stepLen = 2.0;
end

if isempty(path) || size(path,1) < 2
    newPath = path;
    return;
end

newPath = path(1,:);

for i = 1:size(path,1)-1
    p1 = path(i,:);
    p2 = path(i+1,:);
    segLen = norm(p2 - p1);

    if segLen < 1e-10
        continue;
    end

    nSeg = max(floor(segLen / stepLen), 1);

    for k = 1:nSeg
        t = k / nSeg;
        pt = p1 + t * (p2 - p1);

        if k < nSeg || i == size(path,1)-1
            newPath = [newPath; pt]; %#ok<AGROW>
        end
    end
end

% 去掉可能重复的末点
if size(newPath,1) >= 2
    if norm(newPath(end,:) - path(end,:)) > 1e-10
        newPath = [newPath; path(end,:)];
    end
end

end