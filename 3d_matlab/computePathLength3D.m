function L = computePathLength3D(path)

L = 0;
for i = 1:size(path,1)-1
    L = L + norm(path(i+1,:) - path(i,:));
end

end