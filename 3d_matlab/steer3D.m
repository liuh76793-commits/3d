function q_new = steer3D(q_near, q_rand, stepLen)
% 从 q_near 朝 q_rand 方向扩展 stepLen
% 若距离不足 stepLen，则直接到 q_rand

dirVec = q_rand - q_near;
dist = norm(dirVec);

if dist < 1e-10
    q_new = q_near;
    return;
end

if dist <= stepLen
    q_new = q_rand;
else
    q_new = q_near + (dirVec / dist) * stepLen;
end
end