function win = updateWindowState3D(win, successFlag, deltaProg, clearanceVal, nodePos, qualityVal)
% 更新滑动窗口
%
% successFlag  : 1/0
% deltaProg    : 本次净推进量 Δd
% clearanceVal : 本次节点安全裕度 d_obs(q_k)
% nodePos      : 若成功则为 1x3 节点坐标，否则可传 []
% qualityVal   : 当前先记录，后续接 J(q_new)

if nargin < 6
    qualityVal = 0;
end

win.success   = [win.success, successFlag];
win.deltaProg = [win.deltaProg, deltaProg];
win.clearance = [win.clearance, clearanceVal];
win.quality   = [win.quality, qualityVal];

if successFlag == 1 && ~isempty(nodePos)
    win.successNodes = [win.successNodes; nodePos];
end

% 截断到窗口长度 W
if numel(win.success) > win.W
    excess = numel(win.success) - win.W;
    win.success(1:excess)   = [];
    win.deltaProg(1:excess) = [];
    win.clearance(1:excess) = [];
    win.quality(1:excess)   = [];
end

% successNodes 也同步保留最近 W 个成功节点
if size(win.successNodes,1) > win.W
    excessNode = size(win.successNodes,1) - win.W;
    win.successNodes(1:excessNode,:) = [];
end

end