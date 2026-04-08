function win = initWindowState3D(W)
% 初始化滑动窗口状态

win.W = W;

% 最近 W 次扩展记录
win.success   = [];
win.deltaProg = [];
win.clearance = [];
win.quality   = [];

% 成功节点，用于 PCA / 密度
win.successNodes = [];

% 上一次可信的局部线性度
win.lastLscore = 1.0;

% 状态机相关
win.state      = "free";
win.lastState  = "free";
win.dwellCount = 1;
win.freezeCount = 0;

% 连续连接失败次数
win.failConn = 0;

% 最近一次统计结果缓存
win.metrics = struct( ...
    'R_succ', 0, ...
    'E_prog', 0, ...
    'dbar', inf, ...
    'dmin', inf, ...
    'L_score', 1.0, ...
    'rho_local', 0, ...
    'N_succ', 0);

end