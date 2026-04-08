function scene = createSceneS1()
% 改进后的三维主场景 S1（无重叠版）
% 设计目标：
% 1) 起点/终点难度更平衡，减轻双树天然失衡
% 2) 中部主通路为偏折式窄通道，稳定触发 Narrow
% 3) U 型困陷与 goal 收尾区分离，避免 Blocked / Approach 混在一起
% 4) goal 区为真正单入口口袋，便于激活 Approach
% 5) 所有障碍避免体积重叠，并尽量避免贴面粘连

scene.boundary = [0 100; 0 100; 0 100];

scene.start = [12, 14, 10];
scene.goal  = [90, 84, 36];

obs = [];

%% A. 起点侧：轻度绕行区
obs(end+1).type = 'cylinder';
obs(end).center = [20, 24];
obs(end).radius = 6;
obs(end).zmin   = 0;
obs(end).zmax   = 36;
obs(end).color  = [0.12, 0.55, 0.28];

obs(end+1).type = 'cylinder';
obs(end).center = [32, 38];
obs(end).radius = 5;
obs(end).zmin   = 0;
obs(end).zmax   = 44;
obs(end).color  = [0.14, 0.60, 0.30];

obs(end+1).type = 'cylinder';
obs(end).center = [44, 20];
obs(end).radius = 6;
obs(end).zmin   = 0;
obs(end).zmax   = 32;
obs(end).color  = [0.10, 0.50, 0.25];

% 起点侧短墙：让 start 树不是一开始就完全自由
obs(end+1).type = 'cuboid';
obs(end).range  = [18 24; 46 60; 0 18];
obs(end).color  = [0.78, 0.74, 0.20];

%% B. 中部夹层板：高度约束
% 下层板
obs(end+1).type = 'cuboid';
obs(end).range  = [40 58; 30 56; 20 24];
obs(end).color  = [0.82, 0.78, 0.22];

% 上层板
obs(end+1).type = 'cuboid';
obs(end).range  = [40 58; 30 56; 46 50];
obs(end).color  = [0.82, 0.78, 0.22];

% 中部短横板：打断简单直穿，但不与其他障碍重叠
obs(end+1).type = 'cuboid';
obs(end).range  = [46 62; 36 60; 32 36];
obs(end).color  = [0.82, 0.78, 0.22];

%% C. 中部偏折窄通道（dog-leg）：主触发 Narrow
% 左竖板
obs(end+1).type = 'cuboid';
obs(end).range  = [64 68; 22 54; 0 76];
obs(end).color  = [0.68, 0.78, 0.24];

% 右竖板（错位，形成偏折）
obs(end+1).type = 'cuboid';
obs(end).range  = [76 80; 44 76; 0 76];
obs(end).color  = [0.68, 0.78, 0.24];

% 偏折处短挡板：注意与 B 区不重叠
obs(end+1).type = 'cuboid';
obs(end).range  = [68 74; 56 60; 0 24];
obs(end).color  = [0.68, 0.78, 0.24];

%% D. 中右部 U 型困陷：专门触发 Blocked（与 goal 区分离）
obs(end+1).type = 'u_shape';
obs(end).base      = [62, 65, 15];
obs(end).size      = [12, 14, 24];
obs(end).thickness = 3;
obs(end).opening   = '-y';
obs(end).color     = [0.92, 0.70, 0.18];

%% E. 球体：调节安全裕度与曲面绕行（避开 U 与 goal 门口）
obs(end+1).type = 'sphere';
obs(end).center = [42, 78, 36];
obs(end).radius = 6;
obs(end).color  = [0.95, 0.76, 0.16];

obs(end+1).type = 'sphere';
obs(end).center = [38, 58, 14];
obs(end).radius = 5;
obs(end).color  = [0.95, 0.76, 0.16];

%% F. goal 邻域：真正的西侧单入口口袋（主触发 Approach）
% 上横墙
obs(end+1).type = 'cuboid';
obs(end).range  = [82 94; 90 94; 10 70];
obs(end).color  = [0.86, 0.80, 0.28];

% 下横墙
obs(end+1).type = 'cuboid';
obs(end).range  = [82 94; 74 78; 10 70];
obs(end).color  = [0.86, 0.80, 0.28];

% 右竖墙
obs(end+1).type = 'cuboid';
obs(end).range  = [82 86; 78 90; 10 70];
obs(end).color  = [0.86, 0.80, 0.28];

%%% 入口导向短墙：和右侧竖板留出明确间距，避免重叠/贴面
%%obs(end+1).type = 'cuboid';
%%obs(end).range  = [70 74; 78 90; 0 50];
%%obs(end).color  = [0.86, 0.80, 0.28];

scene.obstacles = obs;
end