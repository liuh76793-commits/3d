function summarizeModeStats3D(stats)

fprintf('Free     : %d\n', stats.free);
fprintf('Narrow   : %d\n', stats.narrow);
fprintf('Blocked  : %d\n', stats.blocked);
fprintf('Approach : %d\n', stats.approach);
fprintf('Switches : %d\n', stats.switches);

end