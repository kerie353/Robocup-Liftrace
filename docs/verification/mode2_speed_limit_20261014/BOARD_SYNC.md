# Board source sync: mode 2 speed limit

2026-10-14. Host: 10.189.79.232. Workspace: /home/orangepi/liftrace_board_trials_20260928.

Only drop_descent_config.py, landing_posctl_config.py and DropDescentConfig::valid() changed: mode 2 accepts (0,3] m/s; all other modes retain the previous 1 m/s limit. The current board settings remain drop mode 2 at 1.0 m/s and H mode 2 at 1.0 m/s.

18 Python boundary checks and both Python AST checks passed; the C++ header condition and scoped git diff were checked. No compilation, ROS startup, aircraft or servo action was performed. The existing control binary must be rebuilt for the C++ limit to take effect. The parameter is a Z position trajectory slope, not a measured airspeed guarantee.

Backup: /home/orangepi/.codex_backups/mode2_speed_limit_20261014_004220. Existing board source and site changes were preserved. GitHub was not pushed.
