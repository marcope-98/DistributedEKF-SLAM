%% USER DEFINED VARIABLES
settings          = struct;
settings.nRobots  = 5;          % [1-5] number of robots to simulate
settings.Dataset  = 2;          % [1-9] dataset number
settings.dt       = 0.02;       % [s]   simulation time step

%% SIMULATION SETTINGS
settings.sim          = struct;
settings.sim.start    = NaN;    % default: NaN, index of simulation start
settings.sim.end      = NaN;    % default: NaN, index of simulation end

%% EXTENDED KALMAN FILTER SETTINGS
settings.ekf   = struct;
settings.ekf.Q = NaN;           % default: NaN, measurement noise matrix (2x2)
settings.ekf.A = NaN;           % default: NaN, control     noise vector (4x1)
settings.ekf.N = 15;            % number of landmarks, DO NOT CHANGE THIS UNLESS YOU HAVE A GOOD REASON TO DO SO
