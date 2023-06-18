%% Config file
config;

%% Includes
addpath(genpath('include'));
addpath(genpath('utils'));

%% load dataset and gather Metadata
[profile, Robots] = load_config(settings);
codeDict = containers.Map(profile.info.Barcodes(:,2), profile.info.Barcodes(:,1));
profile.sim.use_distributed = use_distributed;

%% INIT Robots array
params     = struct;
params.sim = profile.sim;
params.ekf = profile.ekf;

if use_distributed
    Agents = AgentDEKF.empty([0, profile.sim.nRobots]);
else
    Agents = AgentEKF.empty([0, profile.sim.nRobots]);
end

server = Server(profile.sim.nRobots);
for i = 1:profile.sim.nRobots
    params.id = i;
    if use_distributed
        Agents(i) = AgentDEKF(Robots{i}, codeDict, server, params);
    else
        Agents(i) = AgentEKF(Robots{i}, codeDict, server, params);
    end
end

clear settings i params codeDict Robot inbox use_distributed
