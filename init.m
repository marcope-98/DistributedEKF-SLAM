%% Config file
config;

%% Includes
addpath(genpath('include'));
addpath(genpath('utils'));

%% load dataset and gather Metadata
[profile, Robots] = load_config(settings);
codeDict = containers.Map(profile.info.Barcodes(:,2), profile.info.Barcodes(:,1));
                          
%% INIT Robots array
params     = struct;
params.sim = profile.sim;
params.ekf = profile.ekf;
Agents = Agent.empty([0, profile.sim.nRobots]);
server = Server(profile.sim.nRobots);
for i = 1:profile.sim.nRobots
    params.id = i;
    Agents(i) = Agent(Robots{i}, codeDict, server, params);
end

clear settings i params codeDict Robot inbox AdjacencyMatrix
