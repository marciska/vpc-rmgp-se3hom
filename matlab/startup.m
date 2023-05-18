%% startup.m
% *Summary:* Loads all necessary libraries and other files for simulation
% -----------
%
% Editor:
%   OMAINSKA Marco - Doctoral Student, Cybernetics
%       <marcoomainska@g.ecc.u-tokyo.ac.jp>
% Property of: Fujita-Yamauchi Lab, The University of Tokyo, 2022
% Website: https://www.scl.ipc.i.u-tokyo.ac.jp

% ------------- BEGIN CODE -------------

% init core VPC-Library
run('vpc_library/startup.m')

% add project root folder
libDir = fileparts(mfilename('fullpath'));
addpath(libDir)

% create output directories if not yet existent
dirs = ["data", "images"];
for d = dirs
    [~, ~, ~] = mkdir(fullfile(libDir,d));
end

% add paths to sub-dependencies
fprintf('Loading other libraries...\n')
dirs = ["data", "lib", "simulink", "../ros"];
for d = dirs
    addpath(genpath(fullfile(libDir,d)))
end

% change all interpreters from tex to latex
fprintf('Change all interpreters to Latex...\n')
list_factory = fieldnames(get(groot,'factory'));
index_interpreter = find(contains(list_factory,'Interpreter'));
for i = 1:length(index_interpreter)
    default_name = strrep(list_factory{index_interpreter(i)},'factory','default');
    set(groot, default_name,'latex');
end

% clear variables
clear d dirs libDir list_factory index_interpreter default_name i

fprintf('[done]\n\n')

% -------------- END CODE --------------
