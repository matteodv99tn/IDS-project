close all;
clear;
clc;

setup;
addpath("SimulationsConfigs");

n_experiments = 2;

scripts_names = {...
    "test_1a", ...
    "test_1b", ...
    "test_1c", ...
    "test_1d", ...
    "test_2a", ...
    "test_2b", ...
    "test_2c", ...
    "test_2d", ...
    "test_3a", ...
    "test_3b", ...
    "test_3c", ...
    "test_3d", ...
    };
% parpool(4);

for j = 1:n_experiments
    correct_results_folder;
    parfor i = 1:length(scripts_names)
        run_test(scripts_names, i);
    end
end


delete(gcp);



function run_test(sources, idx)
    try
        pause(rand()*3);
        run(sources{idx});
    catch
        f = fopen("parpool.log", "a");
        fprintf(f, "Error while computing %s\n", sources{idx});
    end
end
