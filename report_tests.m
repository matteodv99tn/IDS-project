close all;
clear;
clc;

setup;
addpath("SimulationsConfigs");

n_experiments = 3;
perform_star_test = true;
perform_hexa_test = false;

parpool(2);


%  ____  _
% / ___|| |_ __ _ _ __
% \___ \| __/ _` | '__|
%  ___) | || (_| | |
% |____/ \__\__,_|_|
%
if perform_star_test

    scripts_names = {...
        % "test_star_1a", ...
        % "test_star_1c", ...
        "test_star_2a", ...
        "test_star_2c", ...
        % "test_star_3a", ...
        % "test_star_3c", ...
        };

    for j = 1:n_experiments
        correct_results_folder;
        parfor i = 1:length(scripts_names)
            run_test(scripts_names, i);
        end
    end
end



%  _   _
% | | | | _____  ____ _
% | |_| |/ _ \ \/ / _` |
% |  _  |  __/>  < (_| |
% |_| |_|\___/_/\_\__,_|
%
if perform_hexa_test

    scripts_names = {...
        % "test_hexa_1a", ...
        % "test_hexa_1c", ...
        "test_hexa_2a", ...
        "test_hexa_2c", ...
        % "test_hexa_3a", ...
        % "test_hexa_3c", ...
        };

    for j = 1:n_experiments
        correct_results_folder;
        parfor i = 1:length(scripts_names)
            run_test(scripts_names, i);
        end
    end
end

delete(gcp);


%  _____                 _   _
% |  ___|   _ _ __   ___| |_(_) ___  _ __  ___
% | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
% |  _|| |_| | | | | (__| |_| | (_) | | | \__ \
% |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
%

function run_test(sources, idx)
    try
        pause(rand()*3);
        run(sources{idx});
    catch
        f = fopen("parpool.log", "a");
        fprintf(f, "Error while computing %s\n", sources{idx});
    end
end
