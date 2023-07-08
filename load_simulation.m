setup;
addpath("Functions/plot_utils");

testfile = "Results/test_1d/simulation_004.mat";
%plot_results(testfile, true, false);

% eteas

tmp = strcat("*", filesep, "*.mat");
filelist = dir(fullfile("Results", tmp));

for i = 1:length(filelist)
    file_data = filelist(i);
    file_name = fullfile(file_data.folder, file_data.name);
    plot_results(file_name, false, true);
end
