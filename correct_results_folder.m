tmp = strcat("*", filesep, "*.log");
filelist = dir(fullfile("Results", tmp));

for i = 1:length(filelist)
    file_data = filelist(i);
    log_file_name = fullfile(file_data.folder, file_data.name);
    mat_file_name = strrep(log_file_name, ".log", ".mat");

    if ~exist(mat_file_name, 'file')
        delete(log_file_name);
    end
end
