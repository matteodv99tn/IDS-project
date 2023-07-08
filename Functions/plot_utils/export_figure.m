function export_figure(filename, fig_name, fig_height)
    if nargin < 3
        fig_height = 3.5;
    end

    paths = split(filename, filesep);
    filename = split(paths{end}, ".");
    filename = filename{1};

    export_path = fullfile(...
        "Results", ...
        "Figures", ...
        paths{end-1}, ...
        filename);

    if ~exist(export_path, "dir")
        mkdir(export_path)
    end

    savefig(gcf, fullfile(export_path, strcat(fig_name, ".fig")));

    set(0, "DefaultAxesFontSize", 8);
    set(0, "DefaultFigureColor", "w");
    set(0, "defaulttextinterpreter", "tex");
    set(0, "DefaultAxesFontName", "times");
    set(gcf, "Units", "centimeters");
    set(gcf, "Position", [0 0 8.6 fig_height]);
    box on;


    exportgraphics(gcf, fullfile(export_path, strcat(fig_name, ".png")));
    exportgraphics(gcf, fullfile(export_path, strcat(fig_name, ".eps")));


end
