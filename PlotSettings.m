function plt = PlotSettings
% This function setups the common basic settings for plotting figures.

% Marker styles
marker_styles  = {'o', 's', 'd', 'v', 'h', '.', '+', '^', '<', '>', '*', 'p', 'x'};
plt.marker_styles = marker_styles;

% Marker size
plt.marker_size = 15;

% Line styles
%line_styles = {'--', '-', ':', '-.', '--', '-.', '-', '-', '-', '-'};
line_styles = {'--' ,'-' , ':', '-.', '-', '--' , ':', '-.', '-', '--', ':', '-.'};
%line_styles = {'--' ,'-' , ':', '-.', '--', ':' , ':', '-.', '-', '--', ':', '-.'};
%line_styles = {'--', '-', '-', '-', '-', '-', '-', '-', '-', '-'};
plt.line_styles = line_styles;

% Line colors
line_colors = {...
    '#000000', ...
    '#D95319', ...
    '#EDB120', ...
    '#7E2F8E', ...
    '#A2142F', ...
    '#77AC30', ...
    '#706fd3', ...
    '#6a3d9a' , ...
    '#ffff99', ...
    };
plt.line_colors = line_colors;

% Line width
lw = 1.5;
plt.line_width = lw;

% Legend fontsize
legend_fs = 16;
plt.legend_fontsize = legend_fs;

% Label fontsize
label_fs = 16;
plt.label_fontsize = label_fs;

% Axis tick label fontsize
axis_fontsize = 16;
plt.axis_fontsize = axis_fontsize;

% Fontname
font_name = 'Times New Roman';
plt.font_name = font_name;

end

