clear all
nPoints = 3;
dim = 5;
P = rand(dim, nPoints);
data_legend = {'data1', 'data2', 'data3'};
dim_legend = {'X1', 'X2', 'X3', 'X4', 'X5'};
params.axis_reverse = [1 0 0 1 0];
params.data_symbols = {'o', 'x', 'o'};
fig1 = radar_plot(P, dim_legend, data_legend, params);

params.circular_isocurves = true;
params.dim_legend_font_size = 10;
fig2 = radar_plot(P, dim_legend, data_legend, params);

params.isocurve_line_style = '--';
params.show_dim_scale_label = false;
params.rotate_dim_legend = false;
fig3 = radar_plot(P, dim_legend, data_legend, params);