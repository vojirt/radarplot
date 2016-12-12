function fig = radar_plot(data, dim_legends, data_legends, params)
% radar_plot  Plot values in radar-like plot type.
%   fig = radar_plot(data, dim_legends, data_legends)
%              - plot radar using the data and label axis with dim_labels and data legend with data_legends.
%              - each row of the data is one dimmension (axis of radar), columns are datapoints
%   fig = radar_plot(data, dim_legends, data_legends, params)
%              - params is a structure containing additional parameters for radar plot
%                (available params and their default values are described at beginnig of the code)
%
%   output: handle to the plot figure
%
%   Example:
%        nPoints = 4;
%        dim = 5;
%        P = rand(dim, nPoints);
%        data_legend = {'data1', 'data2', 'data3', 'data4'};
%        dim_legend = {'X1', 'X2', 'X3', 'X4', 'X5'};
%        params.axis_reverse = [1 0 0 1 0];
%        params.data_symbols = {'o', 'x', 'o', 'x'};
%        fig = radar_plot(P, dim_legend, data_legend, params);
%
%
% Copyright (c) 2016 Tomas Vojir
%
% Inspired by David Said, Copyright (c) 2011
% All rights reserved.
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are
% met:
%     * Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
%     * Redistributions in binary form must reproduce the above copyright
%       notice, this list of conditions and the following disclaimer in
%       the documentation and/or other materials provided with the distribution
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.


    [dim, nPoints] = size(data);

    % set parameters default values and look if there are set externally
    axis_lims = [];             % axis limits dim*2 [min, max];
    axis_reverse = [];          % should the axis be reversed (i.e. axis max at the radar center) dim*1 [bool]
    axis_color = [1, 1, 1]*0.8; % color of the axis and isocurves
    outer_isocurve_color = [1, 1, 1]*0.65;   % color of outer isocurve (plot boundary)
    num_isocurves = 5;          % number of isocurves
    circular_isocurves = false; % circular isocurves instead of straight lines? [bool]
    isocurve_line_style = '-';  % style of the isocurves lines, e.g. '-', '--', ':'
    datalines_transparancy_value = 0.4; % transparancy level of the datapoints line colors (0 ... transparent, 1 ... full color)
    marker_size = 6;            % size of the datapoints markers
    data_colors = hsv(nPoints); % colors of the datapoints dim*3
    data_symbols = repmat({'o', 'x', 's', '^', 'p', '>'}, [1, ceil(nPoints/6)]); % datapoints markers types dim*1
    figure_visible = 'On';      % show figure {'On', 'Off'}
    show_legend = true;         % show datapoints legend [bool]
    rotate_dim_legend = true;   % rotate axis labels to match the axis rotation [bool]
    dim_legend_font = 'AvantGarde'; % font of the axis labels
    dim_legend_font_size = 14;      % font size of the axis labels
    dim_legend_font_bold = false;   % font bold of the axis labels
    show_dim_scale_label = true;    % show axis scale (min, max) in the axis lablel

    if isfield(params, 'axis_lims'), axis_lims = params.axis_lims; end;
    if isfield(params, 'axis_reverse'), axis_reverse = params.axis_reverse; end;
    if isfield(params, 'axis_color'), axis_color = params.axis_color; end;
    if isfield(params, 'outer_isocurve_color'), outer_isocurve_color = params.outer_isocurve_color; end;
    if isfield(params, 'num_isocurves'), num_isocurves = params.num_isocurves; end;
    if isfield(params, 'circular_isocurves'), circular_isocurves = params.circular_isocurves; end;
    if isfield(params, 'isocurve_line_style'), isocurve_line_style = params.isocurve_line_style; end;
    if isfield(params, 'datalines_transparancy_value'), datalines_transparancy_value = params.datalines_transparancy_value; end;
    if isfield(params, 'marker_size'), marker_size = params.marker_size; end;
    if isfield(params, 'data_colors'), data_colors = params.data_colors; end;
    if isfield(params, 'data_symbols'), data_symbols = params.data_symbols; end;
    if isfield(params, 'figure_visible'), figure_visible = params.figure_visible; end;
    if isfield(params, 'show_legend'), show_legend = params.show_legend; end;
    if isfield(params, 'rotate_dim_legend'), rotate_dim_legend = params.rotate_dim_legend; end;
    if isfield(params, 'dim_legend_font'), dim_legend_font = params.dim_legend_font; end;
    if isfield(params, 'dim_legend_font_size'), dim_legend_font_size = params.dim_legend_font_size; end;
    if isfield(params, 'dim_legend_font_bold'), dim_legend_font_bold = params.dim_legend_font_bold; end;
    if isfield(params, 'show_dim_scale_label'), show_dim_scale_label = params.show_dim_scale_label; end;

%%% Plot the axes
    fig = figure('Visible', figure_visible);
    hold on;

    % Radial offset per axis, first axis is vertical
    th_axis = mod(pi/2 + (2*pi/dim)*(ones(2,1)*(dim:-1:1)), 2*pi);
    % Axis start and end
    r = [0; 0.92]*ones(1,dim);

    % Conversion to cartesian coordinates to plot using regular plot.
    [x,y] = pol2cart(th_axis, r);
    hLine = line(x, y, 'LineWidth', 1, 'Color', axis_color);
    for i = 1:numel(hLine)
        % Exclude line from legend
        set(get(get(hLine(i),'Annotation'),'LegendInformation'), 'IconDisplayStyle','off');
    end

%%% Plot axes isocurves
    if circular_isocurves
        th_iso = 0:2*pi/200:2*pi;
        r = linspace(0.1, 0.9, num_isocurves)';
        x = r * cos(th_iso);
        y = r * sin(th_iso);
        for i = 1:num_isocurves
            line_color = axis_color;
            if i == num_isocurves, line_color = outer_isocurve_color; end;
            hLine = plot(x(i,:), y(i,:), 'LineWidth', 1, 'Color', line_color, 'LineStyle', isocurve_line_style);
            set(get(get(hLine,'Annotation'),'LegendInformation'), 'IconDisplayStyle','off');
        end
    else
        % Radial offset per axis
        th_iso = pi/2 + (2*pi/dim)*(ones(num_isocurves,1)*(dim:-1:1));
        % Axis start and end
        r = (linspace(0.1, 0.9, num_isocurves)')*ones(1,dim);
        % Conversion to cartesian coordinates to plot using regular plot.
        [x,y] = pol2cart(th_iso, r);
        hLine = line([x(1:end-1, :), x(1:end-1, 1)]', [y(1:end-1, :), y(1:end-1, 1)]', 'LineWidth', 1, 'Color', axis_color, 'LineStyle', isocurve_line_style);
        for i = 1:numel(hLine)
            % Exclude line from legend
            set(get(get(hLine(i),'Annotation'),'LegendInformation'), 'IconDisplayStyle','off');
        end
        hLine = line([x(end, :), x(end, 1)]', [y(end, :), y(end, 1)]', 'LineWidth', 1, 'Color', outer_isocurve_color, 'LineStyle', isocurve_line_style);
        % Exclude line from legend
        set(get(get(hLine,'Annotation'),'LegendInformation'), 'IconDisplayStyle','off');
    end

%%%P Plot axis labels
    % Compute minimum and maximum per axis
    if isempty(axis_lims)
        minV = min(data,[],2);
        maxV = max(data,[],2);
    else
        minV = axis_lims(:, 1);
        maxV = axis_lims(:, 2);
    end

    for j = 1:dim
        % Generate the axis label
        msg_1 = sprintf('%s', dim_legends{j});
        if ~isempty(axis_reverse) && axis_reverse(j) > 0
            msg_2 = sprintf('(%.2f, %.2f)', maxV(j), minV(j));
        else
            msg_2 = sprintf('(%.2f, %.2f)', minV(j), maxV(j));
        end
        [mx, my] = pol2cart( th_axis(1, j), 1.02 + 0.05*(dim_legend_font_size/10 - 1));     % distance of axis labels + take into account font size

        if show_dim_scale_label
            txt = {msg_1; msg_2};
        else
            txt = msg_1;
        end

        a = th_axis(1, j);
        rot_angle = 0;

        if a == pi/2 || a == 3*pi/2
            alignment = 'center';
        elseif a >= 0 && a < pi/2
            alignment = 'left';
            rot_angle = a - pi/2;
        elseif a > pi/2 && a <= pi
            alignment = 'right';
            rot_angle = a - pi/2;
        elseif a > pi && a < 3*pi/2
            alignment = 'right';
            rot_angle = a - 3*pi/2;
        else
            alignment = 'left';
            rot_angle = a - 3*pi/2;
        end
        t = text(mx, my, txt, 'HorizontalAlignment', alignment,'FontSize', dim_legend_font_size, 'FontName', dim_legend_font);
        if rotate_dim_legend, set(t, 'Rotation', 180*rot_angle/pi); set(t, 'HorizontalAlignment', 'center'); end;
        if dim_legend_font_bold, set(t, 'FontWeight', 'bold'); end;
    end
    axis([-1,1,-1,1]*1.5)
    hold off;

%%% Plot data
    hold on
    % Radius
    if isempty(axis_reverse)
        R = 0.8*((data - (minV*ones(1,nPoints)))./((maxV-minV)*ones(1,nPoints))) + 0.1;
    else
        R = zeros(dim, nPoints);
        for j = 1:dim
            % Invert axis min/max
            if axis_reverse(j)
                R(j, :) = 0.8*(1 - (data(j,:) - (minV(j)*ones(1,nPoints)))./((maxV(j)-minV(j))*ones(1,nPoints))) + 0.1;
            else
                R(j, :) = 0.8*((data(j,:) - (minV(j)*ones(1,nPoints)))./((maxV(j)-minV(j))*ones(1,nPoints))) + 0.1;
            end
        end
    end
    R = [R; R(1,:)];
    Th = pi/2 + (2*pi/dim) * ((dim:-1:0)'*ones(1,nPoints));

    % polar(Th, R)
    [X, Y] = pol2cart(Th, R);
    order = size(X, 2):-1:1;
    % Handles storage
    h = zeros(size(X, 2), 1);
    for i = order
        h(i) = plot(X(:, i), Y(:, i), '-', 'Color', [data_colors(i, :) datalines_transparancy_value], ...
               'LineWidth', 1.5, 'Marker', data_symbols{i}, 'MarkerSize', marker_size, 'MarkerFaceColor', data_colors(i, :));
    end
    axis([-1,1,-1,1]);
    axis square;
    axis off;
    if show_legend, legend(h, data_legends{:}, 'Location', 'best'); end;
    hold off;

end
