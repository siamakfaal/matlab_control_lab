clc; clear; close all;

% Problem Setup
f = @(t, x)[x(1)+x(2)+x(1)^2+x(2)^2; x(1)-x(2)-x(1)^2+x(2)^2];

% Bounds, Resolution and Time Span
x1_bounds = [-2.5, 1]; x1_res = 0.2;
x2_bounds = [-2.5, 1]; x2_res = 0.2;
tspan = [0,3];

% Visualization Parameters
params = setupVisualizationParameters();

% Figure and Axes Setups
ax = setupAxes(params, x1_bounds, x2_bounds);

% Computations and Plotting
computeAndPlot(ax, f, tspan, x1_bounds, x2_bounds, x1_res, x2_res, params);


function params = setupVisualizationParameters()
% Define visualization parameters here for modularity
params.quiver_color_map = abyss;
params.trajectory_color = [0, 0.447, 0.741];
params.quiver_line_width = 2;
params.quiver_arrow_scale = 0.1;
params.quiver_max_headsize = 1;
params.trajectory_line_width = 1;
params.label_font_size = 25;
params.xticks = -2:1:1;
params.yticks = -2:1:1;
end

function ax = setupAxes(params, x1_bounds, x2_bounds)
ax = axes(figure('Colormap', params.quiver_color_map),...
    'NextPlot', 'add', 'Box', 'on', 'DataAspectRatio', [1, 1, 1],...
    'XTick', params.xticks, 'YTick', params.yticks,...
    'XLim', x1_bounds, 'YLim', x2_bounds,...
    'TickLabelInterpreter', 'latex', ...
    'FontSize', params.label_font_size);

xlabel(ax, '$x_1$', 'Interpreter', 'latex',...
    'FontSize', params.label_font_size);
ylabel(ax, '$x_2$', 'Interpreter', 'latex',...
    'FontSize', params.label_font_size);
end

function computeAndPlot(ax, f, tspan, x1_bounds, x2_bounds, x1_res, x2_res, params)
[x1, x2] = meshgrid(x1_bounds(1):x1_res:x1_bounds(2),...
    x2_bounds(1):x2_res:x2_bounds(2));

% Preallocate matrices
norm_dx = zeros(size(x1));
dx1 = zeros(size(x1));
dx2 = zeros(size(x1));

options = odeset('Events', @(t,x)outofbound(t, x, x1_bounds, x2_bounds));
cindex = linspace(0, 1, length(params.quiver_color_map))';

for i = 1:numel(x1)
    x0 = [x1(i); x2(i)];
    [~, x] = ode45(f, tspan, x0, options);
    plot(ax, x(:,1), x(:,2), 'Color', params.trajectory_color,...
        'LineWidth', params.trajectory_line_width);
    dx = f(0, x0);

    norm_dx(i) = norm(dx);
    dx1(i) = dx(1) / norm_dx(i);
    dx2(i) = dx(2) / norm_dx(i);
end

max_norm = max(norm_dx(:));

for i = 1:numel(x1)
    color = interp1(cindex, params.quiver_color_map, norm_dx(i) / max_norm);
    quiver(ax, x1(i), x2(i), dx1(i), dx2(i), params.quiver_arrow_scale,...
        'Color', color, 'LineWidth', params.quiver_line_width,...
        'MaxHeadSize', params.quiver_max_headsize);
end
end


function [position, isterminal, direction] = outofbound(~, x, x1_bounds, x2_bounds)
x1_limit = [-0.1, 0.1] + x1_bounds;
x2_limit = [-0.1, 0.1] + x2_bounds;
position = (x1_limit(1) < x(1) & x(1) < x1_limit(2) & ...
    x2_limit(1) < x(2) & x(2) < x2_limit(2));
isterminal = 1;
direction = 0;
end