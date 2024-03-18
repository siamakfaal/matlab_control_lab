clc; clear; 
%close all;

f = @(t, x)[x(2); sin(x(1))];

xticks = [-2*pi, -pi, 0, pi, 2*pi];
xticklabels = {'$-2\pi$', '$\pi$','$0$','$\pi$', '$2\pi$'};
yticks = [-2, 0, 2];

phaseplane = portrait([-8, 0.5, 8 ; -3, 0.2, 3], ...
    xticks=xticks, yticks=yticks, ...
    trajectory_color=[0.8, 0.8, 0.8]);

x0 = phaseplane.icgrid('lhr');


ax = phaseplane.buildaxes(figure(Position=[118, 314, 650, 250]));

phaseplane.trajectories(f,x0,[0,20],ax);

plot(ax, xticks, zeros(size(xticks)), 'o',...
    Color=lines(1), MarkerFaceColor=lines(1), MarkerSize=5);

set(ax,XTickLabel=xticklabels)


return

%% Problem Setup
% f = @(t, x)[x(1)+x(2)+x(1)^2+x(2)^2; x(1)-x(2)-x(1)^2+x(2)^2];
% 
% x1_bounds = [-2.5, 1]; x1_res = 0.2;
% x2_bounds = [-2.5, 1]; x2_res = 0.2;

f = @(t, x)[x(2); sin(x(1))];

x1_bounds = [-8, 8]; x1_res = 1;
x2_bounds = [-3, 3]; x2_res = 1;


vertical_point = (0:0.2:3)';
horizontal_points = (0:0.5:x1_bounds(2))';


x0 = [x1_bounds(1)*ones(size(vertical_point)), vertical_point;
    -horizontal_points, zeros(size(horizontal_points));
    horizontal_points, zeros(size(horizontal_points));
    x1_bounds(2)*ones(size(vertical_point)), -vertical_point];


%% Visualization Parameters
% Colors
quiver_color_map = abyss;
trajectory_color = [0, 0.447, 0.741];

% Lines
quiver_line_width = 2;
trajectory_line_width = 1;

label_font_size = 25;

% 
xticks = [-2*pi, -pi, 0, pi, 2*pi];
xticklabels = {'$-2\pi$', '$\pi$','$0$','$\pi$', '$2\pi$'};
yticks = [-2, 0, 2];

%% Figure and Axes Setups

f1 = figure(Colormap=quiver_color_map, Position=[118, 314, 650, 250]);
ax = axes(f1, NextPlot='add', Box='on',...
    XTick=xticks, YTick=yticks,...
    XTickLabel=xticklabels,...
    XGrid='on', YGrid='on',...
    Xlim=x1_bounds, Ylim=x2_bounds,...
    TickLabelInterpreter='latex', FontSize=label_font_size);

xlabel(ax, '$x_1$', Interpreter='latex', FontSize=label_font_size);
ylabel(ax, '$x_2$', Interpreter='latex', FontSize=label_font_size);

cindex = linspace(0,1,length(quiver_color_map))';

%% Computations and Plotting

[x1, x2] = meshgrid(x1_bounds(1):x1_res:x1_bounds(2),...
    x2_bounds(1):x2_res:x2_bounds(2));


for i = 1:size(x0,1)
    
    x0_ = x0(i,:)';

    options = odeset('RelTol',1e-6,'Events', @(t,x)outofbound(t, x, x0_, x1_bounds, x2_bounds));


    [~,x] = ode45(f,[0,20],x0_, options);
        plot(ax, x(:,1), x(:,2), Color=[0.8, 0.8, 0.8],...
            LineWidth=trajectory_line_width);
    % for j = 1:size(x1,2)
    %     x0 = [x1(i,j); x2(i,j)];
    % 
    %      [~,x] = ode45(f,[0,10],x0, options);
    %     plot(ax, x(:,1), x(:,2), Color=[0.8, 0.8, 0.8],...
    %         LineWidth=trajectory_line_width);
    %     dx = f(0, x0);
    % 
    %     norm_dx(i,j) = norm(dx);
    %     dx1(i,j) = dx(1)/norm_dx(i,j);
    %     dx2(i,j) = dx(2)/norm_dx(i,j);
    % end
end

% max_norm = max(max(norm_dx));
% 
% for i = 1:size(x1,1)
%     for j = 1:size(x1,2)
%         quiver(ax, x1(i,j), x2(i,j), dx1(i,j), dx2(i,j), 0.2,...
%             Color=interp1(cindex, quiver_color_map, norm_dx(i,j)/max_norm),...
%             LineWidth=2, MaxHeadSize=5);
%     end
% end

plot(ax, xticks, zeros(size(xticks)), 'o',...
    Color=[0    0.4470    0.7410],...
    MarkerFaceColor= [0    0.4470    0.7410],...
    MarkerSize=5);

% [~,x] = ode45(f,[0,4],[-pi; 0.5], options);
% plot(ax, x(:,1), x(:,2), Color=[0    0.4470    0.7410],...
% LineWidth=3);
% 
% [~,x] = ode45(f,[0,4],[-pi; 2.5], options);
% plot(ax, x(:,1), x(:,2), Color=[0.8500    0.3250    0.0980],...
% LineWidth=3);

    

%% External Functions

function [position, isterminal, direction] = outofbound(t, x, x0, x1_bounds, x2_bounds)
  x1_limit = [-0.2, 0.2] + x1_bounds;
  x2_limit = [-0.2, 0.2] + x2_bounds;
  position = (x1_limit(1) < x(1) & x(1) < x1_limit(2) & ...
      x2_limit(1) < x(2) & x(2) < x1_limit(2));

  % if(t > 0.1)
  %     loop_condition = (norm(x - x0) > 0.1);
  % else
  %     loop_condition = 1;
  % end
  % position = position & loop_condition;
  position = 1;
  isterminal = 1; % Halt integration 
  direction = 0; % The zero can be approached from either direction
end

