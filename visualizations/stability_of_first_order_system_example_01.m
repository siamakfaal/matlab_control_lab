clc; clear; close all;

f = @(t, x) (4*x - x.^3).*exp(-0.3*x.^2);

tspan = linspace(0,3)';
x0 = -3:0.02:3;
dx = f([], x0);

colors = parula;
ax = buildaxes(x0([1,end]), 1.2*[min(dx), max(dx)], tspan([1,end]));

plot(ax(1),dx,x0,LineWidth=2);
I = 1:20:length(x0);
quiver(ax(1), zeros(size(I)), x0(I), zeros(size(I)), sign(dx(I)), 0.2,...
    LineWidth=2);
yline(ax(1),[0 -2 2]);

for k=1:length(x0)
    [~,x] = ode45(f, tspan, x0(k));
    plot(ax(2), tspan, x, ...
        Color=colors(randi(length(colors)),:), ...
        LineWidth=2);
end


function ax = buildaxes(xrange, dxrange, trange)
ax(1) = subplot(1, 2, 1, NextPlot='add', Box='on',...
    Xlim=xrange, Ylim=dxrange,...
    TickLabelInterpreter='latex', FontSize=20);
ax(2) = subplot(1, 2, 2, NextPlot='add', Box='on',...
    Xlim=trange, Ylim=xrange,...
    TickLabelInterpreter='latex', FontSize=20);

linkaxes([ax(1), ax(2)], 'y');

xlabel(ax(1), '$\dot{x}(t)$', Interpreter='latex', FontSize=20);
ylabel(ax(1), '$x(t)$', Interpreter='latex', FontSize=20);
xlabel(ax(2), '$t$', Interpreter='latex', FontSize=20);
ylabel(ax(2), '$x(t)$', Interpreter='latex', FontSize=20);

xline(ax(1),0);
end