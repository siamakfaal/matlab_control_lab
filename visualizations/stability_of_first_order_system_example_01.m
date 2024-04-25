clc; clear; close all;

dx = @(t, x) (4*x - x^3)*exp(-0.3*x^2);

tspan = linspace(0,3)';

x0 = -3;


ax = axes(NextPlot='add', Box='on',...
    Xlim=tspan([1,end]), Ylim=[-3, 3],...
    TickLabelInterpreter='latex', FontSize=20);

xlabel(ax, '$t$', Interpreter='latex', FontSize=20);
ylabel(ax, '$x(t)$', Interpreter='latex', FontSize=20);

colors = parula;

while x0 <= 3
    [~,x] = ode45(dx, tspan, x0);
    plot(ax, tspan, x, Color=colors(randi(length(colors)),:), LineWidth=1);
    x0 = x0 + 0.02;
end