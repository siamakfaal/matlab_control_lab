clc; clear; close all;

% Initial guess for the parameters to be shot
x0 = [-1;0];
tspan = 0:0.01:3;
w = [-2, 2];
ti = linspace(tspan(1), tspan(end), 3)';
ui = randi(w, size(ti));

% % Use fsolve to adjust control
% ui = fsolve(@(ui)shooting(x0, tspan, ti, ui), ui);

% Use fmincon to find optimal control
lb = repmat(w(1),size(ui));
ub = repmat(w(2),size(ui));

options = optimoptions('fmincon',...
    Algorithm='interior-point',...
    Display='iter-detailed', ...
    MaxFunctionEvaluations=6000,...
    MaxIterations=200);

ui = fmincon(@(ui)shooting(x0, tspan, ti, ui), ...
    ui, ...
    [], [], ... % A = [], b = []
    [], [], ... % Aeq = [], beq = []
    lb, ub, ...
    [], ...
    options);

% Solve ODE with optimized control
[t, x] = ode45(@(t,x)dynamics(t, x, control(t, ti, ui)), tspan, x0);

% Plot results
plot(t, x,'-');
title('Solution of BVP using the shooting method');
xlabel('t');


function res = shooting(x0, tspan, ti, ui)
    [~, x] = ode45(@(t,x)dynamics(t, x, control(t, ti, ui)), tspan, x0);
    res = norm(x(end,:) - [1,-1]);
end

function dx = dynamics(t, x, u)
    dx = [x(2); u];
end

function u = control(t, ti, ui)
    u = ui(sum(t >= ti));
end