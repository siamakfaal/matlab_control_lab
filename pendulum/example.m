clc; clear all; close all;

%% System definition
m = 1;
l = 1;
g = 9.81;
p = pendulum(m, l, g);

%% Controller parameters

xd = [pi/4; 0];      % Desired state

roots = [-2, -1];   % Desired poles of the linear closed loop system
ke = 10;            % Energy controller gain
epsilon = 0.1;      % Switching region for hybrid energy-linear controller

kp = 50;
ki = 10;
kd = 5;

lower_bound = -25;
upper_bound = 25;

%% Controller design
% u = design_linear_controller(p, xd, [-2, -1]);
% u = design_energy_controller(p, xd, ke);
% u = hybrid_energy_pd_controller(p, xd, ke, roots, epsilon);
u = pid_controller(xd, kp, ki, kd);


%% Simulation
x0 = [pi/2; 0];   % initial condition [x1(0); x2(0)]
sim_time = 10;      % simulation time [s]
u_ = sat(u, lower_bound, upper_bound);

[t, x] = p.solve(sim_time, x0, u_, xd);

%% Results
p.plot(t,x, u_)
p.animate(t,x)


%% Control Functions
function u_ = sat(u, lower_bound, upper_bound)
    u_ = @(t, x) min( max(u(t,x) , lower_bound), upper_bound);
end

function u = linear_controller(pendulum, xd, roots)
    options = optimoptions('fsolve', 'Algorithm','levenberg-marquardt');
    ud = fsolve(@(u) pendulum.dynamics(xd,u), 0, options);
    e = sym('e%d',[2, 1], 'real');
    v = sym('v','real');
    % e = xd - x; v = ud - u => de = 0 - f(t, xd - e, ud - v)
    de = [0; 0] - pendulum.dynamics(xd - e, ud - v);
    A = double(subs(jacobian(de,e), [e; v], zeros(3,1)));
    B = double(subs(jacobian(de,v), [e; v], zeros(3,1)));
    if rank(ctrb(A,B)) ~= length(xd)
        fprintf('The system is not full state controllable');
        return
    end
    K = place(A, B, roots);
    u = @(t,x) ud + K*(xd - x(1:2));
end

function u = pid_controller(xd, kp, ki, kd)
    u = @(t, x) [kp, kd]*(xd - x(1:2)) + ki*x(3);
end

function u = energy_controller(pendulum, xd, ke)
    Ed = pendulum.energy(xd);
    u = @(t, x) -ke*(pendulum.energy(x) - Ed)*sign(x(2));
end

function u = hybrid_energy_pd_controller(pendulum, xd, ke, roots, epsilon)
    u1 = linear_controller(pendulum, xd, roots);
    u2 = energy_controller(pendulum, xd, ke);
    u = @(t,x) (norm(x(1:2) - xd) <= epsilon)*u1(t,x) +...
    (norm(x(1:2) - xd) > epsilon)*u2(t,x);
end

