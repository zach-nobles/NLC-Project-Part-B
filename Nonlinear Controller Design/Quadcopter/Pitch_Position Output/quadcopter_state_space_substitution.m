%% Given symbolic matrices, find current values for continuous time A and B
% Put all variables together
vars = [x y z phi theta psi xdot ydot zdot p q r m Ixx Iyy Izz g l u1 u2 u3 u4]';

state = [0 0 0 0 0 0 0 0 0 0 0 0]'; % linearize about hover (z=0 not important here)
mass = 0.8; % mass of vehicle
inertia = [0.005, 0.005, 0.009]'; % inertia matrix, expressed as [Ixx, Iyy, Izz]
dist = 0.33/2; % distance from motor to CoM
ubar = [mass*9.81; 0; 0; 0]; % Thrust = mg, no torque for equilibrium
controls = ubar;

% Put all actual numerical values together in the same order as vars
values = [state; mass; inertia; 9.81; dist; controls];
% Now substitute this into A and B to get real matrices
A = eval(subs(Asym, vars, values));
B = eval(subs(Bsym, vars, values));
