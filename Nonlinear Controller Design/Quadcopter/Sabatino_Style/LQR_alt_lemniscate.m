%% Lemniscate Trajectory
k = 2; % factor to slow down trajectory
loops = 2; % #loops
% Number of steps to complete a trajectory: 2*a*k
Ts = 0.01; % timestep for discretization
a = 3; % trajectory size
z0 = 3; % initial height
tt = 0:Ts:2*pi*k*loops; % parametrized t
xref = a*cos(tt/k)./(1+sin(tt/k).^2); % x and y are a function of t
yref = a*cos(tt/k).*sin(tt/k)./(1+sin(tt/k).^2);
zref = z0 * ones(size(xref)); % still holding constant height
phiref = zeros(size(xref));
thetaref = zeros(size(xref));
psiref = zeros(size(xref));
velxref = [0, diff(xref)/Ts]; % still based on moving between points
velyref = [0, diff(yref)/Ts];
velzref = zeros(size(xref));
pref = zeros(size(xref));
qref = zeros(size(xref));
rref = zeros(size(xref));

% Put them all together
steps = length(xref);
trajRef = [xref; yref; zref; phiref; thetaref; psiref; 
    velxref; velyref; velzref; pref; qref; rref];
% Also discretizing the system here
[Ad, Bd] = continuous_to_discrete(A, B, Ts);
%% Controls
% New set of maximum allowables
max_vals = [0.1 0.1 0.1 pi/24 pi/24 pi/2 100 100 20 8000/(180/pi) 8000/(180/pi) 8000/(180/pi)];
max_controls = [15 0.4 0.4 0.4];
% Use these with Bryson's rule for Q and R
Q = diag(1./(max_vals.^2));
R = diag(1./(max_controls.^2));
% Then get the latest K (although it's actually the same as 3 from the
% first script)
K_lqr = dlqr(Ad, Bd, Q, R, []);
%% Rollout
uref_lqr = [mass*9.81; 0; 0; 0]; % Same reference thrust
x0 = [a 0 z0 0 0 0 0 0 0 0 0 0]'; % new lemniscate starts at x = a
% Initialize x and u
xi = x0;
u = uref_lqr;
% Initialize matrices containing all states and controls
xArr = zeros(12, steps);
uArr = zeros(4, steps);
xArr(:, 1) = xi;
uArr(:, 1) = u;

A_cur = A;
B_cur = B;


Asym_state = eval(subs(Asym, [m Ixx Iyy Izz g l u1 u2 u3 u4]', ...
    [mass; inertia; 9.81; dist; ubar]));
A_evaluator = matlabFunction(Asym_state, 'Vars', ...
    {[x y z phi theta psi xdot ydot zdot p q r]});
Bsym_state = eval(subs(Bsym, [m Ixx Iyy Izz g l u1 u2 u3 u4]', ...
    [mass; inertia; 9.81; dist; ubar]));
B_evaluator = matlabFunction(Bsym_state, 'Vars', ...
    {[x y z phi theta psi xdot ydot zdot p q r]});


% Rollout:
for i = 1:steps
    tic
    % Update x at each time with nonlinear_dynamics
    xi = xi + Ts*nonlinear_dynamics(xi, u);
    values = [xi; mass; inertia; 9.81; dist; ubar];
    % Now substitute this into A and B to get real matrices
    A_cur = A_evaluator(xi');
    B_cur = B_evaluator(xi');
    [Ad, Bd] = continuous_to_discrete(A_cur, B_cur, Ts);
    K_lqr = dlqr(Ad, Bd, Q, R, []);
    % Use LQR gains to find new control
    u = -K_lqr*(xi - trajRef(:, i));
    % Place inside matrices
    xArr(:, i+1) = xi;
    uArr(:, i+1) = u;
    toc
end

max_err_lqr = max(abs(xArr(1:3, 2:end) - trajRef([1:3], 1:end)), [], 2)'
mean_err_lqr = mean(abs(xArr(1:3, 201:end) - trajRef(1:3, 200:end)), 2)'
%%
plot(tt, xArr(1:3, 2:end)' - trajRef(1:3, :)')
xlim([-0.5, tt(end) + 0.5])
xlabel('Time [s]')
ylabel('Position [m]')
legend('x', 'y', 'z')
%% Plot Position
ind = steps;
ttemp = (0:ind-1)*Ts;
plot(ttemp, xArr(1, 1:ind)', 'b-')
hold on
plot(ttemp, xArr(2, 1:ind)', 'r-')
plot(ttemp, xArr(3, 1:ind)', 'm-')
plot(ttemp, trajRef(1, 1:ind)', 'b--')
plot(ttemp, trajRef(2, 1:ind)', 'r--')
plot(ttemp, trajRef(3, 1:ind)', 'm--')
xlabel('Time [s]')
ylabel('Position [m]')
legend('x','y', 'z')
%% Plot xyz trajectory
plot3(xArr(1, :), xArr(2, :), xArr(3, :), 'b-')
hold on
plot3(trajRef(1, :), trajRef(2, :), trajRef(3, :), 'k--')
grid on
legend('Actual Path', 'Reference Trajectory')
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
zlim([0, 3.5])

%% Plot Controls

plot(ttemp, uArr(1, 1:ind)', 'k-')
hold on
xlabel('Time [s]')
yyaxis left
ylabel('Force [N]')
yyaxis right
plot(ttemp, uArr(2, 1:ind)', 'b-')
plot(ttemp, uArr(3, 1:ind)', 'r-')
plot(ttemp, uArr(4, 1:ind)', 'm-')
ylabel('Torques [Nm]')
legend('T', '\tau_x', '\tau_y', '\tau_z')
%% Plot roll pitch yaw
plot(ttemp, xArr(4, 1:ind)', 'b-')
hold on
plot(ttemp, xArr(5, 1:ind)', 'r-')
plot(ttemp, xArr(6, 1:ind)', 'm-')
xlabel('Time [s]')
ylabel('Angle [rad]')
legend('\phi', '\theta', '\psi')
