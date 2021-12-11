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
%% Rollout: Take 2
A1 = [0 1 0 0; 0 0 1 0; 0 0 0 1; 0 0 0 0];
A2 = [0 1; 0 0];
B1 = [0 0 0 0; 0 0 0 0; 0 0 0 0; 1 0 0 0];
B2 = [0 0 0 0; 0 0 0 0; 0 0 0 0; 0 1 0 0];
B3 = [0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 1 0];
B4 = [0 0 0 0; 0 0 0 1];
C1 = [1 0 0 0];
C2 = [1 0];

Ac = [A1, zeros(4, 10); zeros(4, 4), A1, zeros(4, 6); ...
    zeros(4, 8), A1, zeros(4, 2); zeros(2, 12) A2];
Bc = [B1; B2; B3; B4];
Cc = [C1, zeros(1, 10); zeros(1, 4), C1, zeros(1, 6); ...
    zeros(1, 8), C1, zeros(1, 2); zeros(1, 12), C2];

[Ac_d, Bc_d] = continuous_to_discrete(Ac, Bc, Ts);
%% Trajectory in z
zTraj_x = xref;
% zTraj_dx = [diff(zTraj_x)/Ts, 0];
% zTraj_ddx = [diff(zTraj_dx)/Ts, 0];
% zTraj_dddx = [diff(zTraj_ddx)/Ts, 0];
zTraj_dx = [0, diff(zTraj_x)/Ts];
zTraj_ddx = [0, diff(zTraj_dx)/Ts];
zTraj_dddx = [0, diff(zTraj_ddx)/Ts];
zTraj_dddx = min(5, max(zTraj_dddx, -5)); % saturate

zTraj_y = yref;
% zTraj_dy = [diff(zTraj_y)/Ts, 0];
% zTraj_ddy = [diff(zTraj_dy)/Ts, 0];
% zTraj_ddy = min(15, max(zTraj_ddy, -15)); % saturate
% zTraj_dddy = [diff(zTraj_ddy)/Ts, 0];
% zTraj_dddy = min(20, max(zTraj_dddy, -20)); % saturate
zTraj_dy = [0, diff(zTraj_y)/Ts];
zTraj_ddy = [0, diff(zTraj_dy)/Ts];
%zTraj_ddy = min(15, max(zTraj_ddy, -15)); % saturate
zTraj_dddy = [0, diff(zTraj_ddy)/Ts];
zTraj_dddy = min(20, max(zTraj_dddy, -20)); % saturate

zTraj_z = zref;
zTraj_dz = [diff(zTraj_z)/Ts, 0];
zTraj_ddz = [diff(zTraj_dz)/Ts, 0];
zTraj_dddz = [diff(zTraj_ddz)/Ts, 0];
zTraj_psi = psiref;
zTraj_dpsi = [diff(psiref)/Ts, 0];

zTraj_full = [zTraj_x; zTraj_dx; zTraj_ddx; zTraj_dddx; 
    zTraj_y; zTraj_dy; zTraj_ddy; zTraj_dddy
    zTraj_z; zTraj_dz; zTraj_ddz; zTraj_dddz; zTraj_psi; zTraj_dpsi];
%% Pole placement controller

%poles_mini = [-10 -2 -2 -1];
poles_mini = [-5 -3 -2 -2];

K_mini = acker(A1, B1(:, 1), poles_mini);
K_alt = [K_mini, zeros(1, 10); zeros(1, 4), K_mini, zeros(1, 6); 
    zeros(1, 8), K_mini, zeros(1, 2); zeros(1, 12), 15 8];

%%

%des_poles = [-0.5 -0.75 -1 -1.25 -1.5 -1.6 -1.7 -1.8 -1.9 -2 -2.1 -2.2 -2.3 -2.4];
%Kc = place(Ac, Bc, des_poles);

x0 = [a 0 z0 0 0 0 0 0 0 0 0 0 mass*9.81 0]'; % Now state has 14 elements
% For nonsingular delta, we need nonzero gamma
% Initialize x and u
xi = x0;
ubar = [0;0;0;0]; % new reference - don't change u
ui = ubar;
% Initialize matrices containing all states and controls
xArr = zeros(14, steps);
uArr = zeros(4, steps);
xArr(:, 1) = xi;
uArr(:, 1) = ui;

% Turn symbolic expression evaluators into functions for speed
delta_mat_evaluator = matlabFunction(delta_mat, 'Vars', {[x y z phi theta psi xdot ydot zdot p q r gam sig]});
b_vec_evaluator = matlabFunction(b_vec, 'Vars', {[x y z phi theta psi xdot ydot zdot p q r gam sig]});

z_eval = simplify([yvec(1); Lf_h_aug(1); Lf2_h_aug(1); Lf3_h_aug(1); 
    yvec(2); Lf_h_aug(2); Lf2_h_aug(2); Lf3_h_aug(2); 
    yvec(3); Lf_h_aug(3); Lf2_h_aug(3); Lf3_h_aug(3); 
    yvec(4); Lf_h_aug(4)]);
z_evaluator = matlabFunction(z_eval, 'Vars', {[x y z phi theta psi xdot ydot zdot p q r gam sig]});


zArr = zeros(14, steps);
zi = z_evaluator(xi');
zArr(:, 1) = zi;

for i = 1:steps
    % Update x at each time with nonlinear_dynamics
    dx = nonlinear_dynamics_aug(xi, ui);
    xi = xi + Ts*dx;

    % Use feedback linearization to find new control
    delta_eval = delta_mat_evaluator(xi');
    b_eval = b_vec_evaluator(xi');

    beta_mat = delta_eval^-1;
    alpha_vec = -beta_mat * b_eval;


    zi = z_evaluator(xi');
    error = zi - zTraj_full(:, i);

    v_con = -K_alt*error; % pole placement
    %v_con = -Kc*error; % lqr
    ui = alpha_vec + beta_mat * v_con;

    ui(2) = min(0.4, max(-0.4, ui(2)));
    
    % Place inside matrices
    xArr(:, i+1) = xi;
    uArr(:, i+1) = ui;
    zArr(:, i+1) = zi;
end

ind = steps;

plot3(xArr(1, 1:ind), xArr(2, 1:ind), xArr(3, 1:ind))
hold on
plot3(xref, yref, zref)
zlim([2 4])

figure(2)

plot(tt, xArr(1:3, 2:end)' - zTraj_full([1,5,9], :)')
xlim([-0.5, tt(end) + 0.5])
xlabel('Time [s]')
ylabel('Position [m]')
legend('x', 'y', 'z')
disp('Errors')
%max(abs(xArr(1:3, 2:end) - zTraj_full([1 5 9], :)), [], 2)'
max_err_fbl = max(abs(xArr(1:3, 2:end) - zTraj_full([1 5 9], 1:end)), [], 2)'
mean_err_fbl = mean(abs(xArr(1:3, 201:end) - zTraj_full([1 5 9], 200:end)), 2)'

error_mat = xArr(1:12, 2:end) - trajRef;

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

%% Plot roll pitch yaw
ttemp = tt(1:ind);
plot(ttemp, xArr(4, 1:ind)', 'b-')
hold on
plot(ttemp, xArr(5, 1:ind)', 'r-')
plot(ttemp, xArr(6, 1:ind)', 'm-')
xlabel('Time [s]')
ylabel('Angle [rad]')
legend('\phi', '\theta', '\psi')


%% Controls

plot(ttemp, xArr(13, 1:ind)', 'k-')
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
xlim([-1, tt(end)+1])

% Maximum torques - 1.257, 1.257, 0.2145
% Maximum rate of change - 0.7542 0.7542 0.1287
% Thrust coefficient - 4.0687e-7, omega = 3093 for equilibrium
% Max thrust: omega = 4720 -> 36.26
% Thrust margin: 2.3287
