%% Trajectory

spiral_steps = 300;
cycles = 5;
steps = cycles*spiral_steps + 1;
spiral_amp = 0.5; % radius
z0 = 1;
xref = spiral_amp*cos(2*pi*(0:steps-1)/spiral_steps);
yref = spiral_amp*sin(2*pi*(0:steps-1)/spiral_steps);
zref = z0 + linspace(0, 3, steps);
Ts = 0.01;
tt = 0:Ts:steps*Ts;

phiref = zeros(1, steps);
thetaref = zeros(1, steps);
psiref = linspace(0, -2*cycles*pi, steps);

velxref = [0, diff(xref)/Ts];
velyref = [0, diff(yref)/Ts];
velzref = [0, diff(zref)/Ts];

pref = [0, diff(phiref)/Ts];
qref = [0, diff(thetaref)/Ts];
rref = [0, diff(psiref)/Ts];


trajRef = [xref; yref; zref; phiref; thetaref; psiref; 
    velxref; velyref; velzref; pref; qref; rref];

%% LQR Rollout
uref = ubar; % Same reference thrust
x0 = [0 0 z0 0 0 0 0 0 0 0 0 0]'; % start on straight line
% Initialize x and u
xi = x0;
u = uref;
% Initialize matrices containing all states and controls
xArr = zeros(12, steps);
uArr = zeros(4, steps);
xArr(:, 1) = xi;
uArr(:, 1) = u;

% Rollout:
for i = 1:steps
    % Update x at each time with nonlinear_dynamics
    xi = xi + Ts*nonlinear_dynamics(xi, u);
    % Use LQR gains to find new control
    error = xi - trajRef(:, i);
    u = -K_lqr*error;
    % Place inside matrices
    xArr(:, i+1) = xi;
    uArr(:, i+1) = u;
end

plot3(xArr(1, :), xArr(2, :), xArr(3, :))
hold on
plot3(trajRef(1, :), trajRef(2, :), trajRef(3, :), 'k--')
zlim([0 4])

%% Trajectory in z
zTraj_x = xref;
zTraj_dx = [diff(zTraj_x)/Ts, 0];
zTraj_ddx = [diff(zTraj_dx)/Ts, 0];
zTraj_dddx = [diff(zTraj_ddx)/Ts, 0];
zTraj_dddx = min(15, max(zTraj_dddx, -15)); % saturate
zTraj_y = yref;
zTraj_dy = [diff(zTraj_y)/Ts, 0];
zTraj_ddy = [diff(zTraj_dy)/Ts, 0];
zTraj_ddy = min(5, max(zTraj_ddy, -5)); % saturate
zTraj_dddy = [diff(zTraj_ddy)/Ts, 0];
zTraj_dddy = min(15, max(zTraj_dddy, -15)); % saturate
zTraj_z = zref;
zTraj_dz = [diff(zTraj_z)/Ts, 0];
zTraj_ddz = zeros(1, steps);
zTraj_dddz = zeros(1, steps);
%zTraj_ddz = [diff(zTraj_dz)/Ts, 0];
%zTraj_dddz = [diff(zTraj_ddz)/Ts, 0];
zTraj_psi = psiref;
zTraj_dpsi = [diff(psiref)/Ts, 0];


zTraj_full = [zTraj_x; zTraj_dx; zTraj_ddx; zTraj_dddx; 
    zTraj_y; zTraj_dy; zTraj_ddy; zTraj_dddy
    zTraj_z; zTraj_dz; zTraj_ddz; zTraj_dddz; zTraj_psi; zTraj_dpsi];
%% Pole placement controller

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

poles_mini = [-5 -3 -2 -2];

K_mini = acker(A1, B1(:, 1), poles_mini);
K_alt = [K_mini, zeros(1, 10); zeros(1, 4), K_mini, zeros(1, 6); 
    zeros(1, 8), K_mini, zeros(1, 2); zeros(1, 12), 15 8];

% X adjustments % default - [60, 92, 51, 12]
%K_alt(1,1) = 70;
%K_alt(1,2) = 200;

% Y adjustments % default - [60, 92, 51, 12]
%K_alt(2, 5) = 70;
%K_alt(2, 6) = 200;
%K_alt(2, 7) = 40;

% Z adjustments % default - [60, 92, 51, 12] % tuned - [5 1400 150 12]
%K_alt(3, 9) = 8;
%K_alt(3, 10) = 500;
%K_alt(3, 11) = 50; % 150
%K_alt(3, 12) = 35;

%K_alt(3, 9) = 20;
%K_alt(3, 10) = 200;
%K_alt(3, 11) = 0;
%K_alt(3, 12) = 0;

%K_alt(3, 9:12) = [5 1400 150 12];

%K_alt(4, 13) = 0;

%%

x0 = [spiral_amp 0 z0 0 0 0 0 0 0 0 0 0 mass*9.81 0]'; % Now state has 14 elements
% For nonsingular delta, we need nonzero gamma
% Initialize x and u
xi = x0;
ubar = [0;0;0;0]; % new reference - don't change u
u = ubar;
% Initialize matrices containing all states and controls
xArr = zeros(14, steps);
uArr = zeros(4, steps);
xArr(:, 1) = xi;
uArr(:, 1) = u;

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
    %tic
    % Update x at each time with nonlinear_dynamics
    dx = nonlinear_dynamics_aug(xi, u);
    xi = xi + Ts*dx;

    % Use feedback linearization to find new control
    delta_eval = delta_mat_evaluator(xi');
    b_eval = b_vec_evaluator(xi');

    beta_mat = delta_eval^-1;
    alpha_vec = -beta_mat * b_eval;


    zi = z_evaluator(xi');
    error = zi - zTraj_full(:, i);

    v_con = -K_alt*error; % pole placement
    %v_con = -K_lqr*error; % lqr
    u = alpha_vec + beta_mat * v_con;
    
    % Place inside matrices
    xArr(:, i+1) = xi;
    uArr(:, i+1) = u;
    zArr(:, i+1) = zi;
    %toc
end

ind = steps;

% plot3(xArr(1, 1:ind), xArr(2, 1:ind), xArr(3, 1:ind))
% hold on
% plot3(xref, yref, zref)
% zlim([1 5])
% 
% figure(2)
% plot(xArr(1:3, 2:end)' - zTraj_full([1,5,9], :)')
% legend('x', 'y', 'z')

disp('Errors')
%max(abs(xArr(1:3, 2:end) - zTraj_full([1 5 9], :)), [], 2)'
max_err_fbl = max(abs(xArr([1:3, 6], 2:end) - zTraj_full([1 5 9 13], 1:end)), [], 2)'
mean_err_fbl = mean(abs(xArr([1:3, 6], 201:end) - zTraj_full([1 5 9 13], 200:end)), 2)'
error_mat = xArr(1:12, 2:end) - trajRef;
error_mat_z = zArr(9:12, 2:end) - [zTraj_z; zTraj_dz; zTraj_ddz; zTraj_dddz];

figure(3)
plot(tt, zArr([1 5 9], :)')
hold on
plot(tt(1:end-1), zTraj_full([1 5 9], :)', 'k--')
legend('x', 'y', 'z', '', '', '')

%% Plot xyz trajectory
plot3(xArr(1, :), xArr(2, :), xArr(3, :), 'b-')
hold on
plot3(trajRef(1, :), trajRef(2, :), trajRef(3, :), 'k--')
grid on
legend('Actual Path', 'Reference Trajectory')
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
%zlim([0, 3.5])
%% Controls

plot(tt, xArr(13, :)', 'k-')
hold on
xlabel('Time [s]')
yyaxis left
ylabel('Force [N]')
yyaxis right
plot(tt, uArr(2, :)', 'b-')
plot(tt, uArr(3, :)', 'r-')
plot(tt, uArr(4, :)', 'm-')
ylabel('Torques [Nm]')
legend('T', '\tau_x', '\tau_y', '\tau_z')
xlim([-1, tt(end)+1])
%% Trajectory length

diffs_temp = diff(xArr(1:3, :), 1, 2);
segments = vecnorm(diffs_temp);
trajLength = sum(segments);

