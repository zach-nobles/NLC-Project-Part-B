%% Lemniscate Trajectory
k = 2; % factor to slow down trajectory
% Number of steps to complete a trajectory: 2*a*k
Ts = 0.01; % timestep for discretization
a = 3; % trajectory size
z0 = 3; % initial height
tt = 0:Ts:2*pi*k; % parametrized t
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

%% Rollout (SKIP)
x0 = [a 0 z0 0 0 0 0 0 0 0 0 0 0.01 0]'; % Now state has 14 elements
% For nonsingular delta, we need nonzero gamma
% Initialize x and u
xi = x0;
u = [0;0;0;0];
% Initialize matrices containing all states and controls
xArr = zeros(14, steps);
uArr = zeros(4, steps);
xArr(:, 1) = xi;
uArr(:, 1) = u;

gamma_init = 0;
sigma_init = 0;

% Rollout:
for i = 1:steps
    % Update x at each time with nonlinear_dynamics
    xi = xi + Ts*nonlinear_dynamics_aug(xi, u);
    % Already uses integrators to update gamma and sigma

    % Use feedback linearization to find new control
    delta_eval = eval(subs(delta_mat, states_aug, xi));
    b_eval = eval(subs(b_vec, states_aug, xi));

    if rank(delta_eval) < 4
        disp('test')
        delta_eval = delta_eval + 0.0001*eye(4);
    end
    beta = delta_eval^-1;
    alpha = -beta * b_eval;

    u_alt = alpha + beta * ubar;
    u = u_alt;
    % Place inside matrices
    xArr(:, i+1) = xi;
    uArr(:, i+1) = u;
end


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
%% Trajectory in z
zTraj_x = xref;
zTraj_dx = [0, diff(zTraj_x)/Ts];
zTraj_ddx = [0, diff(zTraj_dx)/Ts];
zTraj_dddx = [0, diff(zTraj_ddx)/Ts];
zTraj_dddx = min(0.05, max(zTraj_dddx, -0.05)); % saturate
zTraj_y = yref;
zTraj_dy = [0, diff(zTraj_y)/Ts];
zTraj_ddy = [0, diff(zTraj_dy)/Ts];
zTraj_ddy = min(0.2, max(zTraj_ddy, -0.2)); % saturate
zTraj_dddy = [0, diff(zTraj_ddy)/Ts];
zTraj_dddy = min(0.05, max(zTraj_dddy, -0.05)); % saturate
zTraj_z = zref;
zTraj_dz = [0, diff(zTraj_z)/Ts];
zTraj_ddz = [0, diff(zTraj_dz)/Ts];
zTraj_dddz = [0, diff(zTraj_ddz)/Ts];
zTraj_psi = psiref;
zTraj_dpsi = [0, diff(psiref)/Ts];

zTraj_full = [zTraj_x; zTraj_dx; zTraj_ddx; zTraj_dddx; 
    zTraj_y; zTraj_dy; zTraj_ddy; zTraj_dddy
    zTraj_z; zTraj_dz; zTraj_ddz; zTraj_dddz; zTraj_psi; zTraj_dpsi];
%%
poles_mini = [-5 -3 -2 -2];

K_mini = acker(A1, B1(:, 1), poles_mini);
K_alt = [K_mini, zeros(1, 10); zeros(1, 4), K_mini, zeros(1, 6); 
    zeros(1, 8), K_mini, zeros(1, 2); zeros(1, 12), 15 8];
%%
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
delta_mat_evaluator = matlabFunction(delta_mat, 'Vars', {[x y z phi theta psi u v w p q r gam sig]});
b_vec_evaluator = matlabFunction(b_vec, 'Vars', {[x y z phi theta psi u v w p q r gam sig]});

z_eval = simplify([yvec(1); Lf_h_aug(1); Lf2_h_aug(1); Lf3_h_aug(1); 
    yvec(2); Lf_h_aug(2); Lf2_h_aug(2); Lf3_h_aug(2); 
    yvec(3); Lf_h_aug(3); Lf3_h_aug(3); Lf3_h_aug(3); 
    yvec(4); Lf_h_aug(4)]);
z_evaluator = matlabFunction(z_eval, 'Vars', {[x y z phi theta psi u v w p q r gam sig]});


zArr = zeros(14, steps);
zi = z_evaluator(xi');
zArr(:, 1) = zi;

for i = 1:steps
    %tic
    % Update x at each time with nonlinear_dynamics
    dx = nonlinear_dynamics_aug(xi, ui);
    xi = xi + Ts*dx;
    % Already uses integrators to update gamma and sigma

    % Use feedback linearization to find new control
    delta_eval = delta_mat_evaluator(xi');
    b_eval = b_vec_evaluator(xi');

%     if rank(delta_eval) < 4
%         disp('test')
%         delta_eval = delta_eval + 0.0001*eye(4);
%     end
    beta_mat = delta_eval^-1;
    alpha_vec = -beta_mat * b_eval;


    zi = z_evaluator(xi');
    error = zi - zTraj_full(:, i);

    %error = [xi(1:12) - trajRef(:, i); 0; 0];
    v_con = -K_alt*error;
    %v_con = -Kc*error;
    ui = alpha_vec + beta_mat * v_con;
    
    % Place inside matrices
    xArr(:, i+1) = xi;
    uArr(:, i+1) = ui;
    %toc
end
%%
plot(xArr(1:3, 1:3200)' - zTraj_full([1 5 9], 1:3200)')
legend('x', 'y', 'z')

%plot3(xArr(1, :), xArr(2, :), xArr(3, :))
%hold on
%plot3(xref, yref, zref)
%zlim([2 4])
