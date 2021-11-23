%% Trajectory

roll_steps = 501;
straight_steps = 200;
steps = roll_steps + 2*straight_steps;
barrel_amp = 0.1; % radius of roll
z0 = 3;
xref = linspace(0, 2, steps);
yref = [zeros(1, straight_steps), barrel_amp*sin(2*pi*(0:roll_steps-1)/(roll_steps-1)), ...
    zeros(1, straight_steps)];
zref = [z0*ones(1, straight_steps), z0 + barrel_amp - barrel_amp*cos(2*pi*(0:roll_steps-1)/(roll_steps - 1)), ...
    z0*ones(1, straight_steps)];

phiref = [zeros(1, straight_steps), linspace(0, -2*pi, roll_steps), zeros(1, straight_steps)];
thetaref = zeros(1, steps);
psiref = zeros(1, steps);

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
    u = -K_lqr*(xi - trajRef(:, i));
    % Place inside matrices
    xArr(:, i+1) = xi;
    uArr(:, i+1) = u;
end

plot3(xArr(1, :), xArr(2, :), xArr(3, :))
hold on
plot3(trajRef(1, :), trajRef(2, :), trajRef(3, :), 'k--')
zlim([0 4])
%% FBL Rollout


