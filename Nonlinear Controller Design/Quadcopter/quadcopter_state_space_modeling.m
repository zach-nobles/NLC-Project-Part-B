%% Script for nonlinear dynamics
% Initialize symbolic variables for state, controls, and other parameters
syms x y z phi theta psi u v w p q r m Ixx Iyy Izz g l u1 u2 u3 u4 real
pos = [x; y; z]; % Position
eul = [phi; theta; psi]; % Attitude
vel_body = [u; v; w]; % Velocity (body frame)
omega_body = [p; q; r]; % Angular velocity (body frame)
u_con = [u1;u2;u3;u4]; % Thrust taux tauy tauz

% Get rotation matrices with respect to each axis
Rx_gen = @(theta) [1 0 0; 0 cos(theta) sin(theta); 0 -sin(theta) cos(theta)];
Ry_gen = @(theta) [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
Rz_gen = @(theta) [cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0; 0 0 1];

% Plug in for all Euler angles
Rx = Rx_gen(phi);
Ry = Ry_gen(theta);
Rz = Rz_gen(psi);

% Multiply to get the inertial-to-body-frame transformation
RotM = Rx * Ry * Rz;
posdot = RotM' * vel_body; % derivative of position

% Matrix for converting attitude
T = [1 sin(phi)*tan(theta) cos(phi)*tan(theta); 
   0 cos(phi) -sin(phi); 0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
euldot = T * omega_body; % derivative of attitude

% Get the force on the body
Fb = RotM*[0; 0; m*g] + [0; 0; u_con(1)]; % g = -9.81
% This can be used to find acceleration (body frame)
veldot = 1/m * Fb - cross(omega_body, vel_body);

J = [Ixx 0 0; 0 Iyy 0; 0 0 Izz]; % inertia matrix
taub = [u_con(2); u_con(3); u_con(4)]; % force acting on body
% These can be used to get angular acceleration (body frame)
omegadot = simplify(J^-1 * (taub - cross(omega_body, J*omega_body)));

% Compose the state vector and its derivative
states = [pos; eul; vel_body; omega_body];
statesdot = [posdot; euldot; veldot; omegadot];

% Take Jacobian
Asym = jacobian(statesdot, states);
Bsym = jacobian(statesdot, u_con);

% Now we have symbolic expressions for A and B as a function of state,
% controls, and vehicle parameters
