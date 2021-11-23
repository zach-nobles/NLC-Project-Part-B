% Change - z-axis down, state is different

function state_dot = nonlinear_dynamics(state, controls)
% state = [x y z phi theta psi xdot ydot zdot p q r]'
% controls = [T taux tauy tauz]'

m = 0.8;
g = 9.81;
J = diag([0.005, 0.005, 0.009]);
l = 0.33/2;

pos = state(1:3);
eul = state(4:6);
vel_in = state(7:9);
omega_body = state(10:12);

Rx_gen = @(theta) [1 0 0; 0 cos(theta) sin(theta); 0 -sin(theta) cos(theta)];
Ry_gen = @(theta) [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
Rz_gen = @(theta) [cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0; 0 0 1];

Rx = Rx_gen(eul(1));
Ry = Ry_gen(eul(2));
Rz = Rz_gen(eul(3));

RotM = Rx * Ry * Rz;
posdot = vel_in;

T = [1 sin(eul(1))*tan(eul(2)) cos(eul(1))*tan(eul(2)); 
   0 cos(eul(1)) -sin(eul(1)); 0 sin(eul(1))/cos(eul(2)) cos(eul(1))/cos(eul(2))];
euldot = T * omega_body;

Fb = [0; 0; m*g] + RotM'*[0; 0; -controls(1)]; % g = +9.81
veldot = 1/m * Fb;

taub = [controls(2); controls(3); controls(4)]; 
omegadot = J^-1 * (taub - cross(omega_body, J*omega_body));

state_dot = [posdot; euldot; veldot; omegadot];
end