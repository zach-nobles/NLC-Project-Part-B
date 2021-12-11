%% x

Ap_1 = A1;
Bp_1 = B1(:, 1);
Rp_1 = 0.5;
Qp_1 = diag([10 100 100 100]);

%Kp_1 = lqr(Ap_1, Bp_1, Qp_1, Rp_1);

poles_1 = [-5 -3 -2 -2];
Kp_1 = acker(Ap_1, Bp_1, poles_1);

xp_1 = [3;0;0;0];
xp_1_Arr = zeros(4, steps);
xp_1_Arr(:, 1) = xp_1;

pTraj_1 = [zTraj_z; zTraj_dz; zTraj_ddz; zTraj_dddz];
v_1 = 0;
v_1_Arr = zeros(1, steps);
v_1_Arr(1) = v_1;

target = [1;0;0;0];


for i = 2:steps
    dxp_1 = [xp_1(2:4); v_1];
    xp_1 = xp_1 + Ts*dxp_1;

    v_1 = -Kp_1*(xp_1 - pTraj_1(:, i));
    %v_1 = -Kp_1*(xp_1 - target);
    xp_1_Arr(:, i) = xp_1;
    v_1_Arr(i) = v_1;
end

plot(xp_1_Arr' - pTraj_1')
legend('x', 'dx', 'ddx', 'dddx')

max(abs(xp_1_Arr' - pTraj_1'))

%% y

Kp_2 = Kp_1;
%Kp_2(2) = Kp_2(2)*2;
pTraj_2 = [zTraj_y; zTraj_dy; zTraj_ddy; zTraj_dddy];

xp_2 = [0;0;0;0];
xp_2_Arr = zeros(4, steps);
xp_2_Arr(:, 1) = xp_2;

v_2 = 0;
v_2_Arr = zeros(1, steps);
v_2_Arr(1) = v_2;

for i = 2:steps
    dxp_2 = [xp_2(2:4); v_2];
    xp_2 = xp_2 + Ts*dxp_2;

    v_2 = -Kp_2*(xp_2 - pTraj_2(:, i));
    %v_1 = -Kp_1*(xp_1 - target);
    xp_2_Arr(:, i) = xp_2;
    v_2_Arr(i) = v_2;
end

plot(xp_2_Arr' - pTraj_2')
legend('x', 'dx', 'ddx', 'dddx')

max(abs(xp_2_Arr' - pTraj_2'))



%% psi
Ap_4 = A2;
Bp_4 = B4(:, 4);
%Rp_4 = 0.5;
%Qp_4 = diag([10 100]);
%Kp_1 = lqr(Ap_1, Bp_1, Qp_1, Rp_1);

poles_4 = [-5 -3];
Kp_4 = acker(Ap_4, Bp_4, poles_4);

pTraj_4 = [zTraj_psi; zTraj_dpsi];

xp_4 = [0;0];
xp_4_Arr = zeros(2, steps);
xp_4_Arr(:, 1) = xp_4;

v_4 = 0;
v_4_Arr = zeros(1, steps);
v_4_Arr(1) = v_4;

for i = 2:steps
    dxp_4 = [xp_4(2); v_4];
    xp_4 = xp_4 + Ts*dxp_4;

    v_4 = -Kp_4*(xp_4 - pTraj_4(:, i));
    %v_1 = -Kp_1*(xp_1 - target);
    xp_4_Arr(:, i) = xp_4;
    v_4_Arr(i) = v_4;
end

plot(xp_4_Arr' - pTraj_4')
legend('x', 'dx', 'ddx', 'dddx')

max(abs(xp_4_Arr' - pTraj_4'))
%% z, straight up

Ap_5 = A1;
Bp_5 = B1(:, 1);
Rp_5 = 0.5;
Qp_5 = diag([10 100 100 100]);

Kp_5 = lqr(Ap_5, Bp_5, Qp_5, Rp_5);

%poles_1 = [-5 -3 -2 -2];
%Kp_1 = acker(Ap_1, Bp_1, poles_1);

Kp_5 = [5 1400 150 12];

z0 = 1;
steps = 1201;
zTraj_z = z0 + linspace(0, 3, steps);
zTraj_dz = [diff(zTraj_z)/Ts, 0];
zTraj_ddz = zeros(1, steps);
zTraj_dddz = zeros(1, steps);
pTraj_5 = [zTraj_z; zTraj_dz; zTraj_ddz; zTraj_dddz];

xp_5 = [z0;0;0;0];
xp_5_Arr = zeros(4, steps);
xp_5_Arr(:, 1) = xp_5;

v_5 = 0;
v_5_Arr = zeros(1, steps);
v_5_Arr(1) = v_5;

for i = 2:steps
    dxp_5 = [xp_5(2:4); v_5];
    xp_5 = xp_5 + Ts*dxp_5;

    v_5 = -Kp_5*(xp_5 - pTraj_5(:, i));
    %v_1 = -Kp_1*(xp_1 - target);
    xp_5_Arr(:, i) = xp_5;
    v_5_Arr(i) = v_5;
end

plot(xp_5_Arr' - pTraj_5')
legend('x', 'dx', 'ddx', 'dddx')

figure(2)
plot(xp_5_Arr(1, :)' - pTraj_5(1, :)')

error_mat_partial = xp_5_Arr - pTraj_5;
%% y, sinusoidal

Ap_6 = A1;
Bp_6 = B1(:, 1);

%Kp_6 = [5 1400 150 12];
Kp_6 = [60 92 51 12];

roll_steps = 501;
straight_steps = 200;
steps = roll_steps + 2*straight_steps;
barrel_amp = 1; % radius of roll

z0 = 1;
zTraj_y = [zeros(1, straight_steps), barrel_amp*sin(2*pi*(0:roll_steps-1)/(roll_steps-1)), ...
    zeros(1, straight_steps)];
zTraj_dy = [diff(zTraj_y)/Ts, 0];
zTraj_ddy = [diff(zTraj_dy)/Ts, 0];
%zTraj_ddy = min(1.6, max(zTraj_ddy, -1.6)); % saturate 7
zTraj_ddy(straight_steps) = 0;
zTraj_ddy(straight_steps + roll_steps - 1) = 0;
zTraj_dddy = [diff(zTraj_ddy)/Ts, 0];
%zTraj_dddy = min(2, max(zTraj_dddy, -2)); % saturate 8

pTraj_6 = [zTraj_y; zTraj_dy; zTraj_ddy; zTraj_dddy];

xp_6 = [0;0;0;0];
xp_6_Arr = zeros(4, steps);
xp_6_Arr(:, 1) = xp_6;

v_6 = 0;
v_6_Arr = zeros(1, steps);
v_6_Arr(1) = v_5;

for i = 2:steps
    dxp_6 = [xp_6(2:4); v_6];
    xp_6 = xp_6 + Ts*dxp_6;

    v_6 = -Kp_6*(xp_6 - pTraj_6(:, i));
    %v_1 = -Kp_1*(xp_1 - target);
    xp_6_Arr(:, i) = xp_6;
    v_6_Arr(i) = v_6;
end

plot(xp_6_Arr(1, :)')
hold on
plot(pTraj_6(1, :)', 'k--')

%plot(xp_6_Arr' - pTraj_6')
%legend('y', 'dy', 'ddy', 'dddy')

%figure(2)
%plot(xp_6_Arr(1, :)' - pTraj_6(1, :)')

error_mat_partial = xp_6_Arr - pTraj_6;



