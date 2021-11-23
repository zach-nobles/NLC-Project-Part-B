%% x

Ap_1 = A1;
Bp_1 = B1(:, 1);
Rp_1 = 0.5;
Qp_1 = diag([10 100 100 100]);

%Kp_1 = lqr(Ap_1, Bp_1, Qp_1, Rp_1);

poles_1 = [-5 -3 -2 -2];
%Kp_1 = acker(Ap_1, Bp_1, poles_1);

xp_1 = [3;0;0;0];
xp_1_Arr = zeros(4, steps);
xp_1_Arr(:, 1) = xp_1;

pTraj_1 = [zTraj_x; zTraj_dx; zTraj_ddx; zTraj_dddx];
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
