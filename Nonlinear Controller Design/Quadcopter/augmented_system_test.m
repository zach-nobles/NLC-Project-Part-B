
x0 = [0;0;3; 0;0;0; 0;0;0; 0;0;0; mass*9.81;0];
u0 = [0;0;0;0];

Ts = 0.01;
N = 1000;

xArr = zeros(14, 1001);
x = x0;
u = u0;
xArr(:, 1) = x;
for i = 1:N
    dx = nonlinear_dynamics_aug(x, u);
    x = x + Ts*dx;
    xArr(:, i+1) = x;
end

plot(0:Ts:Ts*N, xArr(1:3, :)')
legend('x', 'y', 'z')


