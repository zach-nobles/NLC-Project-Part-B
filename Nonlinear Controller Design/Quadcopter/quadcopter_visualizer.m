%% Plotting script

figure
% CoM, motor 1, CoM, motor 2, CoM, motor 3, CoM, motor 4
xvec_body = [0, dist, 0, 0, 0, -dist, 0, 0];
yvec_body = [0, 0, 0, dist, 0, 0, 0, -dist];
zvec_body = [0, 0, 0, 0, 0, 0, 0, 0];
% Current rotation matrix
R_temp = eval(subs(RotM, [phi theta psi]', xArr(4:6, 1)));
% Transform to current position 
pos_inertial = R_temp * [xvec_body; yvec_body; zvec_body] + xArr(1:3, 1);

ph = plot3(pos_inertial(1, :), pos_inertial(2, :), pos_inertial(3, :));
hold on
grid on
plot3(xref, yref, zref, 'k:')
ph2 = plot3(xArr(1, 1), xArr(2, 1), xArr(3, 1), 'b-');
xlim([min(xArr(1, :)) - 1, max(xArr(1, :)) + 1])
ylim([min(xArr(2, :)) - 1, max(xArr(2, :)) + 1])
zlim([min(0, min(xArr(3, :)) - 1), max(xArr(3, :)) + 1])

for i = 2:steps
    R_temp = eval(subs(RotM, [phi theta psi]', xArr(4:6, i)));
    pos_inertial = R_temp * [xvec_body; yvec_body; zvec_body] + xArr(1:3, i);
    ph.XData = pos_inertial(1, :);
    ph.YData = pos_inertial(2, :);
    ph.ZData = pos_inertial(3, :);
    
    ph2.XData = xArr(1, 1:i);
    ph2.YData = xArr(2, 1:i);
    ph2.ZData = xArr(3, 1:i);
    drawnow
end