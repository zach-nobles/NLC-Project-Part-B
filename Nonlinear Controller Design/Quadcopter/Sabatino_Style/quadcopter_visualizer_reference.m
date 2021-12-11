%% Plotting script
flag = true; % if we save video
figure
% CoM, motor 1, CoM, motor 2, CoM, motor 3, CoM, motor 4
xvec_body = [0, dist, 0, 0, 0, -dist, 0, 0];
yvec_body = [0, 0, 0, dist, 0, 0, 0, -dist];
zvec_body = [0, 0, 0, 0, 0, 0, 0, 0];

xArr = trajRef;
% Current rotation matrix
R_temp = eval(subs(RotM, [phi theta psi]', xArr(4:6, 1)));
% Transform to current position 
pos_inertial = R_temp * [xvec_body; yvec_body; zvec_body] + xArr(1:3, 1);

ph = plot3(pos_inertial(1, :), pos_inertial(2, :), pos_inertial(3, :));
axis equal
hold on
grid on

if flag
    %writeAnimation(ph, 'fbl_barrel.gif')
    %vidObj = VideoWriter('fbl_barrel','MPEG-4');
    %vidObj = VideoWriter('fbl_helix', 'MPEG-4');
    %vidObj = VideoWriter('lqr_lemniscate', 'MPEG-4');
    %vidObj = VideoWriter('fbl_lemniscate', 'MPEG-4');
    vidObj = VideoWriter('ref_barrel', 'MPEG-4');

    vidObj.FrameRate = 100;
    writeAnimation(vidObj)
end


fh = figure(1);
%plot3(xref, yref, zref, 'k:')
ph2 = plot3(xArr(1, 1), xArr(2, 1), xArr(3, 1), 'b-');
xlim([min(xArr(1, :)) - 1, max(xArr(1, :)) + 1])
ylim([min(xArr(2, :)) - 1, max(xArr(2, :)) + 1])
zlim([min(0, min(xArr(3, :)) - 1), max(xArr(3, :)) + 1])
zlim([2.5 5.5]) % barrel roll
%zlim([0.5, 4.5]) % helix
%zlim([0.5, 11.5])
%zlim([0, 4]) % lemniscate

if flag
    open(vidObj)
end

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
    if flag
        currFrame = getframe(fh);
        writeVideo(vidObj, currFrame);
    end

end

if flag
    close(vidObj)
end