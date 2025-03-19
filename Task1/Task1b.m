%% TASK1B - SIMULATION: INCREASE EACH ANGLE INDEPENDENTLY
clc;
clear;
close all

% [x, y, ~] = cylinder(15);
% z = 10*ones(size(x));
% points = [];
% n = 20;
% for i = 1 : length(x)-1
%     current = [x(i); y(i); z(i)];
%     next = [x(i+1); y(i+1); z(i+1)];
% 
%     points = [points, linear_interpol(current, next, n)];
% end
% 
% angles = [];
% for i = 1 : length(points)
%     T = makehgtform('translate', points(:, i));
%     angles = [angles; IK(T)];
% end
npoints = 30;

base_angle = linspace(0, 2*pi, npoints*4);
shoulder_angle = linspace(0, pi/2, npoints);
elbow_angle = linspace(0, pi/2, npoints);
wrist_angle = linspace(0, pi/2, npoints);

pause(5)

figure;
for i = 1 : length(base_angle)
    clf
    plot_4dof_robot(base_angle(i), 0, 0, 0)
    drawnow
end
pause(1)

for i = 1 : length(shoulder_angle)
    clf
    plot_4dof_robot(2*pi, shoulder_angle(i), 0, 0)
    drawnow
end
pause(1)

for i = 1 : length(elbow_angle)
    clf
    plot_4dof_robot(2*pi, pi/2, elbow_angle(i), 0)
    drawnow
end
pause(1)

for i = 1 : length(wrist_angle)
    clf
    plot_4dof_robot(2*pi, pi/2, pi/2, wrist_angle(i))
    drawnow
end
pause(1)