clc;
clear;
close all

f = @(x) dynDeg2pulse(-x + 180);

[x, y, ~] = cylinder(15);
z = 10*ones(size(x));
points = [];
n = 20;
for i = 1 : length(x)-1
    current = [x(i); y(i); z(i)];
    next = [x(i+1); y(i+1); z(i+1)];

    points = [points, linear_interpol(current, next, n)];
end

angles = [];
for i = 1 : length(points)
    T = makehgtform('translate', points(:, i));
    angles = [angles; IK(T)];
end

figure;
for i = 1 : length(angles)
    clf
    plot_4dof_robot(angles(i, 1), angles(i, 2), angles(i, 3), angles(i, 4))
    hold on
    plot3(points(1, 1:i), points(2, 1:i), points(3, 1:i), '-ob')
    drawnow
end