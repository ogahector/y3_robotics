clc;
clear;
close all

% Define DH parameters (modify according to your robot)
L1 = 7.7;   % Base to Joint 2
L2 = 13;    % Joint 2 to Joint 3
L3 = 12.6;  % Joint 3 to Joint 4
L4 = 12.4;  % Joint 4 to End-Effector


% Determine cube coordinates
basic_cube = 5 * [...
                0    0    0;
                1  0    0;
                1  1  0;
                0    1  0;
                0    0    1;
                1  0    1;
                1  1  1;
                0    1  1;
              ];
basic_cube = basic_cube + 10;

full_cube = 5 * [...
                0    0    0;
                1  0    0;
                1  1  0;
                0    1  0;
                0    0    0;

                0    0    1;
                1  0    1;
                1  1  1;
                0    1  1;
              ];
full_cube = full_cube + 10;

cube_coords = full_cube;
plot_4dof_robot(0,0,0,0)
gcf;
grid on;
hold on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('4DOF Robot with Coordinate Axes');
xlim([-5 6])
ylim([-6 6])
zlim([0 20])


% Calculate intermediary points
% store all points in a cell
n = 20;
[npoints, ~] = size(cube_coords);
points = [];
for i = 1 : npoints - 1
    current = cube_coords(i, :);
    next = cube_coords(i + 1, :);

    points = [points, cubic_interpol(current, next, n)];
end

for i = 1 : length(points)
    clf
    current_pos = points(:, i);
    end_effector_pose = [eye(3), current_pos; 0 0 0 1];
        
    angles = IK_H(end_effector_pose, 'up');

    plot_4dof_robot(angles(1), angles(2), angles(3), angles(4))
    hold on
    plot3(points(1, 1:i), points(2, 1:i), points(3, 1:i), ...
        'ob', 'LineWidth',1.5)

    axis equal
    view(3)
    drawnow
    % pause(1/n)
end
    