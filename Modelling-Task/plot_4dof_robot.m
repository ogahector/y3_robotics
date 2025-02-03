function plot_4dof_robot(theta1, theta2, theta3, theta4)
    % Define DH parameters (modify according to your robot)
    L1 = 7.7;   % Base to Joint 2
    L2 = 13;    % Joint 2 to Joint 3
    L3 = 12.6;  % Joint 3 to Joint 4
    L4 = 12.4;  % Joint 4 to End-Effector

    % DH table: [a, alpha, d, theta]
    DH = [
        0, 0, L1, theta1;
        0, pi/2, 0, theta2;%last should maybe be theta2?
        L2, 0, 0, theta3;
        L3, 0, 0, theta4;
        L4, 0, 0, 0;
    ];

    [n_dh, ~] = size(DH);

    % Compute transformations and joint positions
    T = cell(1, n_dh+1);
    T{1} = eye(4);  % Base frame
    for i = 1:n_dh
        a = DH(i, 1);
        alpha = DH(i, 2);
        d = DH(i, 3);
        theta = DH(i, 4);
        
        Ti = dh_transform(a, alpha, d, theta);
        T{i+1} = T{i} * Ti;
    end

    % Extract joint positions
    positions = zeros(3, n_dh+1);
    for i = 1:n_dh+1
        positions(:, i) = T{i}(1:3, 4);
    end

    % Plot links
    % figure;
    gcf;
    hold on;
    for i = 1:n_dh
        line(positions(1, i:i+1), positions(2, i:i+1), positions(3, i:i+1), ...
            'LineWidth', 3, 'Color', 'k');
    end

    % Plot coordinate axes at each joint
    axis_length = 3;  % Length of axes for visualization
    colors = ['r', 'g', 'b'];  % RGB for XYZ axes
    for i = 1:n_dh
        R = T{i}(1:3, 1:3);  % Rotation matrix
        pos = T{i}(1:3, 4);  % Position
        
        % X-axis (red)
        x_end = pos + R(:, 1) * axis_length;
        line([pos(1), x_end(1)], [pos(2), x_end(2)], [pos(3), x_end(3)], ...
            'Color', colors(1), 'LineWidth', 2);
        
        % Y-axis (green)
        y_end = pos + R(:, 2) * axis_length;
        line([pos(1), y_end(1)], [pos(2), y_end(2)], [pos(3), y_end(3)], ...
            'Color', colors(2), 'LineWidth', 2);
        
        % Z-axis (blue)
        z_end = pos + R(:, 3) * axis_length;
        line([pos(1), z_end(1)], [pos(2), z_end(2)], [pos(3), z_end(3)], ...
            'Color', colors(3), 'LineWidth', 2);
    end

    % Annotate joints
    % joint_labels = {'Base', 'Joint 1', 'Joint 2', 'Joint 3', 'End-Effector'};
    % for i = 1:5
    %     text(positions(1, i), positions(2, i), positions(3, i), joint_labels{i}, ...
    %         'FontSize', 10, 'HorizontalAlignment', 'right');
    % end

    % Format plot
    grid on;
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('4DOF Robot with Coordinate Axes');
    view(3);
    hold off;
end