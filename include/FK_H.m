function positions = FK_H(angles)

    % Define DH parameters (modify according to your robot)
    L1 = 7.7;   % Base to Joint 2
    L2 = 13;    % Joint 2 to Joint 3
    L3 = 12.6;  % Joint 3 to Joint 4
    L4 = 12.4;  % Joint 4 to End-Effector

    theta1 = angles(1);
    theta2 = angles(2);
    theta3 = angles(3);
    theta4 = angles(4);

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
end