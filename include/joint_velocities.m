%Need to check this works correctly
function joint_velocities = joint_velocities(theta, end_effector_velocity)
    syms t1 t2 t3 t4

    % DH Table
    DH = [
        0, 0, 7.7, t1;
        0, pi/2, 0, t2;
        13, 0, 0, t3;
        12.4, 0, 0, t4;
        12.6, 0, 0, 0;
    ];

    % Transformations
    T_0_0 = eye(4);
    T_0_1 = T_0_0 * dh_transform(DH(1,1), DH(1,2), DH(1,3), DH(1,4));
    T_0_2 = T_0_1 * dh_transform(DH(2,1), DH(2,2), DH(2,3), DH(2,4));
    T_0_3 = T_0_2 * dh_transform(DH(3,1), DH(3,2), DH(3,3), DH(3,4));
    T_0_4 = T_0_3 * dh_transform(DH(4,1), DH(4,2), DH(4,3), DH(4,4));
    T_0_5 = T_0_4 * dh_transform(DH(5,1), DH(5,2), DH(5,3), DH(5,4));

    % Position of the end effector
    P_E = T_0_5(1:3, 4);

    %Also, may want to preprocess J for faster computing
    J = [jacobian(P_E, [t1, t2, t3, t4]); 0 0 0 1];
    J_num = double(subs(J, [t1, t2, t3, t4], theta));
    J_inv = inv(J_num);

    joint_velocities = J_inv * end_effector_velocity;
end