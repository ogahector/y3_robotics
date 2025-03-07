function angles = IK(frame, config)
arguments
    frame double = eye(4)
    config string = ""
end
%% ---- Check Transform Homogeneity ---- %%
if frame(4, :) ~= [0 0 0 1]
    error('The pose provided is a non-homo transform!')
end

%% ---- Link Lengths & End Effector position ---- %%
    % Assignments
    L1 = 7.7;
    L2 = 13;
    L3 = 12.4;
    L4 = 12.6;
    x_e = frame(1,4);
    y_e = frame(2,4);
    z_e = frame(3,4);

%% ---- Base Rotation theta1 ---- %%
    theta1 = atan2(y_e, x_e);

%% ---- Align robot with XZ plane ---- %%
    R = [cos(-theta1) -sin(-theta1) 0; sin(-theta1) cos(-theta1) 0; 0 0 1];
    R_planar = R * frame(1:3,1:3);
    P_planar = R * frame(1:3, 4);

%% ---- End Effector Position in Planar Config ---- %%
    x_e_planar = P_planar(1);
    y_e_planar = P_planar(2);
    z_e_planar = P_planar(3);

    if y_e_planar > 1e-6
        error('Planar alignment failed: Non-zero y-component after transformation.');
    end

%% ---- End Effector Orientation ---- %%
    gamma = atan2(R_planar(3, 1), R_planar(1, 1));

%% ---- Joint 3 Position ---- %%    
    x3 = x_e_planar - L4*cos(gamma); 
    z3 = z_e_planar - L4*sin(gamma) - L1;
    % z3 = z_e_planar - L4*sin(gamma);

%% ---- Check Range Validity ---- %%
    cos_theta3 = (x3^2 + z3^2 - L2^2 - L3^2) / (2 * L2 * L3);
    cos_theta3 = max(min(cos_theta3, 1), -1);
    sin_theta3 = sqrt(1 - cos_theta3^2);
    % alpha = acos(cos_theta3);

%% ---- Theta3 up & down solutions ---- %%
    theta3_up = atan2(sin_theta3, cos_theta3);
    theta3_down = atan2(-sin_theta3, cos_theta3);

%% ---- Theta2 up & down solutions ---- %%
    % theta2_up = atan2(z3, x3) - asin( (L3*sin(alpha))/sqrt(x3^2 + z3^2) );
    % theta2_down = pi/2 - theta2_up;
    theta2 = atan2(z3, x3) - atan2(L3 * sin(theta3_down), L2 + L3 * cos(theta3_down));
    % theta2_down = atan2(z3, x3) + atan2(L3 * sin(theta3_down), L2 + L3 * cos(theta3_down));

%% ---- Theta4 infer from gamma ---- %%
    theta4_up = gamma - theta2 - theta3_up;
    theta4_down = gamma - theta2 - theta3_down;
    theta1 = theta1 + deg2rad(-2);

    if isempty(config)
        angles = [theta1, theta2, theta3_up, theta3_up];
        return
    end

    if lower(config) == "up"
        angles = [theta1, theta2, theta3_up, theta4_up];
    else
        angles = [theta1, theta2, theta3_down, theta4_down];
    end

    angles = [theta1, theta2, theta3_down, theta4_down];
end
