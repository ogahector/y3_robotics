
L1 = 7.7;
L2 = 13;
L3 = 12.6;
L4 = 12.4;

%% Test Case: Fully Extended (Reachable)
x = 5;
y = 5;
z = 10;


theta = pi/4; % Orientation
P = [x;y;z];
R = [cos(theta), -sin(theta), 0;
     sin(theta),  cos(theta), 0;
     0,           0,          1];

R = makehgtform('yrotate', theta); R = R(1:3, 1:3);
T = [R, P; 0 0 0 1];

T = [0.529576213327687	0.728899125535814	-0.433883739117558	27.1828168511181;
     -0.255030463062816	-0.351019318529051	-0.900968867902419	-13.0905546631920;
     -0.809016994374948	0.587785252292473	6.12323399573677e-17	3.20780678417895;
     0	0	0	1];

angles = IK_H(T, 'up');
T_computed = plot_robot(angles(1), angles(2), angles(3), angles(4), "Ik Attempt");
gcf;
plot3(T(1,4), T(2, 4), T(3, 4), 'xr')

%% Validation
error_position = norm(T(1:3, 4) - T_computed(1:3, 4));
error_orientation = norm(T(1:3, 1:3) - T_computed(1:3, 1:3), 'fro');
fprintf('Position error: %.4f\nOrientation error: %.4f\n', error_position, error_orientation);


%% FUNCTIONS
function T = dh_transform(a, alpha, d, theta)
    T = [
        cos(theta), -sin(theta), 0, a;
        sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d;
        sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d;
        0, 0, 0, 1];
end


function out = plot_robot(t1,t2,t3,t4,name)
    % DH Table
    DH = [
        0, 0, 7.7, t1;
        0, pi/2, 0, t2;
        13, 0, 0, t3;
        12.4, 0, 0, t4;
        12.6, 0, 0, 0;
    ];
    
    
    %Transformations
    T_0_0 = eye(4);
    T_0_1 = T_0_0 * dh_transform(DH(1,1),DH(1,2),DH(1,3),DH(1,4));
    T_0_2 = T_0_1 * dh_transform(DH(2,1),DH(2,2),DH(2,3),DH(2,4));
    T_0_3 = T_0_2 * dh_transform(DH(3,1),DH(3,2),DH(3,3),DH(3,4));
    T_0_4 = T_0_3 * dh_transform(DH(4,1),DH(4, 2),DH(4, 3),DH(4,4));
    T_0_5 = T_0_4 * dh_transform(DH(5,1),DH(5,2),DH(5, 3),DH(5,4));
    
    positions = [T_0_0(1:3,4), T_0_1(1:3,4), T_0_2(1:3,4), T_0_3(1:3,4), T_0_4(1:3,4), T_0_5(1:3,4)];
    
    % figure;
    % title("x-z view")
    % line(positions(1,:),positions(3,:))
    % 
    % figure;
    % title("x-y view")
    % line(positions(1,:),positions(2,:))
    
    figure;
    hold on;
    title(name)
    line(positions(1,:),positions(2,:),positions(3,:))
    view(3)
    grid on
    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')

    out = T_0_5;
end 