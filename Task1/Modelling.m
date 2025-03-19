clc; clear all; close all;
%pi/2 = 1.5708
%pi/4 = 0.7854
%pi/8 = 0.3927


FK = plot_robot(-pi/7,pi/5,-pi/4,-pi/4,"Forward") %What it should look like
x = FK(1,4);
y = FK(2,4);
z = FK(3,4);
g = 0; %angle the end effector should be pointing (see note below)
A = ik_2(FK)

%NOTE:
% Can likely determine the orientation of end effector from it's frame (the
% output of the plot robot function gives the end effector frame). The only
% thing I'm thinking could be an issue is determening which axis this should
% be taken in reference too x or y? What we could alternatively do, is to
% pass the whole frame into the IK function, then when we rotate it to
% remove the effect of theta1, we know that gamma will be the rotation
% around y, and can determine it from the rotated matrix that way. Needs to
% be tested to see if that works. 
% (ik_2 tries, and sort of fails to do this, need more work)
%
% For now tho, if you wanna ensure you don't have to fw the gamma variable,
% just make sure theta2 theta3 and theta4 all add to 0. 

%Parameters
theta1 = A(1);
theta2 = A(2);
theta3 = A(3);
theta4 = A(4);


IK = plot_robot(theta1,theta2,theta3,theta4,"Inverse"); %What the IK looks like

disp(FK)
disp(IK)

function T = dh_transform(a, alpha, d, theta)
    T = [
        cos(theta), -sin(theta), 0, a;
        sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d;
        sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d;
        0, 0, 0, 1];
end

function angles = ik( x, y, z, gamma)
    L1 = 7.7;
    L2 = 13;
    L3 = 12.6;
    L4 = 12.4;
    a1 = atan2(y,x) ;
    P = [cos(-a1) -sin(-a1) 0 ; sin(-a1) cos(-a1) 0 ; 0 0 1] * [x ; y ; z]; % Rotate by -a1
    P3 = [P(1) ; P(3)] - [L4 * cos(gamma); L4 * sin(gamma) + L1];
    
    c2 = (P3(1)^2 + P3(2)^2 - L2^2 - L3^2) / (2 * L2 * L3);
    c2 = max(min(c2, 1), -1);
    s2 = -sqrt(1 - c2^2); % Negative for elbow down

    a2 = atan2(P3(2), P3(1)) - atan2((L3 * s2), (L2 + L3 * c2));
    a3 = atan2(s2, c2);
    a4 = gamma - a2 - a3;
    
    angles = [a1, a2, a3, a4];
end

function angles = ik_2(frame)
    % Assignments
    L1 = 7.7;
    L2 = 13;
    L3 = 12.6;
    L4 = 12.4;
    x = frame(1,4);
    y = frame(2,4);
    z = frame(3,4);

    % Rotate by theta1 to get onto x-z plane
    a1 = atan2(y, x);
    R = [cos(-a1) -sin(-a1) 0; sin(-a1) cos(-a1) 0; 0 0 1];
    P = R * frame(1:3, 4); % Rotate the position vector by -a1

    % Extract the rotation matrix part of the frame
    R_frame = frame(1:3, 1:3);

    % Rotate the rotation matrix by -a1
    R_rotated = R * R_frame;

    % Find orientation from the rotated rotation matrix
    gamma = atan2(R_rotated(3, 1), R_rotated(1, 1));

    P3 = [P(1); P(3)] - [L4 * cos(gamma); L4 * sin(gamma) + L1];
    
    c2 = (P3(1)^2 + P3(2)^2 - L2^2 - L3^2) / (2 * L2 * L3);
    c2 = max(min(c2, 1), -1);
    s2 = -sqrt(1 - c2^2); % Negative for elbow down

    a2 = atan2(P3(2), P3(1)) - atan2((L3 * s2), (L2 + L3 * c2));
    a3 = atan2(s2, c2);
    a4 = gamma - a2 - a3;
    
    angles = [a1, a2, a3, a4];
end

function out = plot_robot(t1,t2,t3,t4,name)
    % DH Table
    DH = [
        0, 0, 7.7, t1;
        0, pi/2, 0, t2;%last should maybe be theta2?
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
    axis equal
    grid on
    xlabel('x')
    ylabel('y')
    zlabel('z')

    out = T_0_5;
end 