clc; clear all; close all;

plot_robot(0,pi/4,-pi/8,-pi/8)%%What it should look like

%Variables - Open
%Should give t1 = 0, t2 = pi/4, t3 = -pi/8, t4 = -pi/8
%pi/4 = 0.7854
%pi/8 = 0.3927
%Orientation should be 0 for this
x = 33.2485;
y = 0;
z = 21.6377;
gamma = 0; % end effector orientation (need to see how this should change for theta1
A = ik_2(7.7,13,12.4,12.6,x,y,z,0);

%Parameters
theta1 = A(1);
theta2 = A(2);
theta3 = A(3);
theta4 = A(4);

plot_robot(theta1,theta2,theta3,theta4)%What the IK looks like


function T = dh_transform(a, alpha, d, theta)
    T = [
        cos(theta), -sin(theta), 0, a;
        sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d;
        sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d;
        0, 0, 0, 1];
end

function angles = ik_2(L1, L2, L3, L4, x, y, z, gamma)
    a1 = atan2(y, x);
    P = [cos(a1) -sin(a1); sin(a1) cos(a1)] * [x; z]; % Rotated x and z
    P3 = P - [L4 * cos(gamma); L4 * sin(gamma) + L1];
    
    c2 = (P3(1)^2 + P3(2)^2 - L2^2 - L3^2) / (2 * L2 * L3);
    c2 = max(min(c2, 1), -1);
    s2 = -sqrt(1 - c2^2); % Negative for elbow down

    a2 = atan2(P3(2), P3(1)) - atan2((L3 * s2), (L2 + L3 * c2));
    a3 = atan2(s2, c2);
    a4 = gamma - a2 - a3;
    
    angles = [a1, a2, a3, a4];
end

function plot_robot(t1,t2,t3,t4)
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
    title("3D view")
    line(positions(1,:),positions(2,:),positions(3,:))
    view(3)
    axis equal
end 