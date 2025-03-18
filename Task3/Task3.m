clc;
clear;
close all;


%% ---- Initialise ---- %%
% Protocol version
PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

%%%%%%%% MODIFY PER DEVICE %%%%%%%% 
DXL_ID1                     = 11;
DXL_ID2                     = 12;
DXL_ID3                     = 13;
DXL_ID4                     = 14;
DXL_ID5                      = 15;
BAUDRATE                    = 1000000;
DEVICENAME                  = 'COM14';       % Check which port is being used on your controller
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

POINT = [15 ; 0 ; 25];

[lib_name, ~, ~] = startup_load_libraries();

port_num = portHandler(DEVICENAME);
packetHandler();
safeOpenPort(port_num, lib_name);
safeSetBaudrate(port_num, BAUDRATE, lib_name);

base = ServoDynamixel("Base Rotator", DXL_ID1, PROTOCOL_VERSION, ...
                        port_num, 180 - 1.5, 1);

shoulder = ServoDynamixel("Shoulder Joint", DXL_ID2, PROTOCOL_VERSION, ...
                        port_num, +270 - 12.4, -1);

elbow = ServoDynamixel("Elbow Joint", DXL_ID3, PROTOCOL_VERSION, ...
                        port_num, +90 + 12.4, -1);

wrist = ServoDynamixel("Wrist Joint", DXL_ID4, PROTOCOL_VERSION, ...
                        port_num, 180, -1);

finger = ServoDynamixel("Finger Joint", DXL_ID5, PROTOCOL_VERSION, ...
                        port_num, 0, 1);

mov_threshold = 1;
base.setMovingThreshold(mov_threshold);
shoulder.setMovingThreshold(mov_threshold);
elbow.setMovingThreshold(mov_threshold);
wrist.setMovingThreshold(mov_threshold);
finger.setMovingThreshold(mov_threshold);

Gripper_Open = 85;
Gripper_Slight = 166;
Gripper_Close = 198;

z_lim = 1.5;


robot = Robot_4DOF(base, shoulder, elbow, wrist, finger, Gripper_Open, Gripper_Slight, Gripper_Close);

%% ---- Variables ---- %%
beaker_grab_grid_coords = [6, -6];
desired_pouring_coords = [5, 5];
tool_grab_grid_coords = [0, -8];

n_points = 50;
n_pour_points = 5;
pour_angle = 90;
pour_offset_mag = 2;
theta_temp = atan2(desired_pouring_coords(2), desired_pouring_coords(1));
pour_offset = grid2cm([-pour_offset_mag*cos(theta_temp); -pour_offset_mag*sin(theta_temp); -2]); %This will make the beaker move back and down a bit while it pours
n_stirs = 4; %How many times we stir

r_beaker = 2e-2;
L_beaker = 8e-2;
V_initial = 80e-3;
h_initial = V_initial / (pi*r_beaker^2);

% wall line definition
linept1 = [2, 0];
linept2 = [9, 0];
wallheight = 12;
walltol = 1;

%% ---- Process Points to Visit ---- %%

%Beaker 1 (6,-6)
beaker_1_coord_down = grid2cm([6 , -6, z_lim])';%Extra height
beaker_1_coord_up = beaker_1_coord_down + grid2cm([0 ; 0 ; 6]);

%Beaker 2 (5,5)
beaker_2_coord_down = grid2cm([5,5,z_lim])';
beaker_2_coord_up = beaker_2_coord_down + grid2cm([0 ; 0 ; 7]);

pour_coord = beaker_2_coord_up + pour_offset;

%Stir (0,-8)
stir_coord_down = grid2cm([0 ; -8 ; z_lim]);%Extra height
stir_coord_up = stir_coord_down + grid2cm([0 ; 0 ; 8]);

circle_radius = 1.5;
circle_n = 100; % Number of points
circle_coord_down = grid2cm([6 ; 6; z_lim + 5]);%Extra height
circle_coord_up = circle_coord_down + grid2cm([0 ; 0; 2.5]);
circle_points = circle(circle_coord_down,circle_radius,circle_n);

% wall intermediary point
wall_coord_up = grid2cm([mean([linept1(1), linept2(1)]), ...
                         mean([linept1(2), linept2(2)]), ...
                         wallheight + 1
    ])';


%% ---- Configure ---- %%
% Disable torque <=> enable configuration
robot.disableTorque();
robot.setMaxSpeed(120);
robot.enableTorque();

%% ---- Grab Beaker 1 ---- %%

robot.open_gripper();
robot.waitUntilDone();%Should move these calls inside the functions to verify internally

%Move above beaker 1
robot.move_cubic_sync_time(beaker_1_coord_up,n_points,0);
robot.waitUntilDone();

%Go down 
robot.move_cubic_sync(beaker_1_coord_up,beaker_1_coord_down,2*n_points,0);
robot.waitUntilDone();

pause(0.5)%Here so I can place 'beaker', not necessary in reality

%Grab it (wide bottle so slight here)
robot.open_gripper_slightly();
robot.waitUntilDone();

%Pick it back up
robot.move_cubic_sync(beaker_1_coord_down,beaker_1_coord_up,n_points,0);
robot.waitUntilDone();

%% ---- Move and Pour ---- %%
%Move above beaker 2
[beaker2pourBefore, beaker2pourAfter] = getIntermediaryWallPoints(wall_coord_up, walltol, beaker_1_coord_up, beaker_2_coord_up);

robot.move_cubic_sync(beaker_1_coord_up, beaker2pourBefore, n_points, 0);
robot.waitUntilDone();

robot.move_cubic_sync(beaker2pourBefore, beaker2pourAfter, n_points, 0);
robot.waitUntilDone();

robot.move_cubic_sync(beaker2pourAfter,beaker_2_coord_up,n_points,0);
robot.waitUntilDone();

% robot.move_cubic_sync_time(pour_coord, n_points, 0);
% robot.waitUntilDone();

%Pour slowly (*2 points)
robot.move_cubic_sync_time(pour_coord,n_points*4,pour_angle);
robot.waitUntilDone();
pause(1)

%Stop pouring slowly (*2 points)
robot.move_cubic_sync_time(beaker_2_coord_up,n_points*2,0);
robot.waitUntilDone();

% current_angle = 0;
% while ~donePouring(r_beaker, h_initial, L_beaker, current_angle, 30e-3)
%     current_angle = current_angle + 1;
%     robot.move_cubic_sync_time(beaker_2_coord_up, n_pour_points, current_angle);
%     robot.waitUntilDone();
% end

%% ---- Replace beaker 1 ---- %%
%Back above beaker 1
[pour2beakerBefore, pour2beakerAfter] = getIntermediaryWallPoints(wall_coord_up, walltol, beaker_2_coord_up, beaker_1_coord_up);

robot.move_cubic_sync(beaker_2_coord_up, pour2beakerBefore, n_points, 0);
robot.waitUntilDone();


robot.move_cubic_sync(pour2beakerBefore, pour2beakerAfter, n_points, 0);
robot.waitUntilDone();

robot.move_cubic_sync(pour2beakerAfter,beaker_1_coord_up,n_points,0);
robot.waitUntilDone();

%Put it down
robot.move_cubic_sync(beaker_1_coord_up,beaker_1_coord_down,n_points,0);
robot.waitUntilDone();

robot.open_gripper();
robot.waitUntilDone();

%Move back above
robot.move_cubic_sync(beaker_1_coord_down,beaker_1_coord_up,n_points,0);
robot.waitUntilDone();

%% ---- Grab Stirrer ---- %%
robot.move_cubic_sync_time(stir_coord_up,n_points,0);
robot.waitUntilDone();

robot.move_cubic_sync(stir_coord_up,stir_coord_down,n_points,0);
robot.waitUntilDone();

robot.close_gripper(); %Check the gripper closing here
robot.waitUntilDone();

robot.move_cubic_sync(stir_coord_down,stir_coord_up,3*n_points,0);
robot.waitUntilDone();

%% ---- Move and Stir ---- %%
[stir2circleBefore, stir2circleAfter] = getIntermediaryWallPoints(wall_coord_up, walltol, stir_coord_up, circle_coord_up);

robot.move_cubic_sync(stir_coord_up, stir2circleBefore, n_points, 0);
robot.waitUntilDone();

robot.move_cubic_sync(stir2circleBefore, stir2circleAfter, n_points, 0);
robot.waitUntilDone();

robot.move_cubic_sync_time(stir2circleAfter,circle_coord_up,n_points,0);
robot.waitUntilDone();

robot.move_cubic_sync(circle_coord_up,circle_coord_down,n_points,0);
robot.waitUntilDone();

robot.move_cubic_sync(circle_coord_down,circle_points(:,1),n_points,0);

circle_interp_n = 25;
for j = 1:n_stirs
    for i = 1:(size(circle_points,2) -1 )
        % robot.move_cubic_sync_time(circle_points(:,i),circle_interp_n,0);
        % robot.waitUntilDone();
        robot.move_sync(circle_points(:,i),0);
        pause(0.01);
    end
end

robot.move_cubic_sync_time(circle_coord_up,n_points,0);
robot.waitUntilDone();

%% ---- Put Stirrer Back ---- %%
robot.move_cubic_sync(circle_coord_up, stir2circleAfter, n_points, 0);
robot.waitUntilDone();

robot.move_cubic_sync(stir2circleAfter, stir2circleBefore, n_points, 0);
robot.waitUntilDone();

robot.move_cubic_sync(stir2circleBefore, stir_coord_up, n_points, 0);
robot.waitUntilDone();

robot.open_gripper();
robot.waitUntilDone();

pause(5);
%% ---- End ---- %%
robot.disableTorque();
closePort(port_num);
fprintf('Port Closed \n');

%%

function points = circle(center,radius,n)
    x = center(1);
    y = center(2);
    th = 0:2*pi/n:2*pi;
    xunit = radius * cos(th) + x;
    yunit = radius * sin(th) + y;
    points = [[xunit ; yunit] ; repelem(center(3),n+1)];
end

function [ptBefore, ptAfter] = getIntermediaryWallPoints(wallmidpoint, walltol, pt1, pt2)
    dir_vector = pt2(1:2) - pt1(1:2);
    dir_vector = dir_vector / (norm(dir_vector));

    ptBefore = [wallmidpoint(1:2);0] + [dir_vector * -grid2cm(walltol); wallmidpoint(3)];
    ptAfter = [wallmidpoint(1:2);0] + [dir_vector * grid2cm(walltol); wallmidpoint(3)];

end