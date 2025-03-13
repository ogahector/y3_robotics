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
DEVICENAME                  = 'COM11';       % Check which port is being used on your controller
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

POINT = [15 ; 0 ; 25];

[lib_name, ~, ~] = startup_load_libraries();

port_num = portHandler(DEVICENAME);
packetHandler();
safeOpenPort(port_num, lib_name);
safeSetBaudrate(port_num, BAUDRATE, lib_name);

base = ServoDynamixel("Base Rotator", DXL_ID1, PROTOCOL_VERSION, ...
                        port_num, 180, 1);

shoulder = ServoDynamixel("Shoulder Joint", DXL_ID2, PROTOCOL_VERSION, ...
                        port_num, +270 - 10.62, -1);

elbow = ServoDynamixel("Elbow Joint", DXL_ID3, PROTOCOL_VERSION, ...
                        port_num, +90 + 10.62, -1);

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

Gripper_Open = 110;
Gripper_Slight = 130;
Gripper_Close = 200;

z_lim = 2.6;


robot = Robot_4DOF(base, shoulder, elbow, wrist, finger, Gripper_Open, Gripper_Slight, Gripper_Close);

%% ---- Variables ---- %%
n_points = 50;
pour_angle = 75;
pour_offset = grid2cm([-0.5; -0.5; -1.5]); %This will make the beaker move back and down a bit while it pours
n_stirs = 4; %How many times we stir

%% ---- Process Points to Visit ---- %%

%Beaker 1 (6,-6)
beaker_1_coord_down = grid2cm([6 , -6, z_lim + 2])';%Extra height
beaker_1_coord_up = beaker_1_coord_down + grid2cm([0 ; 0 ; 7]);

%Beaker 2 (5,5)
beaker_2_coord_down = grid2cm([5,5,z_lim])';
beaker_2_coord_up = beaker_2_coord_down + grid2cm([0 ; 0 ; 7]);

pour_coord = beaker_2_coord_up + pour_offset;

%Stir (0,-8)
stir_coord_down = grid2cm([0 ; -8 ; z_lim]);%Extra height
stir_coord_up = stir_coord_down + grid2cm([0 ; 0 ; 7]);

circle_radius = 3.5;
circle_n = 100; % Number of points
circle_coord_down = grid2cm([6 ; 6; z_lim + 2]);%Extra height
circle_coord_up = circle_coord_down + grid2cm([0 ; 0; 5]);
circle_points = circle(circle_coord_down,circle_radius,circle_n);


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
robot.move_cubic_sync(beaker_1_coord_up,beaker_1_coord_down,n_points,0);
robot.waitUntilDone();

pause(2)%Here so I can place 'beaker', not necessary in reality

%Grab it (wide bottle so slight here)
robot.open_gripper_slightly();
robot.waitUntilDone();

%Pick it back up
robot.move_cubic_sync(beaker_1_coord_down,beaker_1_coord_up,n_points,0);
robot.waitUntilDone();

%% ---- Move and Pour ---- %%
%Move above beaker 2
robot.move_cubic_sync(beaker_1_coord_up,beaker_2_coord_up,n_points,0);
robot.waitUntilDone();

%Pour slowly (*2 points)
robot.move_cubic_sync_time(pour_coord,n_points*2,pour_angle);
robot.waitUntilDone();

%Stop pouring slowly (*2 points)
robot.move_cubic_sync_time(beaker_2_coord_up,n_points*2,0);
robot.waitUntilDone();

%% ---- Replace beaker 1 ---- %%
%Back above beaker 1
robot.move_cubic_sync(beaker_2_coord_up,beaker_1_coord_up,n_points,0);
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

robot.move_cubic_sync(stir_coord_down,stir_coord_up,n_points,0);
robot.waitUntilDone();

%% ---- Move and Stir ---- %%
robot.move_cubic_sync_time(stir_coord_up,circle_coord_up,n_points,0);
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