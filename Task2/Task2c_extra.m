clc;
clear;
close all;

% this is the version that was used during the demo

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
                        port_num, 180 - 1, 1);

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

Gripper_Open = 110;
Gripper_Slight = 155;
Gripper_Close = 200;

robot = Robot_4DOF(base, shoulder, elbow, wrist, finger,Gripper_Open,Gripper_Slight,Gripper_Close);

%% --- GATE COORDS --- %%
z_lim = 2.5 - 1;%Artifical 0
gate_offset = 10; %Approx height of gates, needs to be higher for taller two (2 and 3)
hammer_offset = 14; %Offset for the top of the hammer

hammer_coord = [-2 , -8];

coords = [%Adjust these to get desired results
    grid2cm([-2, -7]) , z_lim + gate_offset;%1 - Next to hammer
    grid2cm([4.9, -7]) , z_lim + gate_offset + 1;%2 - Before gate 1
    grid2cm([4.9, -5]) , z_lim + gate_offset + 1;%3 - After gate 1
    grid2cm([6, -1]) , z_lim + gate_offset - 1;%4 - Before gate 2
    grid2cm([4, -1]) , z_lim + gate_offset - 1;%5 - After gate 2
    grid2cm([4.9, 1]) , z_lim + gate_offset - 1;%6 - Before gate 3
    grid2cm([4.9, 3]) , z_lim + gate_offset - 1;%7 - After gate 3
    grid2cm([8 , 0]) , z_lim + gate_offset + 1; %Before gate 4
    grid2cm([10, 0]) , z_lim + gate_offset + 1%8 - After gate 4
    ];

%% --- CONFIGURE --- %%
robot.disableTorque();
robot.setMaxSpeed(40);
robot.enableTorque();

%% --- INIT ROUTINE ---- %%
%robot.initMovementRoutine([15; 0; 15]);

%% --- GRAB HAMMER --- %%
hammer_grab_angle = 45; %Angle at which the robot will grab the hammer
n_points = 26;


hammer_coord = [grid2cm(hammer_coord), z_lim + hammer_offset]';


% robot.move_sync([-3.4039, -11.4904, 15.702]',45);
% robot.waitUntilDone();


%robot.move_cubic_sync([15; 0; 15],[15; 0; z_lim + hammer_offset],n_points,90);

robot.move_cubic_sync_time([15; 0; z_lim + hammer_offset], 20, hammer_grab_angle);
robot.waitUntilDone();
robot.move_cubic_sync([15; 0; z_lim + hammer_offset],hammer_coord,n_points*2,hammer_grab_angle);
robot.waitUntilDone();

robot.open_gripper();
robot.waitUntilDone();

robot.move_cubic_sync_time((hammer_coord - [0 ; 0 ; 8.2]), n_points, hammer_grab_angle);
robot.waitUntilDone();

robot.close_gripper();
robot.waitUntilDone();

robot.setMaxSpeed(40);

robot.move_cubic_sync(hammer_coord,n_points*3,hammer_grab_angle);
robot.waitUntilDone();

robot.setMaxSpeed(120)

%% --- MOVE THROUGH GATES --- %%

robot.move_cubic_sync_time(coords(1,:)', n_points, hammer_grab_angle);%Next to hammer
robot.waitUntilDone();

intermediary23 = [3, 1];
intermediary23 = [grid2cm(intermediary23), z_lim + gate_offset - 1 ];

intermediary34 = [7, 3];
intermediary34 = [grid2cm(intermediary34), z_lim + gate_offset - 1];

robot.move_cubic_sync(coords(1,:)',coords(2,:)',n_points/2,hammer_grab_angle);%Before gate 1 - less x
robot.waitUntilDone();
% pause(1)
robot.move_cubic_sync(coords(2,:)',coords(3,:)',n_points,hammer_grab_angle);%Through gate 1 
robot.waitUntilDone();
% pause(1)
robot.move_cubic_sync(coords(3,:)',coords(4,:)',n_points/2,hammer_grab_angle);%Before gate 2
robot.waitUntilDone();
% pause(1)
robot.move_cubic_sync(coords(4,:)',coords(5,:)',n_points,hammer_grab_angle);%Through gate 2 (Too high and too far back)
robot.waitUntilDone();

robot.move_cubic_sync(coords(5,:)', intermediary23, n_points/2, hammer_grab_angle);
robot.waitUntilDone();

robot.move_cubic_sync(intermediary23,coords(6,:)',n_points/2,hammer_grab_angle);%Before gate 3 
robot.waitUntilDone();

robot.move_cubic_sync(coords(6,:)',coords(7,:)',n_points,hammer_grab_angle);%Through gate 3 (too far back, little lower)
robot.waitUntilDone();

robot.move_cubic_sync(coords(7,:)', intermediary34, n_points/2, hammer_grab_angle);
robot.waitUntilDone();

robot.move_cubic_sync(intermediary34,coords(8,:)',n_points/2,hammer_grab_angle);%Before gate 4
robot.waitUntilDone();
robot.move_cubic_sync(coords(8,:)',coords(9,:)',n_points,hammer_grab_angle);%Through gate 4
robot.waitUntilDone();

robot.move_cubic_sync(coords(9,:)',coords(8,:)',n_points,hammer_grab_angle);
robot.waitUntilDone();
robot.move_cubic_sync(coords(8,:)',intermediary34,n_points/2,hammer_grab_angle);
robot.waitUntilDone();

robot.move_cubic_sync(intermediary34,coords(7,:)',n_points/2,hammer_grab_angle);
robot.waitUntilDone();

robot.move_cubic_sync(coords(7,:)',coords(6,:)',n_points,hammer_grab_angle);
robot.waitUntilDone();

robot.move_cubic_sync(coords(6,:)',intermediary23,n_points/2,hammer_grab_angle);
robot.waitUntilDone();

robot.move_cubic_sync(intermediary23,coords(5,:)',n_points/2,hammer_grab_angle);
robot.waitUntilDone();

robot.move_cubic_sync(coords(5,:)',coords(4,:)',n_points,hammer_grab_angle);
robot.waitUntilDone();

robot.move_cubic_sync(coords(4,:)',coords(3,:)',n_points,hammer_grab_angle);
robot.waitUntilDone();

robot.move_cubic_sync(coords(3,:)',coords(2,:)',n_points,hammer_grab_angle);
robot.waitUntilDone();

robot.move_cubic_sync(coords(2,:)',coords(1,:)',n_points,hammer_grab_angle);
robot.waitUntilDone();

robot.move_cubic_sync(coords(1,:)',hammer_coord - grid2cm([0;0;2]),n_points,hammer_grab_angle);
robot.waitUntilDone();

robot.open_gripper();
robot.waitUntilDone();


%% --- END --- %%
pause(5);
%%
robot.disableTorque();
closePort(port_num);
fprintf('Port Closed \n');
