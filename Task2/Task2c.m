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

Gripper_Open = 85;
Gripper_Slight = 130;
Gripper_Close = 200;

robot = Robot_4DOF(base, shoulder, elbow, wrist, finger,Gripper_Open,Gripper_Slight,Gripper_Close);

%% --- GATE COORDS --- %%
hammer_coord = [-2 , -8];%Check these

coords = [
    -2, -7;
    4, -7;
    4, -3.5;
    9, -3.5;
    9, 0;
    7, 0;
    7, 4;
    1, 4
    ];

% gate_1_before_coord = [5, -7];
% gate_1_coord = [5, -5];
% gate_1_after_coord = [5,-3.5];
% 
% gate_2_before_coord = [10,-3.5];
% gate_2_coord = [10, -2];
% gate_2_after_coord = [10,0];
% 
% gate_3_before_coord = [8,0];
% gate_3_coord = [8, 2];
% gate_3_after_coord = [8,4];
% 
% gate_4_coord = [4, 4];
% gate_4_after_coord = [2,4];

%% --- CONFIGURE --- %%
robot.disableTorque();
robot.setMaxSpeed(40);
robot.enableTorque();

%% --- INIT ROUTINE ---- %%
robot.initMovementRoutine([15; 0; 15]);

%% --- GRAB HAMMER --- %%
z_lim = 2.5;
gate_offset = 10;
hammer_offset = 14;
n_points = 50;

hammer_coord = [grid2cm(hammer_coord), grid2cm(z_lim) + hammer_offset]';

%robot.move_cubic_sync([15; 0; 15],[15; 0; z_lim + hammer_offset],n_points,90);
robot.move_sync([15; 0; z_lim + hammer_offset],90)
robot.waitUntilDone();
robot.move_cubic_sync([15; 0; z_lim + hammer_offset],hammer_coord,n_points,90);
robot.waitUntilDone();

robot.open_gripper();

robot.move_sync((hammer_coord - [0 ; 0 ; 7]),90);
robot.waitUntilDone();

robot.close_gripper();

robot.move_sync(hammer_coord,90);
robot.waitUntilDone();

%% --- MOVE THROUGH GATES --- %%

robot.move_sync([grid2cm(coords(1,:)) , z_lim + gate_offset]',90);
for i = 1:(size(coords,1)-1)
    start_coord = [grid2cm(coords(i,:)) , z_lim + gate_offset]';
    end_coord = [grid2cm(coords(i+1,:)) , z_lim + gate_offset]';
    robot.move_cubic(start_coord,end_coord,n_points,90);
    robot.waitUntilDone();
end

%% --- END --- %%
pause(10);
robot.disableTorque();
closePort(port_num);
fprintf('Port Closed \n');
