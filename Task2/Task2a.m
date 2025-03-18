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
                        port_num, 180 -1.75, 1);

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

Z_lim = 2.4;

% finger.setOperatingMode('pos');
% finger.setGoalCurrent(80);%80 ma in practice, check doc
robot = Robot_4DOF(base, shoulder, elbow, wrist, finger, Gripper_Open, Gripper_Slight, Gripper_Close);


%% ---- Process Points to Visit ---- %%


grid_start1 = [0, 8];
grid_end1 = [0, -6];

grid_start2 = [5, 5];
grid_end2 = [5, -5];

grid_start3 = [8, 3];
grid_end3 = [0, 4];

coords = [
    grid2cm([grid_start1, 6]);
    grid2cm([grid_start1, Z_lim]);
    grid2cm([grid_end1, 6]);
    grid2cm([grid_end1, Z_lim]);
    
    grid2cm([grid_start2, 6]);
    grid2cm([grid_start2, Z_lim]);
    grid2cm([grid_end2, 6]);
    grid2cm([grid_end2, Z_lim]);

    grid2cm([grid_start3, 9]);
    grid2cm([grid_start3, Z_lim]);
    grid2cm([grid_end3, 9]);
    grid2cm([grid_end3, Z_lim]);
];




% angles = rad2deg(angles);

%% ---- Configure ---- %%
% Disable torque <=> enable configuration
robot.disableTorque();

%% ---- Move ---- %%
robot.setMaxSpeed(80);
robot.enableTorque();

%% ---- MOVE USING CUBIC ---- %%
angle_in = 90;
n_points = 20;
% Init
robot.move_cubic_sync_time(POINT, n_points, 0);
robot.waitUntilDone();


for i = 0 : 2
    ind = 4*i;


    robot.move_cubic_sync_time(coords(ind + 1, :)', n_points, angle_in);
    robot.waitUntilDone();
    robot.open_gripper();
    robot.waitUntilDone();

    robot.move_cubic_sync_time(coords(ind+1, :)', coords(ind+2, :)', n_points, angle_in);
    robot.waitUntilDone();
    robot.close_gripper();
    robot.waitUntilDone();

    robot.move_cubic_sync_time(coords(ind+2, :)', coords(ind+1, :)', n_points, angle_in);
    robot.waitUntilDone();
    

    robot.move_cubic_sync_time(coords(ind+1, :)', coords(ind+3, :)', n_points, angle_in);
    robot.waitUntilDone();

    robot.move_cubic_sync_time(coords(ind+3, :)', coords(ind+4, :)', n_points, angle_in);
    robot.waitUntilDone();

    robot.open_gripper();
    robot.waitUntilDone();

    robot.close_gripper();
    robot.waitUntilDone();

    robot.open_gripper();
    robot.waitUntilDone();

    robot.move_cubic_sync_time(coords(ind+4, :)', coords(ind+3, :)', n_points, angle_in);
    robot.waitUntilDone();
end

robot.move_cubic_sync_time(POINT, n_points, 0);
pause(2)

%% ---- End ---- %%
robot.disableTorque();

closePort(port_num);
fprintf('Port Closed \n');