% clc;
% clear;
% close all;


%% ---- Initialise ---- %%
% Protocol version
PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

%%%%%%%% MODIFY PER DEVICE %%%%%%%% 
DXL_ID1                     = 11;
DXL_ID2                     = 12;
DXL_ID3                     = 13;
DXL_ID4                     = 14;
DXL_ID5                     = 15;
BAUDRATE                    = 1000000;
DEVICENAME                  = 'COM15';       % Check which port is being used on your controller
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

Gripper_Open = 250;
Gripper_Slight = 200;
Gripper_Close = 150;

robot = Robot_4DOF(base, shoulder, elbow, wrist, finger,Gripper_Open,Gripper_Slight,Gripper_Close);

%% ---- USER INPUTS ---- %%

stack_grid_coord = [0, 6];
gate_cube_coord = [9, 0];

coords = [
    8, -8;
    5, 5
];

rotation_coord = [5,5];

%% --- INIT ROUTINE ---- %%
Z_lim = 2.15;
%robot.initMovementRoutine([15; 0; 20]);
robot.enableTorque();

%% ---- GRAB CUBE FROM GATES ---- %%

robot.setMaxSpeed(50);
intermediary_pointCM = grid2cm([5.5, 0, 3.5])';
grab_n_points = 250;

gate_cube_coord = [grid2cm(gate_cube_coord), grid2cm(Z_lim - 0.25)]';

outside_gate_coord = [gate_cube_coord(1) - 4, gate_cube_coord(2), grid2cm(Z_lim)]';

robot.open_gripper_slightly();
robot.waitUntilDone();

robot.move_cubic_sync_time([15 ; 0 ; 20], outside_gate_coord,grab_n_points, 0);
robot.waitUntilDone();

robot.move_cubic_sync_time(outside_gate_coord, gate_cube_coord - [0.5;0;0], grab_n_points, 0);
robot.waitUntilDone();

robot.close_gripper();
robot.waitUntilDone();

robot.move_sync(gate_cube_coord + [0;0;0.3], 0);
robot.waitUntilDone();

robot.move_cubic_sync_time(gate_cube_coord + [0;0;0.3], outside_gate_coord, grab_n_points, 0);
robot.waitUntilDone();

robot.move_cubic_sync_time(outside_gate_coord, intermediary_pointCM, grab_n_points/2, 0);
robot.waitUntilDone();

%%
% Robot now has the cube that was below the gates. 
% It will be the first cube placed on the stack

stack_coord_up = grid2cm([stack_grid_coord, 10])'; % will have to change dynamically
stack_coord_down = grid2cm([stack_grid_coord, 3])'; % will also have to change dynamically

robot.move_cubic_sync_time(intermediary_pointCM, stack_coord_up, 90);
robot.waitUntilDone();

robot.move_cubic_sync_time(stack_coord_up, stack_coord_down, 100, 90);
robot.waitUntilDone();

robot.open_gripper();
robot.waitUntilDone();

robot.move_cubic_sync_time(stack_coord_down, stack_coord_up, 20, 90);
robot.waitUntilDone();

%% ---- PLACE OTHER TWO CUBES ---- %%

coord_up1 = grid2cm([coords(1, :), 10])';
coord_down1 = grid2cm([coords(1, :), 3])';

coord_up2 = grid2cm([coords(1, :), 10])';
coord_down2 = grid2cm([coords(1, :), 3])';


robot.setMaxSpeed(100);

robot.move_cubic_sync_time(stack_coord_up, coord_up1, 30, 90);
robot.waitUntilDone();

robot.rotateCubeNTimes(2, rotation_coord, Z_lim);
robot.waitUntilDone();

robot.open_gripper();
robot.waitUntilDone();

% robot.move_cubic_sync_time(coord_up1, coord_down1, 20, 90);
% robot.waitUntilDone();

robot.close_gripper();
robot.waitUntilDone();

robot.move_cubic_sync_time(coord_down1, coord_up1, 20, 90);
robot.waitUntilDone();

% robot now has cube, stacking on top of tower
robot.move_cubic_sync_time(stack_coord_up + grid2cm([0;0;1]), ...
                stack_coord_down + grid2cm([0;0;1]), 100, 90);
robot.waitUntilDone();

robot.open_gripper();
robot.waitUntilDone();

robot.move_cubic_sync_time(stack_coord_down + grid2cm([0;0;1]), ...
                stack_coord_up + grid2cm([0;0;1]), 100, 90);
robot.waitUntilDone();

%% moving onto 2nd free cube
robot.move_cubic_sync_time(stack_coord_up, coord_up2, 20, 0);
robot.waitUntilDone();

robot.rotateCubeNTimes(2, coords(2, :), Z_lim);
robot.waitUntilDone();

robot.open_gripper();
robot.waitUntilDone();

% robot.move_cubic_sync_time(coord_up2, coord_down2, 20, 90);
% robot.waitUntilDone();

robot.close_gripper();
robot.waitUntilDone();

robot.move_cubic_sync_time(coord_down2, coord_up2, 20, 90);
robot.waitUntilDone();

% robot now has cube, stacking on top of tower
robot.move_cubic_sync_time(stack_coord_up + grid2cm([0;0;2]), ...
                stack_coord_down + grid2cm([0;0;2]), 100, 90);
robot.waitUntilDone();

robot.open_gripper();
robot.waitUntilDone();

robot.move_cubic_sync_time(stack_coord_down + grid2cm([0;0;2]), ...
                stack_coord_up + grid2cm([0;0;2]), 100, 90);
robot.waitUntilDone();


pause(5)

%% ---- End ---- %%
robot.disableTorque();

closePort(port_num);
fprintf('Port Closed \n');