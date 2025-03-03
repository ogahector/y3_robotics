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

robot = Robot_4DOF(base, shoulder, elbow, wrist, finger);

%% ---- USER INPUTS ---- %%

stack_grid_coord = [0, 6];
gate_cube_coord = [9, 0];

coords = [
    8, -8;
    5, 5
];

%% --- INIT ROUTINE ---- %%
robot.initMovementRoutine([15; 0; 20]);

%% ---- GRAB CUBE FROM GATES ---- %%

robot.setMaxSpeed(20);

gate_cube_coord = grid2cm([gate_cube_coord, 3]);

outside_gate_coord = [gate_cube_coord(1) - 3, gate_cube_coord(2)];

robot.move(outside_gate_coord, 0);
robot.waitUntilDone();

robot.open_gripper_slightly();
robot.waitUntilDone();

robot.move_cubic(outside_gate_coord, gate_cube_coord, 100, 0);
robot.waitUntilDone();

robot.close_gripper();
robot.waitUntilDone();

robot.move_cubic(gate_cube_coord, outside_gate_coord, 100, 0);
robot.waitUntilDone();

% Robot now has the cube that was below the gates. 
% It will be the first cube placed on the stack

stack_coord_up = grid2cm([stack_grid_coord, 6])'; % will have to change dynamically
stack_coord_down = grid2cm([stack_grid_coord, 3])'; % will also have to change dynamically

robot.move(stack_coord_up, 90);
robot.waitUntilDone();

robot.move_cubic(stack_coord_up, stack_coord_down, 100, 90);
robot.waitUntilDone();

robot.open_gripper();
robot.waitUntilDone();

robot.move_cubic(stack_coord_down, stack_coord_up, 20, 90);
robot.waitUntilDone();

%% ---- PLACE OTHER TWO CUBES ---- %%

coord_up1 = grid2cm([coords(1, :), 6])';
coord_down1 = grid2cm([coords(1, :), 3])';

coord_up2 = grid2cm([coords(1, :), 6])';
coord_down2 = grid2cm([coords(1, :), 3])';


robot.setMaxSpeed(50);

robot.move_cubic(stack_coord_up, coord_up1, 20, 0);
robot.waitUntilDone();

robot.rotateCubeNTimes(2, coords(1,:));
robot.waitUntilDone();

robot.open_gripper();
robot.waitUntilDone();

% robot.move_cubic(coord_up1, coord_down1, 20, 90);
% robot.waitUntilDone();

robot.close_gripper();
robot.waitUntilDone();

robot.move_cubic(coord_down1, coord_up1, 20, 90);
robot.waitUntilDone();

% robot now has cube, stacking on top of tower
robot.move_cubic(stack_coord_up + grid2cm([0;0;1]), ...
                stack_coord_down + grid2cm([0;0;1]), 100, 90);
robot.waitUntilDone();

robot.open_gripper();
robot.waitUntilDone();

robot.move_cubic(stack_coord_down + grid2cm([0;0;1]), ...
                stack_coord_up + grid2cm([0;0;1]), 100, 90);
robot.waitUntilDone();

%% moving onto 2nd free cube
robot.move_cubic(stack_coord_up, coord_up2, 20, 0);
robot.waitUntilDone();

robot.rotateCubeNTimes(2, coords(2, :));
robot.waitUntilDone();

robot.open_gripper();
robot.waitUntilDone();

% robot.move_cubic(coord_up2, coord_down2, 20, 90);
% robot.waitUntilDone();

robot.close_gripper();
robot.waitUntilDone();

robot.move_cubic(coord_down2, coord_up2, 20, 90);
robot.waitUntilDone();

% robot now has cube, stacking on top of tower
robot.move_cubic(stack_coord_up + grid2cm([0;0;2]), ...
                stack_coord_down + grid2cm([0;0;2]), 100, 90);
robot.waitUntilDone();

robot.open_gripper();
robot.waitUntilDone();

robot.move_cubic(stack_coord_down + grid2cm([0;0;2]), ...
                stack_coord_up + grid2cm([0;0;2]), 100, 90);
robot.waitUntilDone();


pause(5)

%% ---- End ---- %%
robot.disableTorque();

closePort(port_num);
fprintf('Port Closed \n');