% clc;
% clear;
% close all;

% This is the version that was used during the demo

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
Gripper_Slight = 160;
Gripper_Close = 195;

robot = Robot_4DOF(base, shoulder, elbow, wrist, finger,Gripper_Open,Gripper_Slight,Gripper_Close);

%% ---- USER INPUTS ---- %%

stack_grid_coord = [0, -6];
gate_cube_coord = [8, 3];

coords = [
    5, 5;
    0, 8
];

rotation_coord = [5,-5];

gate_angle = 0;


%% ---- INIT ROUTINE ---- %%

%robot.initMovementRoutine([15; 0; 20]);
robot.enableTorque();

%% ---- PARAMETERS ---- %%%

n_points = 60;

z_lim_v = 2.25;
z_lim_h = z_lim_v - 0.25;

offset_h2v = 1.1; %Offset for vertical grabs
rotate_offset_h2v = 0.6; %Offset for vertical grabs in rotation (different for some reason)



coord_up1 = grid2cm([coords(1, :), z_lim_h*3])';
coord_down1 = grid2cm([coords(1, :), z_lim_h])';

coord_up2 = grid2cm([coords(2, :), z_lim_h*3])';
coord_down2 = grid2cm([coords(2, :), z_lim_h])';

rotation_coord_up = grid2cm([rotation_coord,z_lim_h*3])';
% rotation_coord_down = grid2cm(getH2Vcoord([rotation_coord, z_lim_v],offset_h2v))';
rotation_coord_down = grid2cm([rotation_coord, z_lim_h])';

rotate_v = getH2Vcoord( [rotation_coord, z_lim_v ], rotate_offset_h2v);
% rotation_coord_down = grid2cm(rotate_v)';
rotate_h = [rotation_coord , z_lim_h];

gate_cube_coord = [grid2cm(gate_cube_coord), grid2cm(z_lim_h)]';
outside_gate_coord = [gate_cube_coord(1) - 4, gate_cube_coord(2), grid2cm(z_lim_h)]';

stack_coord_up = grid2cm([stack_grid_coord, 10])'; % will have to change dynamically
stack_coord_down = grid2cm([stack_grid_coord, z_lim_v])'; % will also have to change dynamically


%% ---- GRAB CUBE FROM GATES ---- %%
intermediary_pointCM = grid2cm([5.5, 3, 3.5])';


robot.setMaxSpeed(80);

robot.open_gripper_slightly();
robot.waitUntilDone();

%Outside gate
robot.move_cubic_sync([15 ; 0 ; 20], outside_gate_coord,n_points, gate_angle);
robot.waitUntilDone();

%Inside gate
robot.move_cubic_sync(outside_gate_coord, gate_cube_coord - [0.5;0;0], n_points, gate_angle);
robot.waitUntilDone();

robot.close_gripper();
robot.waitUntilDone();

%Move up a bit
robot.move_cubic_sync(gate_cube_coord - [0.5;0;0], gate_cube_coord + [0;0;0.5], n_points,gate_angle);
robot.waitUntilDone();

%Back outside gate
robot.move_cubic_sync(gate_cube_coord + [0;0;0.3], outside_gate_coord, n_points, gate_angle);
robot.waitUntilDone();

%Midpoint
robot.move_cubic_sync(outside_gate_coord, intermediary_pointCM, n_points/2, gate_angle);
robot.waitUntilDone();

%%
% Robot now has the cube that was below the gates. 
% It will be the first cube placed on the stack

%Move higher 
robot.move_cubic_sync_time(intermediary_pointCM,intermediary_pointCM+grid2cm([0;0;4]),100,gate_angle);
robot.waitUntilDone();

%Move above stack
robot.move_cubic_sync(intermediary_pointCM, stack_coord_up,n_points*2, gate_angle);
robot.waitUntilDone();

%Drop onto stack (with vertical offset, since cube was picked up
%horizontally)
temp = getH2Vcoord(stack_coord_down, offset_h2v);
robot.move_cubic_sync_time(stack_coord_up, temp, n_points*2, 90);
robot.waitUntilDone();

robot.open_gripper();
robot.waitUntilDone();

%Move back up above stack
robot.move_cubic_sync_time(temp, stack_coord_up, n_points, 90);
robot.waitUntilDone();

robot.move_cubic_sync_time(stack_coord_up, n_points, 0);
robot.waitUntilDone();

%% ---- PLACE AND ROTATE OTHER TWO CUBES ---- %%

robot.setMaxSpeed(100);
robot.wrist.setMaxSpeed(200);

%% ---- CUBE 2 ---- %%
robot.move_cubic_sync_time(stack_coord_up, coord_up1, n_points, 0);
robot.waitUntilDone();

robot.open_gripper();
robot.waitUntilDone();

robot.move_cubic_sync_time(coord_up1,coord_down1,n_points,0);
robot.waitUntilDone();

robot.close_gripper();
robot.waitUntilDone();

robot.move_cubic_sync(coord_down1,coord_up1,n_points,0);
robot.waitUntilDone();

% robot.move_cubic_sync_time(coord_up1,n_points,90);
% robot.waitUntilDone();

robot.move_cubic_sync_time(coord_up1,rotation_coord_up,n_points, 0);
robot.waitUntilDone();

robot.move_cubic_sync_time(rotation_coord_down,n_points, 0);
robot.waitUntilDone();

robot.open_gripper();
robot.waitUntilDone();

robot.move_cubic_sync(rotation_coord_down, rotation_coord_up,n_points, 0);
robot.waitUntilDone();

% robot.move_cubic_sync_time(rotation_coord_down, n_points, 90);
% robot.waitUntilDone();

robot.rotateCubeNTimes(1, rotate_v,rotate_h);
robot.waitUntilDone();

robot.open_gripper();
robot.waitUntilDone();

robot.close_gripper();
robot.waitUntilDone();

robot.move_cubic_sync_time(rotation_coord_down, stack_coord_up, n_points, 90);
robot.waitUntilDone();

robot.move_cubic_sync(stack_coord_up,stack_coord_down+ [0;0;2.5],n_points,90);
robot.waitUntilDone();

robot.open_gripper();
robot.waitUntilDone();

%% ---- Cube 3 ---- %%
robot.move_cubic_sync_time(stack_coord_up, coord_up2, n_points, 0);
robot.waitUntilDone();

robot.open_gripper();
robot.waitUntilDone();

robot.move_cubic_sync_time(coord_up2,coord_down2,n_points,0);
robot.waitUntilDone();

robot.close_gripper();
robot.waitUntilDone();

robot.move_cubic_sync(coord_down2,coord_up2,n_points,0);
robot.waitUntilDone();

% robot.move_cubic_sync_time(coord_up2,n_points,90);
% robot.waitUntilDone();

robot.move_cubic_sync_time(coord_up2,rotation_coord_up,n_points,0);
robot.waitUntilDone();

robot.move_cubic_sync_time(rotation_coord_down,n_points, 0);
robot.waitUntilDone();

robot.open_gripper();
robot.waitUntilDone();

robot.move_cubic_sync(rotation_coord_down, rotation_coord_up,n_points, 0);
robot.waitUntilDone();

% robot.move_cubic_sync_time(rotation_coord_down, n_points, 90);
% robot.waitUntilDone();

robot.rotateCubeNTimes(2, rotate_v,rotate_h);
robot.waitUntilDone();

robot.open_gripper();
robot.waitUntilDone();

robot.close_gripper();
robot.waitUntilDone();

robot.move_cubic_sync_time(rotation_coord_down, stack_coord_up, n_points, 90);
robot.waitUntilDone();

robot.move_cubic_sync(stack_coord_up,stack_coord_down+ [0;0;5],n_points,90);
robot.waitUntilDone();

robot.open_gripper();
robot.waitUntilDone();

pause(5)

%% ---- End ---- %%
robot.disableTorque();
closePort(port_num);
fprintf('Port Closed \n');

%%
function vertical_coord = getH2Vcoord(coord,vertical_offset)
    vertical_coord = coord;
    theta = atand(coord(2)/coord(1));
    vertical_x_offset = vertical_offset * cosd(theta);
    vertical_y_offset = vertical_offset * sind(theta);
    vertical_coord(1) = coord(1) + vertical_x_offset;
    vertical_coord(2) = coord(2) + vertical_y_offset;
end