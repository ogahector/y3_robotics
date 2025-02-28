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

% finger.setOperatingMode('pos');
% finger.setGoalCurrent(80);%80 ma in practice, check doc
robot = Robot_4DOF(base, shoulder, elbow, wrist, finger);


%% ---- Process Points to Visit ---- %%
% cube_coords = 5 * [...
%                 0    0    0;
%                 1  0    0;
%                 1  1  0;
%                 0    1  0;
%                 0    0    0;
% 
%                 0    0    1;
%                 1  0    1;
%                 1  1  1;
%                 0    1  1;
%               ];
% cube_coords = cube_coords + 10;

cube_len = 2.5;

coords = [
    grid2cm([9, 0, 6]);
    grid2cm([9, 0, 3]);
    grid2cm([5, 5, 6]);
    grid2cm([5, 5, 3]);
];




% angles = rad2deg(angles);

%% ---- Configure ---- %%
% Disable torque <=> enable configuration
base.disableTorque()
shoulder.disableTorque()
elbow.disableTorque()
wrist.disableTorque()

base.setMaxSpeed(30)
shoulder.setMaxSpeed(30)
elbow.setMaxSpeed(30)
wrist.setMaxSpeed(30)

% Assign homing offset
% base.setOffsetDeg(180);

% Assign max and minimums
% base.setRotationLimitsDeg(90, -90);
% shoulder.setRotationLimitsDeg(0, 360);
% elbow.setRotationLimitsDeg(0, 360);
% wrist.setRotationLimitsDeg(0, 360);

%% ---- Move ---- %%

base.enableTorque();
shoulder.enableTorque();
elbow.enableTorque();
wrist.enableTorque();
finger.enableTorque();
pause(1)

base.moveToDeg(-90)
shoulder.moveToDeg(90)
elbow.moveToDeg(-90)
wrist.moveToDeg(0)
robot.waitUntilDone();
robot.open_gripper();
robot.waitUntilDone();
robot.close_gripper();
robot.waitUntilDone();
% figure
% for i = 1 : length(angles)
%     clf
%     plot_4dof_robot(angles(i, 1), angles(i, 2), angles(i, 3), angles(i, 4));
%     drawnow
% end


robot.move(coords(1, :)', 90);
robot.waitUntilDone();
robot.open_gripper();
robot.waitUntilDone();
robot.move(coords(2, :)', 90);
robot.waitUntilDone();
robot.close_gripper();
robot.waitUntilDone();
robot.move(coords(1, :)', 90);
robot.waitUntilDone();

robot.move(coords(3, :)', 90);
robot.waitUntilDone();
robot.move(coords(4, :)', 90);
robot.waitUntilDone();
robot.open_gripper();
robot.waitUntilDone();
robot.move(coords(3, :)', 90);
robot.waitUntilDone();


% n = 20;
% [npoints, ~] = size(coords);
% points = [];
% for i = 1 : npoints - 1
%     current = coords(i, :);
%     next = coords(i + 1, :);
%     robot.move_cubic(current, next, n, 0);
%     pause(0.5);
% end



pause(5)

%% ---- End ---- %%
base.disableTorque()
shoulder.disableTorque()
elbow.disableTorque()
wrist.disableTorque()
finger.disableTorque()

closePort(port_num);
fprintf('Port Closed \n');