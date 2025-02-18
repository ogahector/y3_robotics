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
DXL_ID5                     = 15;
BAUDRATE                    = 1000000;
DEVICENAME                  = '/dev/ttyUSB0';       % Check which port is being used on your controller
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
                        port_num, 180, -1);

finger.setOperatingMode('curpos');
finger.setGoalCurrent(30);%80 ma in practice, check doc
robot = Robot_4DOF(base, shoulder, elbow, wrist, finger);


%% ---- Process Points to Visit ---- %%
cube_coords = [...
                10    0    10;
              ];
%%cube_coords = cube_coords + 10;





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
pause(5)

robot.open_gripper();
pause(4)
robot.close_gripper();

% figure
% for i = 1 : length(angles)
%     clf
%     plot_4dof_robot(angles(i, 1), angles(i, 2), angles(i, 3), angles(i, 4));
%     drawnow
% end


% robot.move(POINT);

% n = 20;
% [npoints, ~] = size(cube_coords);
% points = [];
% for i = 1 : npoints - 1
%     current = cube_coords(i, :);
%     next = cube_coords(i + 1, :);
%     robot.move_cubic(current, next, n);
%     pause(0.5);
% end



pause(10)

%% ---- End ---- %%
base.disableTorque()
shoulder.disableTorque()
elbow.disableTorque()
wrist.disableTorque()

closePort(port_num);
fprintf('Port Closed \n');