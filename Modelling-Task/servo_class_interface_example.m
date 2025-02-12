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
DEVICENAME                  = 'COM10';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


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

%% ---- Process Points to Visit ---- %%
cube_coords = 5 * [...
                0    0    0;
                1  0    0;
                1  1  0;
                0    1  0;
                0    0    0;

                0    0    1;
                1  0    1;
                1  1  1;
                0    1  1;
              ];
cube_coords = cube_coords + 10;

n = 20;
[npoints, ~] = size(cube_coords);
points = [];
for i = 1 : npoints - 1
    current = cube_coords(i, :);
    next = cube_coords(i + 1, :);

    points = [points, cubic_interpol(current, next, n)];
end

% angles = zeros(4, length(points));
angles = [];
% assuming orientation 0
for i = 1 : length(points)
    T = [eye(3), points(:, i); 0 0 0 1];

    angles(i, :) = IK_H(T);
    % angles(:, i) = angle_to_servo(angles(:, i));
end

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

base.enableTorque()
shoulder.enableTorque()
elbow.enableTorque()
wrist.enableTorque()
pause(1)

base.moveToDeg(0)
shoulder.moveToDeg(90)
elbow.moveToDeg(-90)
wrist.moveToDeg(0)

% figure
% for i = 1 : length(angles)
%     clf
%     plot_4dof_robot(angles(i, 1), angles(i, 2), angles(i, 3), angles(i, 4));
%     drawnow
% end

pause(1)
angles = rad2deg(angles);
for i = 1 : length(angles)
    base.moveToDeg(angles(i, 1));
    shoulder.moveToDeg(angles(i, 2));
    elbow.moveToDeg(angles(i, 3));
    wrist.moveToDeg(angles(i, 4));

    % pause(0.5) % not meant to move for very long: short time should be fine
end



pause(10)

%% ---- End ---- %%
base.disableTorque()
shoulder.disableTorque()
elbow.disableTorque()
wrist.disableTorque()

closePort(port_num);
fprintf('Port Closed \n');