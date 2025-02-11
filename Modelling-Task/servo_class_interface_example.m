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
DEVICENAME                  = 'COM7';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

base = ServoDynamixel('Base Rotator', DXL_ID1, PROTOCOL_VERSION, ...
                        DEVICENAME, BAUDRATE, 180, -1);

shoulder = ServoDynamixel('Shoulder Joint', DXL_ID2, PROTOCOL_VERSION, ...
                        DEVICENAME, BAUDRATE, 270-10.62, -1);

elbow = ServoDynamixel('Elbow Joint', DXL_ID3, PROTOCOL_VERSION, ...
                        DEVICENAME, BAUDRATE, 90+10.62, -1);

wrist = ServoDynamixel('Wrist Joint', DXL_ID4, PROTOCOL_VERSION, ...
                        DEVICENAME, BAUDRATE, 180, -1);

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

angles = zeros(4, length(points));
% assuming orientation 0
for i = 1 : length(points)
    T = [eye(3), points(:, i); 0 0 0 1];

    angles(:, i) = IK_H(T);
    % angles(:, i) = angle_to_servo(angles(:, i));
end

angles = rad2deg(angles);

%% ---- Configure ---- %%
% Disable torque <=> enable configuration
base.disableTorque()
shoulder.disableTorque()
elbow.disableTorque()
wrist.disableTorque()

% Assign max and minimums
base.setRotationLimitsDeg(90, -90);
shoulder.setRotationLimitsDeg(0, 360);
elbow.setRotationLimitsDeg(0, 360);
wrist.setRotationLimitsDeg(0, 360);

%% ---- Move ---- %%

base.enableTorque()
shoulder.enableTorque()
elbow.enableTorque()
wrist.enableTorque()
pause(1)

for i = 1 : length(angles)
    base.moveToDeg(angles(1, i));
    shoulder.moveToDeg(angles(2, i));
    elbow.moveToDeg(angles(3, i));
    wrist.moveToDeg(angles(4, i));

    pause(0.5) % not meant to move for very long: short time should be fine
end

%% ---- End ---- %%
base.disableTorque()
shoulder.disableTorque()
elbow.disableTorque()
wrist.disableTorque()

closePort(port_num);
fprintf('Port Closed \n');