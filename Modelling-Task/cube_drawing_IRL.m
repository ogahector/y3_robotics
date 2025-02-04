clc;
clear;
close all;

[lib_name, ~, ~] = startup_load_libraries();

%% ---- Control Table Addresses ---- %%

ADDR_TORQUE_ENABLE       = 64;           % Control table address is different in Dynamixel model
ADDR_GOAL_POSITION       = 116; 
ADDR_PRESENT_POSITION    = 132; 
ADDR_OPERATING_MODE      = 11;
ADDR_MAX_POS = 48;
ADDR_MIN_POS = 52;

MIN_POS1 = dynDeg2pulse(90);
MAX_POS1 = dynDeg2pulse(270);

MIN_POS2 = dynDeg2pulse(290);
MAX_POS2 = dynDeg2pulse(165);

MIN_POS3 = dynDeg2pulse(265);
MAX_POS3 = dynDeg2pulse(60);

MIN_POS4 = dynDeg2pulse(300);
MAX_POS4 = dynDeg2pulse(78);

MIN_POS5 = dynDeg2pulse(180);
MAX_POS5 = dynDeg2pulse(170);

%% ---- Other Settings ---- %%

% Protocol version
PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting
%%%%%%%% MODIFY PER DEVICE %%%%%%%% 
DXL_ID1                     = 11;           % Dynamixel ID: 1
DXL_ID2                     = 12;
DXL_ID3                     = 13;
DXL_ID4                     = 14;
DXL_ID5                     = 15;
BAUDRATE                    = 1000000;
DEVICENAME                  = 'COM7';       % Check which port is being used on your controller
                                            % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                                            
TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0;      % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4095;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

%% ---- Connect to Servos ---- %%

port_num = portHandler(DEVICENAME);
packetHandler();
safeOpenPort(port_num, lib_name);
safeSetBaudrate(port_num, BAUDRATE, lib_name);


%% ---- Configure Servo ---- %%
% disable torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_TORQUE_ENABLE, 0);
verifyTxRxResult(port_num, PROTOCOL_VERSION);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_TORQUE_ENABLE, 0);
verifyTxRxResult(port_num, PROTOCOL_VERSION);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_TORQUE_ENABLE, 0);
verifyTxRxResult(port_num, PROTOCOL_VERSION);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_TORQUE_ENABLE, 0);
verifyTxRxResult(port_num, PROTOCOL_VERSION);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_TORQUE_ENABLE, 0);
verifyTxRxResult(port_num, PROTOCOL_VERSION);

% maximums and minimums
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_MIN_POS, MIN_POS1);
% verifyTxRxResult(port_num, PROTOCOL_VERSION);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_MAX_POS, MAX_POS1);
% verifyTxRxResult(port_num, PROTOCOL_VERSION);
% 
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_MIN_POS, uint32(-15000));
% verifyTxRxResult(port_num, PROTOCOL_VERSION);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_MAX_POS, uint32(15000));
% verifyTxRxResult(port_num, PROTOCOL_VERSION);
% 
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_MIN_POS, MIN_POS3);
% verifyTxRxResult(port_num, PROTOCOL_VERSION);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_MAX_POS, MAX_POS3);
% verifyTxRxResult(port_num, PROTOCOL_VERSION);
% 
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_MIN_POS, MIN_POS4);
% verifyTxRxResult(port_num, PROTOCOL_VERSION);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_MAX_POS, MAX_POS4);
% verifyTxRxResult(port_num, PROTOCOL_VERSION);
% 
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_MIN_POS, MIN_POS5);
% verifyTxRxResult(port_num, PROTOCOL_VERSION);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_MAX_POS, MAX_POS5);
% verifyTxRxResult(port_num, PROTOCOL_VERSION);

%% ---- configure ---- %%
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

% offsets = [ dynDeg2pulse(+180);
%             dynDeg2pulse();
%             dynDeg2pulse();
%             dynDeg2pulse();
%         ];

angles = [];
for i = 1 : npoints
    T = [eye(3), points(:, i); 0 0 0 1];
    angles(1:4, i) = IK_H(T, 'up')';
    % angles(1:4, i) = 
    angles(1, i) = dynRad2pulse(angles(1, i));
    angles(2, i) = dynRad2pulse(angles(1, i));
    angles(3, i) = dynRad2pulse(angles(1, i));
    angles(4, i) = dynRad2pulse(angles(1, i));

    angles(1, i) = - angles(1, i) + dynDeg2pulse(180);
    angles(1, i) = - angles(1, i) + dynDeg2pulse(270 - 10.62);
    angles(1, i) = - angles(1, i) + dynDeg2pulse(90 + 10.62);
    angles(1, i) = - angles(1, i) + dynDeg2pulse(180);

end

% enable torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_TORQUE_ENABLE, 1);
verifyTxRxResult(port_num, PROTOCOL_VERSION);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_TORQUE_ENABLE, 1);
verifyTxRxResult(port_num, PROTOCOL_VERSION);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_TORQUE_ENABLE, 1);
verifyTxRxResult(port_num, PROTOCOL_VERSION);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_TORQUE_ENABLE, 1);
verifyTxRxResult(port_num, PROTOCOL_VERSION);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_TORQUE_ENABLE, 1);
verifyTxRxResult(port_num, PROTOCOL_VERSION);

samples = zeros(4, 1);

% for i = 1 : length(angles)
%     %% WRITE 
%     write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_GOAL_POSITION, angles(1, i));
%     verifyTxRxResult(port_num, PROTOCOL_VERSION);
% 
%     write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_GOAL_POSITION, angles(2, i));
%     verifyTxRxResult(port_num, PROTOCOL_VERSION);
% 
%     write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_GOAL_POSITION, angles(3, i));
%     verifyTxRxResult(port_num, PROTOCOL_VERSION);
% 
%     write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_GOAL_POSITION, angles(4, i));
%     verifyTxRxResult(port_num, PROTOCOL_VERSION);
% 
%     % write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_GOAL_POSITION, angles(5, i));
%     % verifyTxRxResult(port_num, PROTOCOL_VERSION);
% 
%     %% READ
%     samples(1, i) = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRESENT_POSITION);
%     verifyTxRxResult(port_num, PROTOCOL_VERSION);
% 
%     samples(2, i) = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRESENT_POSITION);
%     verifyTxRxResult(port_num, PROTOCOL_VERSION);
% 
%     samples(3, i) = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRESENT_POSITION);
%     verifyTxRxResult(port_num, PROTOCOL_VERSION);
% 
%     samples(4, i) = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRESENT_POSITION);
%     verifyTxRxResult(port_num, PROTOCOL_VERSION);
% 
%     pause(0.5)
% end

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_GOAL_POSITION, dynDeg2pulse(180));
verifyTxRxResult(port_num, PROTOCOL_VERSION);

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_GOAL_POSITION, dynDeg2pulse(200));
verifyTxRxResult(port_num, PROTOCOL_VERSION);

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_GOAL_POSITION, dynDeg2pulse(165));
verifyTxRxResult(port_num, PROTOCOL_VERSION);

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_GOAL_POSITION, dynDeg2pulse(180));
verifyTxRxResult(port_num, PROTOCOL_VERSION);

% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_GOAL_POSITION, angles(5, i));
% verifyTxRxResult(port_num, PROTOCOL_VERSION);
%%

pause(5)
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_TORQUE_ENABLE, 0);
verifyTxRxResult(port_num, PROTOCOL_VERSION);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_TORQUE_ENABLE, 0);
verifyTxRxResult(port_num, PROTOCOL_VERSION);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_TORQUE_ENABLE, 0);
verifyTxRxResult(port_num, PROTOCOL_VERSION);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_TORQUE_ENABLE, 0);
verifyTxRxResult(port_num, PROTOCOL_VERSION);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_TORQUE_ENABLE, 0);
verifyTxRxResult(port_num, PROTOCOL_VERSION);


write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_TORQUE_ENABLE, 0);
verifyTxRxResult(port_num, PROTOCOL_VERSION);

figure;
plot_4dof_robot(0,0,0,0)
gcf;
grid on;
hold on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('4DOF Robot with Coordinate Axes');
xlim([-5 6])
ylim([-6 6])
zlim([0 20])
for i = 1 : length(samples)
    clf

    plot_4dof_robot(samples(1, i), samples(2, i), samples(3, i), samples(4, i))
    hold on
    % plot3(points(1, 1:i), points(2, 1:i), points(3, 1:i), ...
    %     'ob', 'LineWidth',1.5)

    axis equal
    view(3)
    drawnow
    % pause(1/n)
end


