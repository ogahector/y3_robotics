clc; clear; 
close all;

[lib_name, ~, ~] = startup_load_libraries();

%% ---- Control Table Addresses ---- %%

ADDR_TORQUE_ENABLE       = 64;           % Control table address is different in Dynamixel model
ADDR_GOAL_POSITION       = 116; 
ADDR_PRESENT_POSITION    = 132; 
ADDR_OPERATING_MODE      = 11;
ADDR_MAX_POS = 48;
ADDR_MIN_POS = 52;

MIN_POS = dynPulse2deg(90);
MAX_POS = dynPulse2deg(270);

%% ---- Other Settings ---- %%

% Protocol version
PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting
%%%%%%%% MODIFY PER DEVICE %%%%%%%% 
DXL_ID1                     = 13;           % Dynamixel ID: 1
DXL_ID2                     = 11;
BAUDRATE                    = 57600;
DEVICENAME                  = 'COM4';       % Check which port is being used on your controller
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

% maximums and minimums
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_MIN_POS, MIN_POS);
verifyTxRxResult(port_num, PROTOCOL_VERSION);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_MAX_POS, MAX_POS);
verifyTxRxResult(port_num, PROTOCOL_VERSION);

%% ---- Pre-Processing ---- %%

t = 1 : 360;
vals = sind(t);

wave_amplitude = 0.5*(MAX_POS - MIN_POS);
wave_offset = MIN_POS;
positions = wave_amplitude*vals + wave_offset;
positions = typecast(int32(positions), 'uint32');

samples = zeros(1, t(end));

%% ---- Servo CTRL ---- %%

write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_TORQUE_ENABLE, 1);

for i = t
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_GOAL_POSITION, positions(i));
    verifyTxRxResult(port_num, PROTOCOL_VERSION);

    samples(i) = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRESENT_POSITION);
    verifyTxRxResult(port_num, PROTOCOL_VERSION);
end


%% ---- END ---- %%

write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_TORQUE_ENABLE, 0);
verifyTxRxResult(port_num, PROTOCOL_VERSION);

