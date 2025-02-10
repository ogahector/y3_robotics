%% OBJECT POSE CLASS: 
% i have no idea if this is the right direction please modify 
classdef ServoDynamixel
    properties (Access = public)
        name {mustBeNonzeroLengthText};
        world_frame {mustBeFloat};
        frame {mustBeFloat};
        % R {mustBeFloat};
        P {mustBeFloat};
        % T {mustBeFloat};
    
        lib_name {mustBeText};
        SERVO_ID {mustBeText};
        PROTOCOL_VERSION {mustBeNumeric};
        port_num {mustBeText};

        ADDR = struct();
    end

    methods
        function obj = ServoDynamixel(name, SERVO_ID, PROTOCOL_VERSION, ...
                                    DEVICENAME, BAUDRATE)

            if ~libisloaded('shrlibsample')
                addpath(fullfile(matlabroot,'extern','examples','shrlib'))
                loadlibrary('shrlibsample')
            end

            [obj.lib_name, ~, ~] = startup_load_libraries();

            obj.port_num = portHandler(DEVICENAME);
            safeOpenPort();

            safeSetBaudrate(obj.port_num, BAUDRATE, obj.lib_name);
            obj.SERVO_ID = SERVO_ID;
            obj.PROTOCOL_VERSION = PROTOCOL_VERSION;

            obj.name = name;

            obj.ADDR.TORQUE_ENABLE       = 64;           
            obj.ADDR.GOAL_POSITION       = 116; 
            obj.ADDR.PRESENT_POSITION    = 132; 
            obj.ADDR.OPERATING_MODE      = 11;
            obj.ADDR.MAX_POS            = 48;
            obj.ADDR.MIN_POS             = 52;
        end

        function obj = setAngleRuleDeg(obj, offset, scale)
            % Set the angle such that its center is now where we want it
        end

%% Dynamixel - related methods
        function obj = disableTorque(obj)
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                obj.SERVO_ID, obj.ADDR.TORQUE_ENABLE, 0);
            verifyTxRxResult(obj.port_num, obj.PROTOCOL_VERSION);
        end

        function obj = enableTorque(obj)
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                obj.SERVO_ID, obj.ADDR.TORQUE_ENABLE, 1);
            verifyTxRxResult(obj.port_num, obj.PROTOCOL_VERSION);
        end

        function obj = setRotationLimitsPulse(obj, min, max)
            obj.disableTorque()

            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                obj.SERVO_ID, obj.ADDR.MIN_POS, min);
            verifyTxRxResult(obj.port_num, obj.PROTOCOL_VERSION);

            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                obj.SERVO_ID, obj.ADDR.MAX_POS, max);
            verifyTxRxResult(obj.port_num, obj.PROTOCOL_VERSION);
        end

        function obj = setRotationLimitsDeg(obj, min, max)
            obj.disableTorque()

            obj.setRotationLimitsPulse(obj, dynDeg2pulse(min), dynDeg2pulse(max));
        end

        function obj = moveToPulse(obj, target)
            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                obj.SERVO_ID, obj.ADDR.GOAL_POSITION, target);
            verifyTxRxResult(obj.port_num, obj.PROTOCOL_VERSION);
        end

        function obj = moveToDeg(obj, target)
            obj.moveToPulse(dynDeg2pulse(target));
        end

        function current_pos = getCurrentPositionPulse(obj)
            current_pos = read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                                obj.SERVO_ID, OBJ.ADDR.PRESENT_POSITION);
            verifyTxRxResult(obj.port_num, obj.PROTOCOL_VERSION);
        end
    end
end


