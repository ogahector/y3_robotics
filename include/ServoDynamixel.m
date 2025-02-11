%% SERVO DYNAMIXEL CLASS
% meant to be an interface to the Dynamixel Servos 
% to make code 100x cleaner
classdef ServoDynamixel
    properties (Access = public)
        name {mustBeNonzeroLengthText};
    
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
            obj.ADDR.PRESENT_VELOCITY    = 128;
            obj.ADDR.OPERATING_MODE      = 11;
            obj.ADDR.MAX_POS             = 48;
            obj.ADDR.MIN_POS             = 52;
            obj.ADDR.MAX_VEL             = 44;
            obj.ADDR.HOMING_OFFSET       = 20;
            obj.ADDR.OPERATING_MODE      = 11;
            obj.ADDR.IS_MOVING           = 122;
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
            % without frame correction
            obj.disableTorque()

            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                obj.SERVO_ID, obj.ADDR.MIN_POS, min);
            verifyTxRxResult(obj.port_num, obj.PROTOCOL_VERSION);

            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                obj.SERVO_ID, obj.ADDR.MAX_POS, max);
            verifyTxRxResult(obj.port_num, obj.PROTOCOL_VERSION);
        end

        function obj = setRotationLimitsDeg(obj, min, max)
            % min = obj.angleDegUser2Servo(min);
            % max = obj.angleDegUser2Servo(max);

            obj.disableTorque()

            obj.setRotationLimitsPulse(obj, dynDeg2pulse(min), dynDeg2pulse(max));
        end

        function obj = moveToPulse(obj, target)
            % without frame correction
            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                obj.SERVO_ID, obj.ADDR.GOAL_POSITION, target);
            verifyTxRxResult(obj.port_num, obj.PROTOCOL_VERSION);
        end

        function obj = moveToDeg(obj, target)
            % target = obj.angleDegUser2Servo(target);
            obj.moveToPulse(dynDeg2pulse(target));
        end

        function current_pos = getCurrentPositionPulse(obj)
            current_pos = read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                                obj.SERVO_ID, OBJ.ADDR.PRESENT_POSITION);
            verifyTxRxResult(obj.port_num, obj.PROTOCOL_VERSION);
        end

        function current_ang = getCurrentPositionDeg(obj)
            current_pos = obj.getCurrentPositionPulse();
            current_ang = dynPulse2deg(current_pos);
            % current_ang = obj.angleDegServo2User(current_ang);
        end

        function current_vel = getCurrentVelocity(obj)
            current_vel = read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                          obj.SERVO_ID, obj.ADDR.PRESENT_VELOCITY);
            verifyTxRxResult(obj.port_num, obj.PROTOCOL_VERSION);
        end

        function obj = setMaxSpeed(obj, maxspeedrad)
            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                            obj.SERVO_ID, obj.ADDR.MAX_VEL, maxspeedrad);
            verifyTxRxResult(obj.port_num, obj.PROTOCOL_VERSION);
        end

        function obj = setOffsetPulse(obj, offsetPulse)
            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                        obj.SERVO_ID, obj.ADDR.HOMING_OFFSET, offsetPulse);
            verifyTxRxResult(obj.port_num, obj.PROTOCOL_VERSION);
        end

        function obj = setOffsetDeg(obj, offsetDeg)
            obj.setOffsetPulse(dynDeg2pulse(offsetDeg));
        end

        function obj = setOperatingMode(obj, mode)
            if ~ischar(mode)
                error('Unable to set operating mode, requires string input');
            end
            mode = lower(mode);
            % Default will be position control
            if contains(mode, 'cur')
                mode = 0;
            elseif contains(mode, 'vel')
                mode = 1;
            else 
                mode = 2;
            end
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                            obj.SERVO_ID, obj.ADDR.OPERATING_MODE, mode);
            verifyTxRxResult(obj.port_num, obj.PROTOCOL_VERSION);
        end

        function ismoving = isMoving(obj)
            ismoving = read1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                                    obj.SERVO_ID, obj.ADDR.IS_MOVING);
            ismoving = boolean(ismoving);
        end
    end
end


