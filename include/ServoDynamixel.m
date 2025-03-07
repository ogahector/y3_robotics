%% SERVO DYNAMIXEL CLASS
% meant to be an interface to the Dynamixel Servos 
% to make code 100x cleaner
classdef ServoDynamixel
    properties (Access = public)
        % name {mustBeNonzeroLengthText};
        name;
    
        lib_name;
        SERVO_ID;
        PROTOCOL_VERSION
        port_num

        ADDR = struct();

        userAngle2Servo = @(userAngle) userAngle;
        servoAngle2User = @(servoAngle) servoAngle;

    end

    methods
        function obj = ServoDynamixel(name, SERVO_ID, PROTOCOL_VERSION, ...
                                    port_num, offset, scale)

            if ~libisloaded('shrlibsample')
                addpath(fullfile(matlabroot,'extern','examples','shrlib'))
                loadlibrary('shrlibsample')
            end

            if isempty(scale) || scale == 0
                scale = 1;
            end
            obj.userAngle2Servo = @(userAngle) scale*userAngle + offset;
            obj.servoAngle2User = @(servoAngle) (servoAngle - offset) / scale;

            % [obj.lib_name, ~, ~] = startup_load_libraries();
            % 
            % obj.port_num = portHandler(DEVICENAME);
            % safeOpenPort(obj.port_num, obj.lib_name);
            % 
            % safeSetBaudrate(obj.port_num, BAUDRATE, obj.lib_name);
            obj.port_num = port_num;
            obj.SERVO_ID = SERVO_ID;
            obj.PROTOCOL_VERSION = PROTOCOL_VERSION;

            obj.name = name;

            obj.ADDR.TORQUE_ENABLE       = 64;           
            obj.ADDR.GOAL_POSITION       = 116; 
            obj.ADDR.GOAL_CURRENT        = 102;
            obj.ADDR.PRESENT_POSITION    = 132; 
            obj.ADDR.PRESENT_VELOCITY    = 128;
            obj.ADDR.OPERATING_MODE      = 11;
            obj.ADDR.MAX_POS             = 48;
            obj.ADDR.MIN_POS             = 52;
            obj.ADDR.MAX_VEL             = 44;
            obj.ADDR.HOMING_OFFSET       = 20;
            obj.ADDR.OPERATING_MODE      = 11;
            obj.ADDR.IS_MOVING           = 122;
            obj.ADDR.PROFILE_VELOCITY    = 112;
            obj.ADDR.MOVING_THRESHOLD    = 24;

            fprintf("Servo "+name+" has been initialised correctly!\n");
        end

        function obj = setAngleOffset(obj, offset, scale)
            obj.userAngle2Servo = @(userAngle) scale*userAngle + offset;
            obj.servoAngle2User = @(servoAngle) (servoAngle - offset) / scale;
        end

%% Dynamixel - related methods
        function obj = disableTorque(obj)
            fprintf("Disabling Torque for "+obj.name+"\n");
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                obj.SERVO_ID, obj.ADDR.TORQUE_ENABLE, 0);
            verifyTxRxResult(obj.port_num, obj.PROTOCOL_VERSION);
            fprintf("Disabled Torque for "+obj.name+"\n");
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
            min = obj.userAngle2Servo(min);
            max = obj.userAngle2Servo(max);

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
            target = obj.userAngle2Servo(target);
            obj.moveToPulse(dynDeg2pulse(target));
        end

        function current_pos = getCurrentPositionPulse(obj)
            current_pos = read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                                obj.SERVO_ID, obj.ADDR.PRESENT_POSITION);
            verifyTxRxResult(obj.port_num, obj.PROTOCOL_VERSION);
        end

        function current_ang = getCurrentPositionDeg(obj)
            current_pos = obj.getCurrentPositionPulse();
            current_ang = dynPulse2deg(current_pos);
            current_ang = obj.servoAngle2User(current_ang);
        end

        function current_vel = getCurrentVelocity(obj)
            current_vel = read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                          obj.SERVO_ID, obj.ADDR.PRESENT_VELOCITY);
            verifyTxRxResult(obj.port_num, obj.PROTOCOL_VERSION);
        end

        function obj = setMaxSpeed(obj, maxspeedrad)
            % write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
            %                 obj.SERVO_ID, obj.ADDR.MAX_VEL, maxspeedrad);
            % verifyTxRxResult(obj.port_num, obj.PROTOCOL_VERSION);

            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                            obj.SERVO_ID, obj.ADDR.PROFILE_VELOCITY, maxspeedrad);
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
            elseif contains(mode, 'pos')
                mode = 3;
            elseif contains(mode, 'curpos')
                mode = 5;
            else 
                mode = 3;
            end
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                            obj.SERVO_ID, obj.ADDR.OPERATING_MODE, mode);
            verifyTxRxResult(obj.port_num, obj.PROTOCOL_VERSION);
        end

        %For current-based mode, should prevent current form getting too
        %high when trying to close gripper
        function obj = setGoalCurrent(obj,current)
            write2ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION,...
                            obj.SERVO_ID, obj.ADDR.GOAL_CURRENT, current);
            verifyTxRxResult(obj.port_num,obj.PROTOCOL_VERSION);
        end

        function obj = setMovingThreshold(obj, threshPulse)
            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                obj.SERVO_ID, obj.ADDR.MOVING_THRESHOLD, threshPulse);
            verifyTxRxResult(obj.port_num, obj.PROTOCOL_VERSION);
        end

        function ismoving = isMoving(obj)
            ismoving = read1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                                    obj.SERVO_ID, obj.ADDR.IS_MOVING);
            verifyTxRxResult(obj.port_num, obj.PROTOCOL_VERSION);
            % ismoving = ~ismoving;
        end
    end
end


