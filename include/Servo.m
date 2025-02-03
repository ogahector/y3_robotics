%% OBJECT POSE CLASS: 
% i have no idea if this is the right direction please modify 
classdef Servo
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

        parent = Servo.empty; %?????????????????????
        child = Servo.empty;  %?????????????????????

        ADDR = struct();
    end

    methods
        function obj = Servo(name, world_frame_ptr, frame, P_initial, ...
                                  SERVO_ID, PROTOCOL_VERSION, DEVICENAME, BAUDRATE)

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
            obj.world_frame = world_frame_ptr;
            obj.frame = frame;
            obj.P = P_initial;


            obj.ADDR.TORQUE_ENABLE       = 64;           
            obj.ADDR.GOAL_POSITION       = 116; 
            obj.ADDR.PRESENT_POSITION    = 132; 
            obj.ADDR.OPERATING_MODE      = 11;
            obj.ADDR.MAX_POS            = 48;
            obj.ADDR.MIN_POS             = 52;
        end

        function obj = servoInit(obj) 
            startup_load_libraries();
            ...
        end

        function obj = applyTransformation(obj, T)
            obj.P = T*obj.P; obj.P = obj.P(1:3);
            % obj.T = T;
            % obj.R = T(1:3, 1:3);
        end

        function obj = applyRotation(obj, R)
            obj.P = R*obj.P;
        end

        function obj = applyTranslation(obj, Tr)
            obj.P = Tr + obj.P;
        end

        function obj = updateWorldFrame(obj, new_WF)
            obj.world_frame = new_WF;
        end

%% Dynamixel - related methods
        function obj = disableTorque(obj)
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                obj.SERVO_ID, obj.ADDR.TORQUE_ENABLE, 0);
        end

        function obj = enableTorque(obj)
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                obj.SERVO_ID, obj.ADDR.TORQUE_ENABLE, 1);
        end

        function obj = setRotationLimitsPulse(obj, min, max)
            obj.disableTorque()

            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                obj.SERVO_ID, obj.ADDR.MIN_POS, min);

            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                obj.SERVO_ID, obj.ADDR.MAX_POS, max);
        end

        function obj = setRotationLimitsDeg(obj, min, max)
            obj.disableTorque()

            obj.setRotationLimitsPulse(obj, dynDeg2pulse(min), dynDeg2pulse(max));
        end

        function obj = moveToPulse(obj, target)
            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                obj.SERVO_ID, obj.ADDR.GOAL_POSITION, target)
        end

        function obj = moveToDeg(obj, target)
            obj.moveToPulse(dynDeg2pulse(target));
        end
    end
end


