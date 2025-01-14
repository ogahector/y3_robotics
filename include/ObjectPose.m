%% OBJECT POSE CLASS: 
% i have no idea if this is the right direction please modify 
classdef ObjectPose
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
    end

    methods
        % function obj = ObjectPose(name, other, world_frame_name, frame, P_initial, offset)
        function obj = ObjectPose(name, world_frame_ptr, frame, P_initial, ...
                                  SERVO_ID, PROTOCOL_VERSION, DEVICENAME, BAUDRATE)
        % function obj = ObjectPose(varargin)
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
    end
end


