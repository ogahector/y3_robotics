%To simplify directing the robot. May not be necessary but could be nice to
%have
classdef Robot_4DOF
    properties (Access = public)
        base ServoDynamixel;
        shoulder ServoDynamixel;
        elbow ServoDynamixel;
        wrist ServoDynamixel;
        finger ServoDynamixel;
    end
    methods
        function obj = Robot_4DOF(base,shoulder,elbow,wrist,finger)
            obj.base = base;
            obj.shoulder = shoulder;
            obj.elbow = elbow;
            obj.wrist = wrist;
            obj.finger = finger;%Might wanna ensure mode here
            fprintf("Initialised Robot")
        end
        
        function obj = move_linear(obj,p1,p2,n, yDeg)
            arguments
                obj = [];
                p1 = 0;
                p2 = 0;
                n = 20;
                yDeg = 0;
            end

            points = linear_interpol(p1,p2,n);
            rot = makehgtform('yrotate', deg2rad(yDeg));
            for i = 1:size(points,2)
                T = [rot(1:3,1:3) points(:,i) ; 0 0 0 1];
                move_to_point(obj.base,obj.shoulder,obj.elbow,obj.wrist,T)
            end
        end

        function obj = move_cubic(obj,p1,p2,n, yDeg)
            arguments
                obj = [];
                p1 = 0;
                p2 = 0;
                n = 20;
                yDeg = 0;
            end

            points = cubic_interpol(p1,p2,n);
            rot = makehgtform('yrotate', deg2rad(yDeg));
            for i = 1:size(points,2)
                T = [rot(1:3,1:3) points(:,i) ; 0 0 0 1];
                T(1:3, 4) = points(:, i);
                move_to_point(obj.base,obj.shoulder,obj.elbow,obj.wrist,T)
            end
        end

        function obj = move(obj, point, yDeg) % point is a row vector
            rot = makehgtform('yrotate', deg2rad(yDeg));
            T = [rot(1:3, 1:3), point ; 0 0 0 1];
            move_to_point(obj.base, obj.shoulder, obj.elbow, obj.wrist, T);
        end

        function obj = applyRotation(obj, rot)
            T = [rot, [0;0;0];0 0 0 1];
            move_to_point(obj.base, obj.shoulder, obj.elbow, obj.wrist, T);
        end
        
        %Assuming here grippper is in current based position mode (Check
        %this)
        function obj = open_gripper(obj)
            obj.finger.moveToDeg(250);% This was set arbitrarily, since we haven't checked this servo yet
        end
            
        function obj = close_gripper(obj)
            obj.finger.moveToDeg(150);
        end

        function obj = enableTorque(obj)
            obj.base.enableTorque();
            obj.shoulder.enableTorque();
            obj.elbow.enableTorque();
            obj.wrist.enableTorque();
            obj.finger.enableTorque();
        end

        function obj = disableTorque(obj)
            obj.base.disableTorque();
            obj.shoulder.disableTorque();
            obj.elbow.disableTorque();
            obj.wrist.disableTorque();
            obj.finger.disableTorque();
        end
    end
end
