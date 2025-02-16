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
            fprintf("Initiallised Robot")
        end
        
        function obj = move_linear(obj,p1,p2,n)
            points = linear_interpol(p1,p2,n);
            for i = 1:size(points,2)
                T = [eye(3) points(:,i) ; 0 0 0 1];
                move_to_point(obj.base,obj.shoulder,obj.elbow,obj.wrist,T)
            end
        end

        function obj = move_cubic(obj,p1,p2,n)
            points = cubic_interpol(p1,p2,n);
            for i = 1:size(points,2)
                T = [eye(3) points(:,i) ; 0 0 0 1];
                move_to_point(obj.base,obj.shoulder,obj.elbow,obj.wrist,T)
            end
        end
        
        %Assuming here grippper is in current based position mode (Check
        %this)
        function obj = open_gripper(obj)
            obj.finger.setGoalCurrent(0);
            obj.finger.moveToPulse(0);% This was set arbitrarily, since we haven't checked this servo yet
        end
            
        function obj = close_gripper(obj)
            ojb.finger.setGoalCurrent(25) %Again, arbitrary
            obj.finger.moveToPulse(2048)
        end
    end
end
