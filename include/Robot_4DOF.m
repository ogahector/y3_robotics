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

            mov_threshold = 1;
            base.setMovingThreshold(mov_threshold);
            shoulder.setMovingThreshold(mov_threshold);
            elbow.setMovingThreshold(mov_threshold);
            wrist.setMovingThreshold(mov_threshold);
            finger.setMovingThreshold(mov_threshold);

            maxspeed = 30;
            base.setMaxSpeed(maxspeed);
            shoulder.setMaxSpeed(maxspeed);
            elbow.setMaxSpeed(maxspeed);
            wrist.setMaxSpeed(maxspeed);
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

        function obj = open_gripper_slightly(obj)
            obj.finger.moveToDeg(200);
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

        function obj = setMaxSpeed(obj, maxSpeed)
            obj.base.setMaxSpeed(maxSpeed);
            obj.shoulder.setMaxSpeed(maxSpeed);
            obj.elbow.setMaxSpeed(maxSpeed);
            obj.wrist.setMaxSpeed(maxSpeed);
            obj.finger.setMaxSpeed(maxSpeed);
        end

        function angles = getAngles(obj)
            angles(1) = obj.base.getCurrentPositionDeg();
            angles(2) = obj.shoulder.getCurrentPositionDeg();
            angles(3) = obj.elbow.getCurrentPositionDeg();
            angles(4) = obj.wrist.getCurrentPositionDeg();
        end


        function ismoving = isMoving(obj)
            ismoving = isMoving(obj.base) || isMoving(obj.shoulder) ...
                   || isMoving(obj.elbow) || isMoving(obj.wrist) ...
                   || isMoving(obj.finger);
        end

        function obj = waitUntilDone(obj)
            while obj.isMoving()
                pause(0.1)
            end
        end


        %% ---- ROUTINES ---- %%
        function obj = initMovementRoutine(obj, initpoint)
            obj.disableTorque();

            obj.base.enableTorque();
            obj.shoulder.enableTorque();
            obj.elbow.enableTorque();
            obj.wrist.enableTorque();
            obj.finger.enableTorque();
            pause(1)
            
            obj.base.moveToDeg(-90)
            obj.shoulder.moveToDeg(90)
            obj.elbow.moveToDeg(-90)
            obj.wrist.moveToDeg(0)
            obj.waitUntilDone();

            obj.move(initpoint, 0);
            obj.waitUntilDone();

            obj.open_gripper();
            obj.waitUntilDone();
            obj.close_gripper();
            obj.waitUntilDone();
        end

        function obj = rotateCubeNTimes(obj, n, cube_coord)
            arguments
                obj {Robot_4DOF};
                n {mustBeInteger};
                cube_coord {mustBeNumeric};
            end
            
            coord_up = grid2cm([cube_coord, 6])';
            coord_down = grid2cm([cube_coord, 3])';
            obj.move(coord_up, 90);
            
            obj.open_gripper();
            obj.waitUntilDone();
            
            obj.move_cubic(coord_up, coord_down, 10, 90);
            obj.waitUntilDone();
            
            for i = 1 : n
            
                obj.close_gripper();
                obj.waitUntilDone();
            
                obj.move_cubic(coord_down, coord_up, 10, 90);
                obj.waitUntilDone();
            
                obj.move(coord_up, 0);
                obj.waitUntilDone();
            
                obj.move_cubic(coord_up, coord_down, 10, 0);
                obj.waitUntilDone();
            
                obj.open_gripper();
                obj.waitUntilDone();
            
                obj.move(coord_down, 90);
                obj.waitUntilDone();
            end
            
            
        end
        
        %% ---- TASKS ---- %%
        %% looking back i dont think the tasks should just be like this lol sorry
        function obj = Task1a(obj, grid_start1, grid_end1, ...
                                grid_start2, grid_end2, ...
                                grid_start3, grid_end3)
            coords = [
                grid2cm([grid_start1, 6]);
                grid2cm([grid_start1, 3]);

                grid2cm([grid_end1, 6]);
                grid2cm([grid_end1, 3]);

                grid2cm([grid_start2, 6]);
                grid2cm([grid_start2, 3]);

                grid2cm([grid_end2, 6]);
                grid2cm([grid_end2, 3]);

                grid2cm([grid_start3, 6]);
                grid2cm([grid_start3, 3]);

                grid2cm([grid_end3, 6]);
                grid2cm([grid_end3, 3]);
            ];

            obj.disableTorque();

            obj.initMovementRoutine();

            for i = 1 : 3
                % i'm aware this + 3*(i-1) looks cursed, i'm sorry
                obj.move(coords(1 + 3*(i-1), :)', 90);
                obj.waitUntilDone();
                obj.open_gripper();
                obj.waitUntilDone();
                obj.move(coords(2 + 3*(i-1), :)', 90);
                obj.waitUntilDone();
                obj.close_gripper();
                obj.waitUntilDone();
                obj.move(coords(1 + 3*(i-1), :)', 90);
                obj.waitUntilDone();
                
                obj.move(coords(3 + 3*(i-1), :)', 90);
                obj.waitUntilDone();
                obj.move(coords(4 + 3*(i-1), :)', 90);
                obj.waitUntilDone();
                obj.open_gripper();
                obj.waitUntilDone();
                obj.move(coords(3 + 3*(i-1), :)', 90);
                obj.waitUntilDone();
            end

            fprintf("Task1A Done!\n");
            pause(5);
        end
    end
end
