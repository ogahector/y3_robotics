%To simplify directing the robot. May not be necessary but could be nice to
%have
classdef Robot_4DOF
    properties (Access = public)
        base ServoDynamixel;
        shoulder ServoDynamixel;
        elbow ServoDynamixel;
        wrist ServoDynamixel;
        finger ServoDynamixel;
        Gripper_Open double;
        Gripper_Slight double;
        Gripper_Close double;
        groupwrite;
        groupreadmov;
    end
    methods
        function obj = Robot_4DOF(base,shoulder,elbow,wrist,finger,open,slight,close)
            obj.base = base;
            obj.shoulder = shoulder;
            obj.elbow = elbow;
            obj.wrist = wrist;
            obj.finger = finger;%Might wanna ensure mode here
            obj.Gripper_Open = open;
            obj.Gripper_Close = close;
            obj.Gripper_Slight = slight;

            mov_threshold = 1;
            base.setMovingThreshold(mov_threshold);
            shoulder.setMovingThreshold(mov_threshold);
            elbow.setMovingThreshold(mov_threshold);
            wrist.setMovingThreshold(mov_threshold);
            finger.setMovingThreshold(mov_threshold);

            maxspeed = 40;
            base.setMaxSpeed(maxspeed);
            shoulder.setMaxSpeed(maxspeed);
            elbow.setMaxSpeed(maxspeed);
            wrist.setMaxSpeed(maxspeed);
            finger.setMaxSpeed(maxspeed);

            obj.groupwrite = groupSyncWrite(obj.base.port_num, base.PROTOCOL_VERSION, base.ADDR.GOAL_POSITION, 4);
            obj.groupreadmov = groupSyncRead(obj.base.port_num, base.PROTOCOL_VERSION, base.ADDR.IS_MOVING, 1);
            fprintf("Initialised Robot!\n\n")
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
                move_to_point(obj.base,obj.shoulder,obj.elbow,obj.wrist,T);
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
                move_to_point(obj.base,obj.shoulder,obj.elbow,obj.wrist,T);
            end
        end

        function obj = move_cubic_sync(obj,p1,p2,n, yDeg)
            arguments
                obj = [];
                p1 = 0;
                p2 = 0;
                n = 20;
                yDeg = 0;
            end

            points = cubic_interpol(p1,p2,n);
            for i = 1:size(points,2)
                obj.move_sync(points(:,i),yDeg);
            end
        end

        function obj = move_cubic_sync_time(obj, p1, p2, n, yDeg1, yDeg2)
            arguments
                obj = [];
                p1 = [0;0;0];
                p2 = [0;0;0];
                n = 20;
                yDeg1 = 0;
                yDeg2 = 0;
            end
            theta11 = atan2(p1(2), p1(1));
            zrot1 = makehgtform('zrotate', theta11);
            yrot1 = makehgtform('yrotate', deg2rad(yDeg1));
            rot1 = zrot1 * yrot1;
            T1 = [rot1(1:3,1:3), p1; 0 0 0 1];
            angles1 = IK(T1);

            theta12 = atan2(p2(2), p2(1));
            zrot2 = makehgtform('zrotate', theta12);
            yrot2 = makehgtform('yrotate', deg2rad(yDeg2));
            rot2 = zrot2 * yrot2;
            T2 = [rot2(1:3,1:3), p2; 0 0 0 1];
            angles2 = IK(T2);

            t = linspace(0, 1, n);

            angles1 = rad2deg(angles1)
            angles2 = rad2deg(angles2)

            delta_theta = angles2 - angles1

            theta1 = angles1(1) + delta_theta(1) * (3*t.^2 - 2*t.^3);
            theta2 = angles1(2) + delta_theta(2) * (3*t.^2 - 2*t.^3);
            theta3 = angles1(3) + delta_theta(3) * (3*t.^2 - 2*t.^3);
            theta4 = angles1(4) + delta_theta(4) * (3*t.^2 - 2*t.^3);

            theta4 = max(min(theta4, 104), -120);

            theta1 = obj.base.userAngle2Servo(theta1);
            theta2 = obj.shoulder.userAngle2Servo(theta2);
            theta3 = obj.elbow.userAngle2Servo(theta3);
            theta4 = obj.wrist.userAngle2Servo(theta4);

            

            theta1 = dynDeg2pulse(theta1);
            theta2 = dynDeg2pulse(theta2);
            theta3 = dynDeg2pulse(theta3);
            theta4 = dynDeg2pulse(theta4);

            for i = 1 : n
                groupSyncWriteClearParam(obj.groupwrite);

                groupSyncWriteAddParam(obj.groupwrite, obj.base.SERVO_ID, theta1(i), 4);
                groupSyncWriteAddParam(obj.groupwrite, obj.shoulder.SERVO_ID, theta2(i), 4);
                groupSyncWriteAddParam(obj.groupwrite, obj.elbow.SERVO_ID, theta3(i), 4);
                groupSyncWriteAddParam(obj.groupwrite, obj.wrist.SERVO_ID, theta4(i), 4);

                groupSyncWriteTxPacket(obj.groupwrite);

            end

        end

        function obj = move(obj, point, yDeg) % point is a row vector
            rot = makehgtform('yrotate', deg2rad(yDeg));
            T = [rot(1:3, 1:3), point ; 0 0 0 1];
            move_to_point(obj.base, obj.shoulder, obj.elbow, obj.wrist, T);
        end

        function obj = move_sync(obj, point, yDeg)
            theta1 = atan2(point(2), point(1));
            rot_z = makehgtform('zrotate',theta1);
            rot_y = makehgtform('yrotate', deg2rad(yDeg));
            rot = rot_z * rot_y;
            T = [rot(1:3, 1:3), point ; 0 0 0 1];

            angles = IK(T);
            angles = rad2deg(angles);
        
            %Need to check if angles are within within movement range
            % angles(1) = max(min(angles(1), 90), -90);
            % angles(2) = max(min(angles(2), 105), 5);
            % angles(3) = max(min(angles(3), 122), -85);
             angles(4) = max(min(angles(4), 104), -120);
        
            angles(1) = dynDeg2pulse(obj.base.userAngle2Servo(angles(1)));
            angles(2) = dynDeg2pulse(obj.shoulder.userAngle2Servo(angles(2)));
            angles(3) = dynDeg2pulse(obj.elbow.userAngle2Servo(angles(3)));
            angles(4) = dynDeg2pulse(obj.wrist.userAngle2Servo(angles(4)));

            groupSyncWriteClearParam(obj.groupwrite);

            groupSyncWriteAddParam(obj.groupwrite, obj.base.SERVO_ID, angles(1), 4);
            groupSyncWriteAddParam(obj.groupwrite, obj.shoulder.SERVO_ID, angles(2), 4);
            groupSyncWriteAddParam(obj.groupwrite, obj.elbow.SERVO_ID, angles(3), 4);
            groupSyncWriteAddParam(obj.groupwrite, obj.wrist.SERVO_ID, angles(4), 4);
        
            groupSyncWriteTxPacket(obj.groupwrite);

        end

        function obj = applyRotation(obj, rot)
            T = [rot, [0;0;0];0 0 0 1];
            move_to_point(obj.base, obj.shoulder, obj.elbow, obj.wrist, T);
        end
        
        %Assuming here grippper is in current based position mode (Check
        %this)
        function obj = open_gripper(obj)
            obj.finger.moveToDeg(obj.Gripper_Open);% This was set arbitrarily, since we haven't checked this servo yet
        end

        function obj = open_gripper_slightly(obj)
            obj.finger.moveToDeg(obj.Gripper_Slight);
        end
            
        function obj = close_gripper(obj)
            obj.finger.moveToDeg(obj.Gripper_Close);
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
            % groupSyncReadClearParam(obj.groupreadmov)
            % 
            % groupSyncReadAddParam(obj.base.port_num, obj.base.SERVO_ID);
            % groupSyncReadAddParam(obj.base.port_num, obj.shoulder.SERVO_ID)
            % groupSyncReadAddParam(obj.base.port_num, obj.elbow.SERVO_ID)
            % groupSyncReadAddParam(obj.base.port_num, obj.wrist.SERVO_ID)
            % 
            % groupSyncReadTxRxPacket(obj.groupreadmov);
            % groupSyncReadRxPacket(obj.groupreadmov);
            % ismoving = groupSyncReadGetData(obj.groupreadmov, obj.base.SERVO_ID, obj.base.ADDR.IS_MOVING, 1)

            % ismoving = 
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
            
            obj.wrist.moveToDeg(0);
            obj.elbow.moveToDeg(-90);
            obj.shoulder.moveToDeg(90);
            obj.base.moveToDeg(-90);
            obj.waitUntilDone();

            obj.move_sync(initpoint, 0);
            obj.waitUntilDone();

            obj.open_gripper();
            obj.waitUntilDone();
            obj.close_gripper();
            obj.waitUntilDone();
        end

        function obj = rotateCubeNTimes(obj, n, cube_coord, z_limCM)
            arguments
                obj;
                n {mustBeInteger};
                cube_coord {mustBeNumeric};
                z_limCM double;
            end
            
            coord_up = grid2cm([cube_coord, 4*z_limCM])';
            coord_down = grid2cm([cube_coord, z_limCM + 1.5])';

            obj.move_sync(coord_up, 0);
            obj.waitUntilDone();
            
            obj.open_gripper();
            obj.waitUntilDone();
            
            obj.move_cubic_sync(coord_up, coord_down, 10, 0);
            obj.waitUntilDone();
            
            for i = 1 : n
                theta1 = atan2(cube_coord(2), cube_coord(1));
                horizontal_gripper_correction = 2 * [cos(theta1); sin(theta1); 0];
            
                obj.close_gripper();
                obj.waitUntilDone();
            
                obj.move_cubic_sync(coord_down, coord_up, 10, 0);
                obj.waitUntilDone();
            
                obj.move(coord_up, 0);
                obj.waitUntilDone();
            
                obj.move_cubic_sync(coord_up, coord_down, 10, 90);
                obj.waitUntilDone();
            
                obj.open_gripper();
                obj.waitUntilDone();
            
                obj.move_sync(coord_down, 0);
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
