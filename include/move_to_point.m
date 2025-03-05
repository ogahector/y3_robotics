function move_to_point(base, shoulder, elbow, wrist, point)%Could move to robot class
    arguments
        base ServoDynamixel
        shoulder ServoDynamixel
        elbow ServoDynamixel
        wrist ServoDynamixel
        point double
    end
    
    angles = IK(point);
    angles = rad2deg(angles);

    %Need to check if angles are within within movement range
    % angles(1) = max(min(angles(1), 90), -90);
    % angles(2) = max(min(angles(2), 105), 5);
    % angles(3) = max(min(angles(3), 122), -85);
    angles(4) = max(min(angles(4), 104), -120);
    % This will just stop the angles exceeding these values, could also
    % wrap them back around to within the ranges defined above

    % base.moveToDeg(angles(1));
    % shoulder.moveToDeg(angles(2));
    % elbow.moveToDeg(angles(3));
    % wrist.moveToDeg(angles(4));

    angles(1) = base.userAngle2Servo(angles(1));
    angles(2) = shoulder.userAngle2Servo(angles(2));
    angles(3) = elbow.userAngle2Servo(angles(3));
    angles(4) = wrist.userAngle2Servo(angles(4));
    
    group = base.port_num;
    groupSyncWrite(group, base.PROTOCOL_VERSION, base.ADDR.GOAL_POSITION, 4);

    groupSyncWriteAddParam(group, base.SERVO_ID, angles(1), 4);
    groupSyncWriteAddParam(group, shoulder.SERVO_ID, angles(2), 4)
    groupSyncWriteAddParam(group, elbow.SERVO_ID, angles(3), 4)
    groupSyncWriteAddParam(group, wrist.SERVO_ID, angles(4), 4)

    groupSyncWrite(group, base.PROTOCOL_VERSION, base.ADDR.GOAL_POSITION, 4);

    groupSyncWriteTxPacket(group);

end