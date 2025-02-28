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

    base.moveToDeg(angles(1));
    shoulder.moveToDeg(angles(2));
    elbow.moveToDeg(angles(3));
    wrist.moveToDeg(angles(4));
end