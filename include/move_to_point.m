function move_to_point(base, shoulder, elbow, wrist, point)%Could move to robot class
    arguments
        base ServoDynamixel
        shoulder ServoDynamixel
        elbow ServoDynamixel
        wrist ServoDynamixel
        point double
    end
    
    angles = IK_H(point);

    %Need to check if angles are within within movement range
    % angles(1) = max(min(angles(1),-90),90);
    % angles(2) = max(min(angles(2),0),180);
    % angles(3) = max(min(angles(3),-90),90);
    % angles(4) = max(min(angles(4),-90),90);
    % This will just stop the angles exceeding these values, could also
    % wrap them back around to within the ranges defined above
    angles = rad2deg(angles)
    base.moveToDeg(angles(1))
    shoulder.moveToDeg(angles(2))
    elbow.moveToDeg(angles(3))
    wrist.moveToDeg(angles(4))
end