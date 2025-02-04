function new_ang = angle_to_servo(angles_rad)
    [~, nangles] = size(angles_rad);
    angles = [];
    
    for i = 1 : nangles
        angles(1, i) = dynRad2pulse(-angles_rad(1, i));
        angles(2, i) = dynRad2pulse(-angles_rad(1, i));
        angles(3, i) = dynRad2pulse(-angles_rad(1, i));
        angles(4, i) = dynRad2pulse(-angles_rad(1, i));
    
        angles(1, i) = angles(1, i) + dynDeg2pulse(180);
        angles(2, i) = angles(1, i) + dynDeg2pulse(270 - 10.62);
        angles(3, i) = angles(1, i) + dynDeg2pulse(90 + 10.62);
        angles(4, i) = angles(1, i) + dynDeg2pulse(180);
    end
    new_ang = angles;
end