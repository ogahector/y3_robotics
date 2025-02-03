function points = cubic_interpol(p1, p2, n)
    t = linspace(0, 1, n);

    % Cubic interpolation formula: P(t) = P0 + Δ*(3t² - 2t³)
    delta_x = p2(1) - p1(1);
    delta_y = p2(2) - p1(2);
    delta_z = p2(3) - p1(3);
    
    x_cubic = p1(1) + delta_x * (3*t.^2 - 2*t.^3);
    y_cubic = p1(2) + delta_y * (3*t.^2 - 2*t.^3);
    z_cubic = p1(3) + delta_z * (3*t.^2 - 2*t.^3);
    
    % Combine into 3D points
    points = [x_cubic; y_cubic; z_cubic];
end