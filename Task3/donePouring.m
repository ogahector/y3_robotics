function done_pouring = donePouring(r, h, L, thetaDeg, Vpour, accuracy)
arguments
    r; % radius of pouring beaker
    h; % initial height of liquid in beaker
    L; % pouring beaker height
    thetaDeg; % current tilt angle DEG
    Vpour; % how much volume you want poured
    accuracy = 0; % error threshold 
end
    % theta = deg2rad(thetaDeg);
    % hx = @(x) h - x*tan(theta);
    % V_rem_fun = @(x) (r.^2).*acos( (r - hx(x))./r ) - (r - hx(x)).*sqrt(2.*r.*hx(x) - hx(x).^2);
    % 
    % V_rem = integral(V_rem_fun, 0, L)
    % 
    % Vpoured = pi*r^2*h - V_rem;
    % 
    % done_pouring = boolean( Vpour - Vpoured <= 0 );

    

    theta = deg2rad(thetaDeg);

    % V_initial = pi*r^2*h;

    V_poured = (2/3) * r^3 * sin(theta)

    done_pouring = boolean(Vpour - V_poured >= accuracy);
    
end