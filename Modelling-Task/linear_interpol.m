function points = linear_interpol(p1, p2, n)
    if size(p1) ~= size(p2)
        error('Points of Different sizes were given.')
    end

    t = linspace(0, 1, n);

    x_linear = p1(1) + t * (p2( 1) - p1(1));
    y_linear = p1(2) + t * (p2(2) - p1(2));
    z_linear = p1(3) + t * (p2(3) - p1(3));

    points = [x_linear; y_linear; z_linear];
end