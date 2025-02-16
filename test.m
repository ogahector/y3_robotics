p1 = [1,2,3];
p2 = [5,6,15];

points = cubic_interpol(p1,p2,25);

plot3(points(1,:),points(2,:),points(3,:),"-o")
view(3)