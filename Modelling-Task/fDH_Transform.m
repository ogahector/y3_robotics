function [T] = fDH_Transform(DH_Table, Link_No)
%FDH_TRANSFORM Genenerate DH Transform for a specified link
%   Link_No  < size(DH_Table)(1) - 1
% DH_Table Layout: a_{i-1}, \alpha_{i-1}, d_i, \theta_i
    theta = DH_Table(Link_No, 4);
    d = DH_Table(Link_No, 3);
    a = DH_Table(Link_No, 1);% a_{i-1}
    alpha = DH_Table(Link_No, 2); % alpha_{i-1}
    T = [
        cos(theta), -sin(theta), 0, a;
        sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d;
        sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d;
        0, 0, 0, 1;]

end

