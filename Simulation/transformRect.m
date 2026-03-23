function [X,Y] = transformRect(x,y,theta,W,H)
    Xc = [-W/2 W/2 W/2 -W/2]; Yc = [-H/2 -H/2 H/2 H/2];
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    pts = R*[Xc;Yc];
    X = pts(1,:)+x; Y = pts(2,:)+y;
end