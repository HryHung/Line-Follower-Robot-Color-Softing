%% ================== HÀM CẢM BIẾN & KIỂM TRA TRACK ==================
function [Xs,Ys] = sensorPositions(x,y,theta,d,gap)
    offsets = (-2:2)*gap;
    Xs = x + d*cos(theta) - offsets*sin(theta);
    Ys = y + d*sin(theta) + offsets*cos(theta);
end