function plotArcFixedRadiusDirColor(P1, P2, R, width, cw, color)
    M = (P1 + P2)/2; d = norm(P2-P1)/2;
    if d > R, plotLineWithWidthColor(P1,P2,width,color); return; end
    h = sqrt(max(0,R^2 - d^2));
    v = (P2-P1)/norm(P2-P1); n = [-v(2), v(1)];
    if cw, O = M - h*n; else, O = M + h*n; end
    theta1 = atan2(P1(2)-O(2), P1(1)-O(1));
    theta2 = atan2(P2(2)-O(2), P2(1)-O(1));
    if cw && theta2 > theta1, theta2 = theta2 - 2*pi; end
    if ~cw && theta2 < theta1, theta2 = theta2 + 2*pi; end
    theta = linspace(theta1, theta2, 300);
    x_out = O(1) + (R + width/2) * cos(theta);
    y_out = O(2) + (R + width/2) * sin(theta);
    x_in  = O(1) + (R - width/2) * cos(fliplr(theta));
    y_in  = O(2) + (R - width/2) * sin(fliplr(theta));
    fill([x_out x_in], [y_out y_in], color, 'EdgeColor','none');
end
