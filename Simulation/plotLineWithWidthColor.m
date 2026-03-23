function plotLineWithWidthColor(P1, P2, width, color)
    v = P2 - P1; v = v / norm(v);
    n = [-v(2), v(1)];
    P1a = P1 + n * width/2;
    P1b = P1 - n * width/2;
    P2a = P2 + n * width/2;
    P2b = P2 - n * width/2;
    fill([P1a(1) P2a(1) P2b(1) P1b(1)], ...
         [P1a(2) P2a(2) P2b(2) P1b(2)], color, 'EdgeColor','none');
end