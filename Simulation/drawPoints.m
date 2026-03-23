function drawPoints(points)
    fn = fieldnames(points);
    for k = 1:numel(fn)
        P = points.(fn{k});
        plot(P(1), P(2), 'ro', 'MarkerFaceColor','r');
        text(P(1)+30, P(2), fn{k}, 'FontWeight','bold', 'Color','r');
    end
end