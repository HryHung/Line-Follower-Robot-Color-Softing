function dmin = pointToTrackDistance(x, y, segments)
    % Trả về khoảng cách nhỏ nhất từ điểm (x,y) đến đường line trung tâm track
    dmin = inf;
    for i = 1:length(segments)
        seg = segments{i};
        switch seg.type
            case 'line'
                d = pointSegmentDistance([x,y], seg.P1, seg.P2);
            case 'arcR'
                C = arcCenterSafe(seg.P1, seg.P2, seg.R, seg.cw);
                d = abs(norm([x,y]-C) - seg.R);
        end
        if d < dmin, dmin = d; end
    end
end