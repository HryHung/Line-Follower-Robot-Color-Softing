function drawTrackBlack(segments)
    for i = 1:length(segments)
        seg = segments{i};
        switch seg.type
            case 'line'
                plotLineWithWidthColor(seg.P1, seg.P2, seg.width, [0 0 0]);
            case 'arcR'
                plotArcFixedRadiusDirColor(seg.P1, seg.P2, seg.R, seg.width, seg.cw, [0 0 0]);
        end
    end
end
