function inside = onTrack(x,y,segments)
    P = [x,y]; inside = false;
    for i = 1:length(segments)
        seg = segments{i};
        switch seg.type
            case 'line'
                d = pointSegmentDistance(P, seg.P1, seg.P2);
                if d <= seg.width/2, inside = true; return; end
            case 'arcR'
                [ok,O] = arcCenterSafe(seg.P1, seg.P2, seg.R, seg.cw);
                if ~ok, continue; end
                dR = norm(P - O);
                if abs(dR - seg.R) > seg.width/2, continue; end
                angP = atan2(P(2)-O(2), P(1)-O(1));
                ang1 = atan2(seg.P1(2)-O(2), seg.P1(1)-O(1));
                ang2 = atan2(seg.P2(2)-O(2), seg.P2(1)-O(1));
                if seg.cw
                    if ang2 > ang1, ang2 = ang2 - 2*pi; end
                    angP = normalizeAngleNear(angP, ang1);
                    if angP <= ang1 && angP >= ang2, inside = true; return; end
                else
                    if ang2 < ang1, ang2 = ang2 + 2*pi; end
                    angP = normalizeAngleNear(angP, ang1);
                    if angP >= ang1 && angP <= ang2, inside = true; return; end
                end
        end
    end
end