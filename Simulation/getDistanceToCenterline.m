function d_min = getDistanceToCenterline(P, segments)
    % (Hàm này giữ nguyên, không cần sửa)
    d_min = inf;
    for i = 1:length(segments)
        seg = segments{i};
        d = inf; 
        switch seg.type
            case 'line'
                d = pointSegmentDistance(P, seg.P1, seg.P2);     
            case 'arcR'
                [ok, O] = arcCenterSafe(seg.P1, seg.P2, seg.R, seg.cw);
                if ~ok, continue; end 
                ang1 = atan2(seg.P1(2)-O(2), seg.P1(1)-O(1));
                ang2 = atan2(seg.P2(2)-O(2), seg.P2(1)-O(1));
                angP = atan2(P(2)-O(2), P(1)-O(1)); 
                is_inside_angular_range = false;
                if seg.cw
                    if ang2 > ang1, ang2 = ang2 - 2*pi; end
                    angP = normalizeAngleNear(angP, ang1);
                    if angP <= ang1 && angP >= ang2
                        is_inside_angular_range = true;
                    end
                else
                    if ang2 < ang1, ang2 = ang2 + 2*pi; end
                    angP = normalizeAngleNear(angP, ang1);
                    if angP >= ang1 && angP <= ang2
                        is_inside_angular_range = true;
                    end
                end
                if is_inside_angular_range
                    d = abs(norm(P - O) - seg.R);
                else
                    d = min(norm(P - seg.P1), norm(P - seg.P2));
                end
        end
        if d < d_min
            d_min = d;
        end
    end
end