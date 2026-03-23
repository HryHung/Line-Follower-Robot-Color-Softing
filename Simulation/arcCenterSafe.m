function [ok,O] = arcCenterSafe(P1,P2,R,cw)
    M = (P1+P2)/2; halfChord = norm(P2-P1)/2;
    if halfChord > R+1e-9, ok=false; O=[NaN NaN]; return; end
    h = sqrt(max(0,R^2-halfChord^2)); v = (P2-P1)/norm(P2-P1);
    n = [-v(2),v(1)]; if cw, O=M-h*n; else, O=M+h*n; end
    ok=true;
end