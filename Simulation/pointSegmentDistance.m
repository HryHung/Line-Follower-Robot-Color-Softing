function d = pointSegmentDistance(P,A,B)
    v = B-A; L2 = dot(v,v);
    if L2 < eps, d = norm(P-A); return; end
    t = dot(P-A,v)/L2; t = max(0,min(1,t));
    Q = A + t*v; d = norm(P-Q);
end
