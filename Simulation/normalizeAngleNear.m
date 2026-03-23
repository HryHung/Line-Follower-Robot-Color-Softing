function ang = normalizeAngleNear(ang,angRef)
    while ang-angRef > pi, ang = ang-2*pi; end
    while ang-angRef < -pi, ang = ang+2*pi; end
end