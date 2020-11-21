function [ForceVector] = Repulsive(pos,obstacle,RepStrength,poScale)
% calculates repulsive force vector field 

% translate rect obstacle into spherical


po = poScale;


%Pi = norm(pos - obstOrigin);
[Pi,unit]=distPointToBox(pos, obstacle);
    if (po > Pi)
        ForceVector = -RepStrength*((1/Pi)-(1/po))*((po/Pi)^2)*unit;
        %ForceVector = -RepStrength*((1/Pi)-(1/po))*((1/Pi)^2)*unit;
    else
        ForceVector = [0, 0, 0];
    end
end

