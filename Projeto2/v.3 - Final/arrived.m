function [ret] = arrived(targetPosition, targetOrientation, myPosition, myOrientation)
%   ARRIVED Chegou no destino?
%   Check if arrived there
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ret = 0;
    if abs(targetPosition(1) - myPosition(1))  < 0.01 && abs(targetPosition(2) - myPosition(2)) < 0.01
        if abs(targetOrientation(3) - myOrientation(3))  < 0.01
            ret = 1;
        end
    end
end

