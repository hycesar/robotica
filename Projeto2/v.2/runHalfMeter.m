function runHalfMeter(clientID, sim, handleObjects)
% ANDAR 0.5 METRO
%%%%%%%%%%%%%%
    setSpeed(clientID, sim, handleObjects, 9, 9)
    pause(10);
    setSpeed(clientID, sim, handleObjects, 0, 0)
end

