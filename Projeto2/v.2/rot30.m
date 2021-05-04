function rot30(clientID, sim, handleObjects)
%rot30 rotate30deg
%%%%%%%%%%%%%%%%%%%%%%%%
    setSpeed(clientID, sim, handleObjects, -11.65, 11.65)
    pause(1);
    setSpeed(clientID, sim, handleObjects, 0, 0)
end

