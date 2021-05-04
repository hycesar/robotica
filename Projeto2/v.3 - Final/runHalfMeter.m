function ret = runHalfMeter(clientID, sim, handleObjects)
% ANDAR 0.5 METRO
%%%%%%%%%%%%%%
    setSpeed(clientID, sim, handleObjects, 9, 9)
    i = 0;
    status = 0;
    while i < 10
        pause(1);
        [~ , myLinearSpeed, ~] = sim.simxGetObjectVelocity(clientID, handleObjects(3), sim.simx_opmode_buffer);
        if abs(myLinearSpeed(1)) < 0.03 && abs(myLinearSpeed(2)) < 0.03
            status = 1;
            disp('Houve um problema durante o trajeto? Possível obstáculo!');
        else
            status = 0;
        end
        i = i + 1;
    end
    setSpeed(clientID, sim, handleObjects, 0, 0)
    if ~status
        disp('Aparentemente andei 0.5m');
    end
    ret = status;
end

