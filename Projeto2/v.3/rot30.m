function ret = rot30(clientID, sim, handleObjects)
%rot30 rotate30deg
%%%%%%%%%%%%%%%%%%%%%%%%
    setSpeed(clientID, sim, handleObjects, -11.65, 11.65)
    pause(1);
    [~ , ~, myAngularSpeed] = sim.simxGetObjectVelocity(clientID, handleObjects(3), sim.simx_opmode_buffer);
    setSpeed(clientID, sim, handleObjects, 0, 0)
    if abs(myAngularSpeed(3)) < 0.01
        disp('Houve um problema ao rotacionar? Possível obstáculo!');
        ret = 1;
    else
        disp('Aparentemente rotacionei 30º ~ 1hora');
        ret = 0;
    end
end

