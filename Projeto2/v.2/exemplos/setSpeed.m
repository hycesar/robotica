function setSpeed(clientID, speedR, speedL)
    rob = robot();
    
    if abs(speedR) < rob.speedLimit
        sim.simxSetJointTargetVelocity(clientID, handleObjects(rob.idRightMotor), speedR, sim.simx_opmode_oneshot);                
    else
        disp('Erro: Ultrapassou limite de velocidade! Verifique constantes')
    end

    if abs(speedL) < rob.speedLimit
        sim.simxSetJointTargetVelocity(clientID, handleObjects(rob.idLeftMotor), speedL, sim.simx_opmode_oneshot);
    else
        disp('Erro: Ultrapassou limite de velocidade! Verifique constantes')
    end
end