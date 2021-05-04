function setSpeed(clientID, sim, handleObjects, speedR, speedL)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    rob = robot();
    if abs(speedR) < rob.speedLimit || abs(speedL) < rob.speedLimit
                sim.simxSetJointTargetVelocity(clientID, handleObjects(rob.idRightMotor), speedR, sim.simx_opmode_oneshot);                
                sim.simxSetJointTargetVelocity(clientID, handleObjects(rob.idLeftMotor), speedL, sim.simx_opmode_oneshot);
    else
        disp('Erro: Ultrapassou limite de velocidade!')
    end
end

