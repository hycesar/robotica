function [ret] = initMotors(clientID, sim, handleObjects, speedR, speedL)
%   initMotors
%   Initialize Motors
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    rob = robot();
    
    sim.simxSetJointTargetVelocity(clientID, handleObjects(rob.idRightMotor), speedR, sim.simx_opmode_oneshot);
    sim.simxSetJointTargetVelocity(clientID, handleObjects(rob.idLeftMotor), speedL, sim.simx_opmode_oneshot);
    
    ret = 0;
end

