function [handleObjects] = initHandles(clientID, sim)
%   Inicialize objects
%   get ids and initialize
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    handleNames = {'RobotFrame#', 'LeftMotor#', 'RightMotor#', 'Target#', 'velodyneHDL_64E_S2#', 'Vision_sensorL#', 'Vision_sensorR#', 'sensorDistancia#'};
    handleSize = size(handleNames, 2);
    handleObjects = zeros(1, handleSize);
    for i = 1:handleSize
        [ret, handleObjects(i)] = sim.simxGetObjectHandle(clientID, handleNames{i}, sim.simx_opmode_blocking);
        sim.simxGetObjectPosition(clientID, handleObjects(i), -1, sim.simx_opmode_streaming);
        sim.simxGetObjectOrientation(clientID, handleObjects(i), -1, sim.simx_opmode_streaming);
        sim.simxGetVisionSensorImage2(clientID, handleObjects(i), 0, sim.simx_opmode_streaming);
        sim.simxGetObjectVelocity(clientID, handleObjects(i), sim.simx_opmode_streaming);
        sim.simxReadProximitySensor(clientID, handleObjects(i), sim.simx_opmode_streaming);
        if ret ~= 0
            disp(['Erro ' num2str(ret)]);
        end
    end
end

