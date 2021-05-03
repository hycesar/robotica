% Deve existir servidor rodando, ative uma vez usando:
% simRemoteApi.start(19999)
% para cada simxStart, um simxFinish

function main()
    disp('Programa iniciado')   %gui
    rob = robot();
    sim = remApi('remoteApi');  %api (,'extApi.h')
    sim.simxFinish(-1);         %terminar eventual processo aberto
    
    clientID = sim.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
    if(clientID > -1)
        disp('Conectado com sucesso ao servidor')
        sim.simxAddStatusbarMessage(clientID, 'Conectado com sucesso ao cliente', sim.simx_opmode_oneshot);
        
        %get ids
        handleNames = {'RobotFrame#', 'LeftMotor#', 'RightMotor#', 'Target#', 'velodyneHDL_64E_S2#', 'Vision_sensorL', 'Vision_sensorR'};
        handleSize = size(handleNames, 2);
        handleObjects = zeros(1, handleSize);
        for i = 1:size(handleNames, handleSize)
            [ret, handleObjects(i)] = sim.simxGetObjectHandle(clientID, handleNames{i}, sim.simx_opmode_blocking);
            sim.simxGetObjectPosition(clientID, handleObjects(i), -1, sim.simx_opmode_streaming);
            sim.simxGetObjectOrientation(clientID, handleObjects(i), -1, sim.simx_opmode_streaming);
            sim.simxGetVisionSensorImage2(clientID, handleObjects(i), 0, sim.simx_opmode_streaming);
            if ret ~= 0
                disp(['Erro ' num2str(ret)]);
            end
        end
        
        
        
        speedR = -0.0001;
        sim.simxSetJointTargetVelocity(clientID, handleObjects(rob.idRightMotor), speedR, sim.simx_opmode_oneshot);
        speedL = -0.0001;
        sim.simxSetJointTargetVelocity(clientID, handleObjects(rob.idLeftMotor), speedL, sim.simx_opmode_oneshot);
        
        sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot);
        while sim.simxGetConnectionId(clientID) ~= -1 && speedR ~= 0
            %movement control
            [~, targetPosition] = sim.simxGetObjectPosition(clientID, handleObjects(rob.idTarget), -1, sim.simx_opmode_buffer);
            [~, targetOrientation] = sim.simxGetObjectOrientation(clientID, handleObjects(rob.idTarget), -1, sim.simx_opmode_buffer);

            [~, myPosition] = sim.simxGetObjectPosition(clientID, handleObjects(rob.idRobot), -1, sim.simx_opmode_buffer);
            [~, myOrientation] = sim.simxGetObjectOrientation(clientID, handleObjects(rob.idRobot), -1, sim.simx_opmode_buffer);
            
            rho = hypot(targetPosition(1) - myPosition(1), targetPosition(2) - myPosition(2));
            v = rob.K_rho * rho;

            theta = myOrientation(3);
            alpha = atan2(targetPosition(2) - myPosition(2), targetPosition(1) - myPosition(1)) - theta;
            beta = targetOrientation(3) - theta - alpha;
            
            temp = Inf;
            if rad2deg(alpha) > 90 || rad2deg(alpha) < -90 %está de costas
                temp = abs(alpha) - pi;
                if alpha > 0
                    alpha = -temp;
                end
                
                beta = - targetOrientation(3) + theta + alpha;
            end
            
            omega = rob.K_alpha * alpha + rob.K_beta * beta;
            
            speedR = (v + 2 * rob.L * omega)/rob.raio;
            speedL = (v - 2 * rob.L * omega)/rob.raio;

            if temp ~= Inf
                speedR = -speedR;
                speedL = -speedL;
            end
            
            if abs(rho) < rob.L
                if abs(rho) > 0.05
                    speedR = speedR * rob.K_zeta;
                    speedL = speedL * rob.K_zeta;
                else
                    speedR = 0;
                    speedL = 0;
                end
            end
            
            if abs(speedR) < rob.speedLimit || abs(speedL) < rob.speedLimit
                sim.simxSetJointTargetVelocity(clientID, handleObjects(rob.idRightMotor), speedR, sim.simx_opmode_oneshot);                
                sim.simxSetJointTargetVelocity(clientID, handleObjects(rob.idLeftMotor), speedL, sim.simx_opmode_oneshot);
            else
                disp('Erro: Ultrapassou limite de velocidade!')
            end
            
            %vision
            [ret, arr, img] = sim.simxGetVisionSensorImage2(clientID, handleObjects(6), 0, sim.simx_opmode_buffer);
            imshow(img)
            
            %markov
            %criar matriz de espaços
            mySupposition = zeros(10, 10, 12);
            

        end
    else
        disp('Falha ao conectar ao servidor');
    end
    
    speedR = 0;
    speedL = 0;
    sim.simxSetJointTargetVelocity(clientID, handleObjects(rob.idLeftMotor), speedR, sim.simx_opmode_oneshot);
    sim.simxSetJointTargetVelocity(clientID, handleObjects(rob.idRightMotor), speedL, sim.simx_opmode_oneshot);

    sim.simxPauseSimulation(clientID, sim.simx_opmode_oneshot); %pause
    sim.simxGetPingTime(clientID);  %garantir que todos os comandos foram executados
    sim.simxFinish(clientID);       %fechar conexão
    sim.delete();                   %destrutor
    disp('Programa finalizado');    %gui
end