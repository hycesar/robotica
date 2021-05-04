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
        disp('Conectado com sucesso ao servidor');
        sim.simxAddStatusbarMessage(clientID, 'Conectado com sucesso ao cliente', sim.simx_opmode_oneshot);
        
        [handleObjects] = initHandles(clientID, sim);
        
        %initMotors
        setSpeed(clientID, sim, handleObjects, 0, 0);
        
        %markov - distribui��o de cren�a inicial
        mySupPositionSize = [10, 10, 12];
        mySupPosition = ones(mySupPositionSize(1), mySupPositionSize(2), mySupPositionSize(3))/(mySupPositionSize(1)*mySupPositionSize(2)*mySupPositionSize(3)); %matriz de espa�os
        xMySupMovement = 1;
        yMySupMovement = 1;
        thMySupMovement = 1;
        myBeliefs = zeros(mySupPositionSize(1), mySupPositionSize(2), mySupPositionSize(3));
        map = zeros(mySupPositionSize(1), mySupPositionSize(2), mySupPositionSize(3));
        
        %detect the enviroment
        [~, targetPosition] = sim.simxGetObjectPosition(clientID, handleObjects(rob.idTarget), -1, sim.simx_opmode_buffer);
        [~, targetOrientation] = sim.simxGetObjectOrientation(clientID, handleObjects(rob.idTarget), -1, sim.simx_opmode_buffer);
        [~, myPosition] = sim.simxGetObjectPosition(clientID, handleObjects(rob.idRobot), -1, sim.simx_opmode_buffer);
        [~, myOrientation] = sim.simxGetObjectOrientation(clientID, handleObjects(rob.idRobot), -1, sim.simx_opmode_buffer);
        [~ , myLinearVelocity, myAngularVelocity] = sim.simxGetObjectVelocity(clientID, handleObjects(3), sim.simx_opmode_buffer);

        sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot);
        while sim.simxGetConnectionId(clientID) ~= -1 && ~arrived(targetPosition, targetOrientation, myPosition, myOrientation)
            % Se o n�vel de confian�a chegar um limiar, assumir minha
            % posi��o e orienta��o e rodar o controle de movimento
            %[speedR, speedL] = controlMovement(targetPosition,targetOrientation,myPosition,myOrientation);
            
            % Fazer 10 leituras iniciais no sensor
            obst = 0;
            rotation = 0;
            while ~obst
                i = 1;
                obstacle = 0;
                distanceObstacle = 0;
                while i < 10 && ~obstacle
                    [~, obstacle, distanceObstacle, ~, ~] = sim.simxReadProximitySensor(clientID, handleObjects(rob.idSensorDistancia), sim.simx_opmode_buffer);
                    i = i + 1;
                end

                % Foi encontrado obst�culo?
                if obstacle && distanceObstacle(3) < 0.8
                    disp('Obst�culo a frente detectado');
                    trouble = rot30(clientID, sim, handleObjects);
                    thMySupMovement = mod(thMySupMovement, 12) + 1;
                else
                    obst = 1;
                    disp('N�o foi encontrado obst�culo a frente');
                    trouble = runHalfMeter(clientID, sim, handleObjects);
                end
                pause(1);
            end
            
            %preparar Matriz
            mySupMovement = zeros(mySupPositionSize(1), mySupPositionSize(2), mySupPositionSize(3));
            
            %suposi��es odometria
            if obstacle %houve rota��o
                if trouble % encontrou outro obst�culo supor 100% 0h
                    mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 1;
                else %sem obst�culos - supor 50% 1h, 2h
                    mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.5;
                    mySupMovement(yMySupMovement, xMySupMovement, mod(thMySupMovement + 1, 12) + 1) = 0.5;
                end
            else %houve avan�o
                if thMySupMovement == 1
                    if trouble %encontrou obst�culo imprevisto - supor 100% 1c�lula
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 1;
                    else %dividir suposi��o 50% 1c�lula 50% 2c�lulas
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.5;
                        xMySupMovement = mod(xMySupMovement,  mySupPositionSize(3)) + 1;
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.5;
                    end
                elseif thMySupMovement == 2
                    if trouble %encontrou obst�culo imprevisto - supor 100% 1c�lula
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 1;
                    else %dividir suposi��o 50% 1c�lula 50% 2c�lulas
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.35;
                        xMySupMovement = mod(xMySupMovement,  mySupPositionSize(3)) + 1;
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.35;

                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.15;
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.15;
                    end
                elseif thMySupMovement == 3
                    if trouble %encontrou obst�culo imprevisto - supor 100% 1c�lula
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 1;
                    else %dividir suposi��o 50% 1c�lula 50% 2c�lulas
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.15;
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.15;

                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.35;
                        yMySupMovement = mod(yMySupMovement,  mySupPositionSize(3)) + 1;
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.35;
                    end
                elseif thMySupMovement == 4
                    if trouble %encontrou obst�culo imprevisto - supor 100% 1c�lula
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 1;
                    else %dividir suposi��o 50% 1c�lula 50% 2c�lulas
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.5;
                        yMySupMovement = mod(yMySupMovement,  mySupPositionSize(3)) + 1;
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.5;
                    end
                elseif thMySupMovement == 5
                    if trouble %encontrou obst�culo imprevisto - supor 100% 1c�lula
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 1;
                    else %dividir suposi��o 50% 1c�lula 50% 2c�lulas
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.35;
                        yMySupMovement = mod(yMySupMovement,  mySupPositionSize(3)) + 1;
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.35;

                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.15;
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.15;
                    end
                elseif thMySupMovement == 6
                    if trouble %encontrou obst�culo imprevisto - supor 100% 1c�lula
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 1;
                    else %dividir suposi��o 50% 1c�lula 50% 2c�lulas
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.15;
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.15;

                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.35;
                        xMySupMovement = mod(xMySupMovement,  mySupPositionSize(3)) - 1;
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.35;
                    end
                elseif thMySupMovement == 7
                    if trouble % encontrou outro obst�culo supor 100% 0h
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 1;
                    else %sem obst�culos - supor 50% 1h, 2h
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.5;
                        xMySupMovement = mod(xMySupMovement,  mySupPositionSize(3)) - 1;
                        mySupMovement(yMySupMovement, xMySupMovement, mod(thMySupMovement + 1, 12) + 1) = 0.5;
                    end
                elseif thMySupMovement == 8
                    if trouble %encontrou obst�culo imprevisto - supor 100% 1c�lula
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 1;
                    else %dividir suposi��o 50% 1c�lula 50% 2c�lulas
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.15;
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.15;

                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.35;
                        xMySupMovement = mod(xMySupMovement,  mySupPositionSize(3)) - 1;
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.35;
                    end
                elseif thMySupMovement == 9
                    if trouble %encontrou obst�culo imprevisto - supor 100% 1c�lula
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 1;
                    else %dividir suposi��o 50% 1c�lula 50% 2c�lulas
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.15;
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.15;

                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.35;
                        yMySupMovement = mod(yMySupMovement,  mySupPositionSize(3)) - 1;
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.35;
                    end
                elseif thMySupMovement == 10
                    if trouble %encontrou obst�culo imprevisto - supor 100% 1c�lula
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 1;
                    else %dividir suposi��o 50% 1c�lula 50% 2c�lulas
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.15;
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.15;

                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.35;
                        yMySupMovement = mod(yMySupMovement,  mySupPositionSize(3)) - 1;
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.35;
                    end
                elseif thMySupMovement == 11
                    if trouble %encontrou obst�culo imprevisto - supor 100% 1c�lula
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 1;
                    else %dividir suposi��o 50% 1c�lula 50% 2c�lulas
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.15;
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.15;

                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.35;
                        xMySupMovement = mod(xMySupMovement,  mySupPositionSize(3)) - 1;
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.35;
                    end
                elseif thMySupMovement == 12
                    if trouble %encontrou obst�culo imprevisto - supor 100% 1c�lula
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 1;
                    else %dividir suposi��o 50% 1c�lula 50% 2c�lulas
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.15;
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.15;

                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.35;
                        xMySupMovement = mod(xMySupMovement,  mySupPositionSize(3)) + 1;
                        mySupMovement(yMySupMovement, xMySupMovement, thMySupMovement) = 0.35;
                    end
                else
                    disp('erro');
                end
            end

            % atualizar minha suposi��o
            
            
            % vision
            % [ret, arr, img] = sim.simxGetVisionSensorImage2(clientID, handleObjects(6), 0, sim.simx_opmode_buffer);
            % imshow(img)
            
            %markov - Action Update
            myBeliefs = markovActUpd3D(mySupPosition, mySupMovement, myBeliefs);
            
            %preparar matriz
            mySupSensor = zeros(mySupPositionSize(1), mySupPositionSize(2), mySupPositionSize(3));
            
            %verificar sensores
            for i = 1:12
                [~, obstacle, distanceObstacle, ~, ~] = sim.simxReadProximitySensor(clientID, handleObjects(rob.idSensorDistancia), sim.simx_opmode_buffer);
                while i < 50 && ~obstacle
                    [~, obstacle, distanceObstacle, ~, ~] = sim.simxReadProximitySensor(clientID, handleObjects(rob.idSensorDistancia), sim.simx_opmode_buffer);
                    i = i + 1;
                end
                disp('Distancia do obst�culo:');
                disp(distanceObstacle);
                if obstacle
                    if distanceObstacle(3) < 0.5 %75% em 1 e 25% em 2
                        mySupSensor(1,1,i) = 0.75; 
                        mySupSensor(1,2,i) = 0.25;
                    elseif distanceObstacle(3) < 1 %25% em 1 50% em 2 25% em 3
                        mySupSensor(1,1,i) = 0.25; 
                        mySupSensor(1,2,i) = 0.50;
                        mySupSensor(1,3,i) = 0.25; 
                    elseif distanceObstacle(3) < 1.5 %25% em 2 75% em 3
                        mySupSensor(1,2,i) = 0.25;
                        mySupSensor(1,3,i) = 0.75;
                    else
                        disp('Inconsist�ncia de dados: obst�culo sem dist�ncia?');
                    end
                end
                rot30(clientID, sim, handleObjects);
            end
            
            %markov - Perception Update
            map = markovPerUpd3D(myBeliefs, mySupSensor, map);
            
            %showMap
            imshow(sqrt(sum(map.^2,3)))
            
            %check my params
            %[~, targetPosition] = sim.simxGetObjectPosition(clientID, handleObjects(rob.idTarget), -1, sim.simx_opmode_buffer);
            %[~, targetOrientation] = sim.simxGetObjectOrientation(clientID, handleObjects(rob.idTarget), -1, sim.simx_opmode_buffer);
            %[~, myPosition] = sim.simxGetObjectPosition(clientID, handleObjects(rob.idRobot), -1, sim.simx_opmode_buffer);
            %[~, myOrientation] = sim.simxGetObjectOrientation(clientID, handleObjects(rob.idRobot), -1, sim.simx_opmode_buffer);
            %[~ , myLinearVelocity, myAngularVelocity] = sim.simxGetObjectVelocity(clientID, handleObjects(3), sim.simx_opmode_buffer);

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
    sim.simxFinish(clientID);       %fechar conex�o
    sim.delete();                   %destrutor
    disp('Programa finalizado');    %gui
end