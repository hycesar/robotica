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
        
        [handleObjects] = initHandles(clientID, sim);
        
        %initMotors
        setSpeed(clientID, sim, handleObjects, 0, 0);
        
        %markov - distribuição de crença inicial
        mySupPositionSize = [10, 10, 12];
        mySupPosition = ones(mySupPositionSize(1), mySupPositionSize(2), mySupPositionSize(3))/(mySupPositionSize(1)*mySupPositionSize(2)*mySupPositionSize(3)); %matriz de espaços
        mySupMovement = zeros(mySupPositionSize(1), mySupPositionSize(2), mySupPositionSize(3));
        
        %detect the enviroment
        [~, targetPosition] = sim.simxGetObjectPosition(clientID, handleObjects(rob.idTarget), -1, sim.simx_opmode_buffer);
        [~, targetOrientation] = sim.simxGetObjectOrientation(clientID, handleObjects(rob.idTarget), -1, sim.simx_opmode_buffer);
        [~, myPosition] = sim.simxGetObjectPosition(clientID, handleObjects(rob.idRobot), -1, sim.simx_opmode_buffer);
        [~, myOrientation] = sim.simxGetObjectOrientation(clientID, handleObjects(rob.idRobot), -1, sim.simx_opmode_buffer);
        [~ , myLinearVelocity, myAngularVelocity] = sim.simxGetObjectVelocity(clientID, handleObjects(3), sim.simx_opmode_buffer);

        sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot);
        while sim.simxGetConnectionId(clientID) ~= -1 && ~arrived(targetPosition, targetOrientation, myPosition, myOrientation)
            %[speedR, speedL] = controlMovement(targetPosition,targetOrientation,myPosition,myOrientation);
            
            % girar 360 graus
            %for i = 1: 12
            %    rot30(clientID, sim, handleObjects);
            %    pause(2)
                %atualizar markov
            %end
            
            %runHalfMeter(clientID, sim, handleObjects);
            
            pause(10);
            
            %detectar possibilidade de andar para frente - verificar sensor
            %sim? andar não? gira 90 graus (exemplo)
            
            %anda espaço determinado para frente
            %setSpeed(clientID, sim, handleObjects, 11.65, -11.65)

            %detectar movemento
            %[~ , myLinearVelocity, myAngularVelocity] = sim.simxGetObjectVelocity(clientID, handleObjects(3), sim.simx_opmode_buffer);
            
            %aguardar movemento
            %pause(1);
            
            %parar o carro
            %setSpeed(clientID, sim, handleObjects, 0, 0)
            %pause(2);
            %detectar sensor frontal não bati? continua! se não ->exceção
            
            %

            % fazer minha suposição
            %disp('Acho que andei 0.50 m');
            
            % atualizar minha suposição
            
            
            
            %vision
            %[ret, arr, img] = sim.simxGetVisionSensorImage2(clientID, handleObjects(6), 0, sim.simx_opmode_buffer);
            %imshow(img)
            
            %markov
            %for i = 1:mySupPositionSize(1)
            %    for j = 1:mySupPositionSize(2)
            %        for k = 1:mySupPositionSize(3)
            %            mySupPosition(i,j,k) = mySupPosition(i,j,k);
            %        end
            %    end
            %end
            
            %check my params
            [~, targetPosition] = sim.simxGetObjectPosition(clientID, handleObjects(rob.idTarget), -1, sim.simx_opmode_buffer);
            [~, targetOrientation] = sim.simxGetObjectOrientation(clientID, handleObjects(rob.idTarget), -1, sim.simx_opmode_buffer);
            [~, myPosition] = sim.simxGetObjectPosition(clientID, handleObjects(rob.idRobot), -1, sim.simx_opmode_buffer);
            [~, myOrientation] = sim.simxGetObjectOrientation(clientID, handleObjects(rob.idRobot), -1, sim.simx_opmode_buffer);
            [~ , myLinearVelocity, myAngularVelocity] = sim.simxGetObjectVelocity(clientID, handleObjects(3), sim.simx_opmode_buffer);

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