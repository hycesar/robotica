/**
 * Controle do Robô de direção diferencial
 * Disciplina de Robótica CIn/UFPE
 * 
 * @autor Prof. Hansenclever Bassani
 * 
 * Este código é proporcionado para facilitar os passos iniciais da programação.
 * 
 * Testado em: Pop_OS 20.04
 * 
 */


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <unistd.h>

#define COPPELIA_SIM_IP_ADDRESS /*"10.0.2.2"*/"127.0.0.1"
#define COPPELIA_SIM_PORT 19997//1999;

#define RAIO 0.02
#define L 0.1

extern "C" {
#include "extApi.h"
    /*	#include "extApiCustom.h" if you wanna use custom remote API functions! */
}

simxInt ddRobotHandle;
simxInt leftMotorHandle;
simxInt rightMotorHandle;
simxInt sensorHandle;
simxInt targetHandle;

void getPosition(int clientID, simxInt objectHandle, simxFloat pos[]) { //[x,y,theta]

    simxInt ret = simxGetObjectPosition(clientID, objectHandle, -1, pos, simx_opmode_oneshot_wait);
    if (ret > 0) {
        printf("Error reading object position\n");
        return;
    }

    simxFloat orientation[3];
    ret = simxGetObjectOrientation(clientID, objectHandle, -1, orientation, simx_opmode_oneshot_wait);
    if (ret > 0) {
        printf("Error reading object orientation\n");
        return;
    }

    simxFloat theta = orientation[2];
    pos[2] = theta;
}

simxInt getSimTimeMs(int clientID) { //In Miliseconds
    return simxGetLastCmdTime(clientID);
}

void setTargetSpeed(int clientID, simxFloat phiL, simxFloat phiR) {
    simxSetJointTargetVelocity(clientID, leftMotorHandle, phiL, simx_opmode_oneshot);
    simxSetJointTargetVelocity(clientID, rightMotorHandle, phiR, simx_opmode_oneshot);   
}

inline double to_deg(double radians) {
    return radians * (180.0 / M_PI);
}

void controle_movimento(simxFloat pos[3], simxFloat goal[3], simxFloat* PhiL, simxFloat* PhiR)
{    
    //p[3] = 0°/180° antihorário 12h to 6h sinal positivo
    //180°/0° antihorário 6h to 12h sinal negativo

    //valores iniciais de ks
    double Kp = 2, Ka = 4, Kb = -1;

    //Usar o sistema de coordenadas com origem centrado no objetivo
    double ro, al, bt, dX, dY, th, at2, pos2, gol2;
    
    //velocidades calculadas
    double v, w;
    
    //ro: distância
    dX = (double) goal[0] - (double) pos[0];
    dY = (double) goal[1] - (double) pos[1];
    ro = sqrt(pow(dX, 2) + pow(dY, 2));
    
    if(pos[2] < 0){
        pos2 = (double) pos[2] * M_PI;
    }
    if(goal[2] < 0){
        gol2 = (double) goal[2] * M_PI;
    }

    //tetha
    if (pos[2] - goal[2] >= -M_PI && pos[2] <= M_PI_2 ){
        th = (double) pos[2];// + M_PI_2;
    }else{
        th = (double) pos[2];// - (M_PI_2 * 3.0);
    }

    //alpha e beta
    al = -th + atan2(dY, dX);
    if(abs(al) > M_PI){
        al = al - M_PI * 2.0;
        printf("oi");
    }
    bt = -th - al;

    printf("[nO:%.4f dX:%.4f dy:%.4f ro:%.4f al:%.4f° bt:%.4f°]", to_deg(pos[2] - M_PI_2), dX, dY, ro, al, bt);

    //velocidade linear
    v = Kp * ro;
    //velocidade angular
    w = (Ka * al) + (Kb * bt);

    //determinar velocidade de cada roda
    //Para trás
    if (al <= (-M_PI_2) || al >= (M_PI_2)){
        v = -v;
        //verificare
        //w = -w;
    }

    *PhiR = (v + ((L * w) / 2.0)) * 5.0;
    *PhiL = (v - ((L * w) / 2.0)) * 5.0;
    printf("V e w:[%.2f %.2f]", v, w);
    
    if(abs(ro)<0.03 && abs(al)<1 && abs(bt)<1){
        printf("\nChegou o objetivo...\n");
        *PhiR = 0;
        *PhiL = 0;
        pause();
    }
}

int main(int argc, char* argv[]) {

    std::string ipAddr = COPPELIA_SIM_IP_ADDRESS;
    int portNb = COPPELIA_SIM_PORT;

    if (argc > 1) {
        ipAddr = argv[1];
    }

    printf("Iniciando conexao com: %s...\n", ipAddr.c_str());

    int clientID = simxStart((simxChar*) (simxChar*) ipAddr.c_str(), portNb, true, true, 2000, 5);
    if (clientID != -1) {
        printf("Conexao efetuada\n");
        
        //Get handles for robot parts, actuators and sensores:
        simxGetObjectHandle(clientID, "RobotFrame#", &ddRobotHandle, simx_opmode_oneshot_wait);
        simxGetObjectHandle(clientID, "LeftMotor#", &leftMotorHandle, simx_opmode_oneshot_wait);
        simxGetObjectHandle(clientID, "RightMotor#", &rightMotorHandle, simx_opmode_oneshot_wait);
        simxGetObjectHandle(clientID, "Target#", &targetHandle, simx_opmode_oneshot_wait);
        
        printf("RobotFrame: %d\n", ddRobotHandle);
        printf("LeftMotor: %d\n", leftMotorHandle);
        printf("RightMotor: %d\n", rightMotorHandle);

        //start simulation
        int ret = simxStartSimulation(clientID, simx_opmode_oneshot_wait);
        
        if (ret==-1) {
            printf("Não foi possível iniciar a simulação.\n");
            return -1;
        }
        
        printf("Simulação iniciada.\n");

        //While is connected:
        while (simxGetConnectionId(clientID) != -1) {
        
            //Read current position:
            simxFloat pos[3]; //[x,y,theta] in [cm cm rad]
            getPosition(clientID, ddRobotHandle, pos);

            //Read simulation time of the last command:
            simxInt time = getSimTimeMs(clientID); //Simulation time in ms or 0 if sim is not running
            //stop the loop if simulation is has been stopped:
            if (time == 0) break;             
            printf("Posicao: [%.2f %.2f %.2f°] ", pos[0], pos[1], to_deg(pos[2]));
            
            //Set new target speeds: robot going in a circle:
            simxFloat goal[3]; 
            getPosition(clientID, targetHandle, goal);
            printf("Objetivo: [%.2f %.2f %.2f°]", goal[0], goal[1], to_deg(goal[2]));
            
            simxFloat phiL, phiR; //rad/s
            controle_movimento(pos, goal, &phiL, &phiR);
            
            setTargetSpeed(clientID, phiL, phiR);

            //Leave some time for CoppeliaSim do it's work:
            extApi_sleepMs(1);
            printf("\n");
        }
        
        //Stop the robot and disconnect from CoppeliaSim;
        setTargetSpeed(clientID, 0, 0);
        simxPauseSimulation(clientID, simx_opmode_oneshot_wait);
        simxFinish(clientID);
        
    } else {
        printf("Nao foi possivel conectar.\n");
        return -2;
    }
    
    return 0;
}


