classdef robot
     properties (Constant)
         idRobot                = 1;
         idLeftMotor            = 2;
         idRightMotor           = 3;
         idTarget               = 4;
         idvelodyneHDL_64E_S2   = 5;
         idVisionSensorL        = 6;
         idVisionSensorR        = 7;
         idSensorDistancia      = 8;
         speedLimitMin          = 85;
         speedLimit             = 220;
         K_rho                  = 0.4;%1.2
         K_alpha                = 0.5;%1.6
         K_beta                 = -0.10;%-0.30
         K_zeta                 = 10;%10
         raio                   = 0.02;
         L                      = 0.13;
     end
end