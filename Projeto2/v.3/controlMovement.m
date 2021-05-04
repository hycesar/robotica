function [speedR, speedL] = controlMovement(targetPosition, targetOrientation, myPosition, myOrientation)
%   Controle de Movimento
%   Coppelia via Matlab
%   movement control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    rob = robot();

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
end

