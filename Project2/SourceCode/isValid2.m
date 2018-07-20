function [validation] = isValid2(position)
    posX = position(1);
    posY = position(2);
    x = [120 158 165 188 168 145];
    y = [55 51 89 51 14 14];
    % Define the polygon obstacle.
    A = (y(1)-y(3))/(x(1)-x(3))*(posX-x(3))+y(3) >= posY;
    B = (y(3)-y(4))/(x(3)-x(4))*(posX-x(3))+y(3) >= posY;
    C = (y(5)-y(4))/(x(5)-x(4))*(posX-x(4))+y(4) <= posY;
    D = 14 <= posY;
    E = (y(6)-y(1))/(x(6)-x(1))*(posX-x(6))+y(6) <= posY;
    F = A&&B&&C&&D&&E;

    G = (y(2)-y(3))/(x(2)-x(3))*(posX-x(3))+y(3) < posY;
    H = (y(2)-y(1))/(x(2)-x(1))*(posX-x(1))+y(1) < posY;
    I = A&&G&&H;

    J = F&&~I;
    % Define the free space.
    % Regard the points on the edge of obstacles as obstacle positions.
    if posX <= 0 || posX >= 250 || posY <= 0 || posY >= 150 
        validation = false;
    % Define the rectangular obstacle.
    elseif (55 <= posX) && (posX <= 105) && (posY <= 112.5) && (67.5 <= posY) 
        validation = false;
    % Define the circular obstacle.
    elseif (posX - 180)^2 + (posY - 120)^2 <= 15^2
        validation = false;
    % Define the polygon obstacle.
    elseif J
        validation = false;
    else
        validation = true;
    end
end