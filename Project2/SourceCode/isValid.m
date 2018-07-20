function [validation] = isValid(position)
    % Define outter boundary and the first obstacle together.
    % Note: the direction decide the area is outside or inside the rectangle.
    Obs1X = [0  250  250    0   0   NaN    55      55       105    105    55];
    Obs1Y = [0  0    150    250 0   NaN    67.5    112.5    112.5  67.5  67.5];
    posX = position(1);
    posY = position(2);
    % Regard the points on the edge of obstacles as obstacle positions.
    [in1, on1] = inpolygon(posX, posY, Obs1X, Obs1Y);
    if in1 == 0 
        validation = false;
    elseif in1 == 1 && on1 == 1
        validation = false;
    elseif in1 == 1 && on1 == 0
        % Define the second obstacle.
        Obs2X = [120    158 165 188 168 145 120];
        Obs2Y = [55     51  89  51  14  14  55];   
        in2 = inpolygon(posX, posY, Obs2X, Obs2Y);
        if in2 == 1
            validation = false;
        else
            % Define the circular obstacle.
            rr = (posX - 180)^2 + (posY - 120)^2;
            if rr > 15^2
                validation = true;
            else
                validation = false;
            end
%             Abandoned because cannot detect every points on the edge.
%             t = linspace(0, 2 * pi, 100);
%             Obs2X = 15 * cos(t) + 180;
%             Obs2Y = 15 * sin(t) + 120;
%             in3 = inpolygon(posX, posY, Obs2X, Obs2Y);
%             if in3 == 1
%                 validation = false;
%             else
%                 validation = true;
%             end
        end
    end
end