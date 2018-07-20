function [heuristicCost] = CostFunction(inputing, goalState)
    % Manhattan distance:
    heuristicCost = 0;
    for i = 1 : 3
        for j = 1 : 3
            tmp = inputing(i, j);
            if tmp > 0
                [row , col] = find(goalState == tmp);
                heuristicCost = heuristicCost + abs(i - row) + abs(j - col);
            end
        end
    end
end