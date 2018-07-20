function [goalState] = GoalSelection(inputing)
    % Complex conjugate transpose('). 
    % Transpose(.').
    input2 = reshape(inputing.', [], 9);
    % Delete elements equal 0:
    input3 = input2(input2 ~= 0);
    % Just to remind myself that size(input3, 2) = 8.
    count = 0;
    for i = 1 : size(input3, 2)
        for j = i + 1 : size(input3, 2)
            if input3(1, i) > input3(1, j)
                count = count + 1;
            end
        end
    end
    % Decide the goal state.
    if mod(count, 2)
        % 1 odd
        goalState = [1 2 3; 8 0 4; 7 6 5];
    else
        % 0 even
        goalState = [0 1 2; 3 4 5; 6 7 8];
    end
end
