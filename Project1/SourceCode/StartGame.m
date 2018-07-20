function [] = StartGame( )
    % Solves the puzzle 8 game using A*
	clear all
    clc
	
    inputing = input('Insert matrix [3x3] to be solved: ');
    dimension = size(inputing);
    if not(all(dimension == [3 3]))
        msgbox('The matrix must be 3x3!');

    elseif sum(sum(inputing)) ~= 36
        msgbox('The matrix must contain the values [1 2 3; 4 5 6; 7 8 0]!');
    else
        goalState = GoalSelection(inputing);
		PathPlanning(inputing, goalState);
	end
end