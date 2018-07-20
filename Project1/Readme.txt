## 8 Puzzle Game

# General

This program has 4 Matlabs files and no specific output stored in the 'Output' repository. 
All the outputs depends on your input matrix and they'll be shown in the Matlab Command Window.

# Source Code Explanation
1. 'StartGame': The main function, callback other 3 functions.
2. 'CostFunction': The heuristic cost using Manhattan distance.
3. 'GoalSelection': Count the number of inversions to determine the goalState is [1 2 3; 8 0 4; 7 6 5] or [0 1 2; 3 4 5; 6 7 8].
4. 'PathPlanning': Implementing path planning alrorithm - A*. Prioritizing the nodes in the openList with minimal f(n) = g(n) + h(n), while g(n) is the cost of the path from the start node to n, and h(n) is a heuristic that estimates the cost of the cheapest path from n to the goal. And trace back from the goalState to get the optimal path.

# Run
1. cd path to folder SourceCode.
2. Open 'StartGame' file, press F5, type in any start matrix.
3. See the step-by-step movement shown in the Command Window in the reverse order.