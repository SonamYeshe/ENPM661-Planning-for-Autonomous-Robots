clc
clear all

%% Plotting the free space and obstacles.
figure, hold on;
A = rectangle('Position', [0 0 250 150], 'EdgeColor', 'b', 'LineWidth', 2 , 'FaceColor', [0 0 1 0.2]);
Obs1 = rectangle('Position', [55 67.5 50 45], 'EdgeColor', 'b','LineWidth', 2, 'FaceColor', [0 0 1 0.4]);
Obs2 = rectangle('Position', [165 105 30 30], 'EdgeColor', 'b','LineWidth', 2, 'FaceColor', [0 0 1 0.4], 'Curvature', [1 1]);
Obs3X = [120, 158, 165, 188, 168, 145];
Obs3Y = [55, 51, 89, 51, 14, 14];
Obs3 = patch(Obs3X, Obs3Y, 'r', 'EdgeColor', 'b', 'LineWidth', 2, 'FaceColor', [0 0 1], 'FaceAlpha', 0.4);
axis equal;

%% Give start and goal points and validates them.
startPosition = [145, 13];
goalPosition = [185, 57];
if ~isValid(startPosition) || ~isValid(goalPosition)
    fprintf('Start and final position should be in the free space!!!');
elseif isequal(startPosition, goalPosition)
    fprintf('Start and final position should be different!!!');
else
    % Initiate the node struct.
    S.node = startPosition;
    S.nodeId = 1;
    S.parentId = 0;
    q = Queue;
    q.enqueue(1);
    openList = S;
    closedList = S;
    tmplist = S;
    list = S;
    % Neighbor points order.
    x = 0;
    % All points Id.
    y = 1;
    % closedList order.
    z = 1;
    goalFound = 0;

    %% Main algorithm.
    while ~goalFound
        % Breadth First Search(BFS).
        openListOrderNow = q.dequeue.data;
        for i = 1 : length(list)
            if list(i).nodeId == openListOrderNow
                openListNow = list(i);
            end
        end
        if isValid(openListNow.node)
            % Expanding neighbor, clockwise, from upper left. & Validate new points.
            % No.1
            x = x + 1;
            tmplist(x).node = [openListNow.node(1) - 1, openListNow.node(2) + 1];
            if isValid(tmplist(x).node)
                tmplist(x).nodeId = 0; 	% dummy
                tmplist(x).parentId = openListNow.nodeId;
            else
                x = x - 1;
            end
            % No.2
            x = x + 1;
            tmplist(x).node = [openListNow.node(1), openListNow.node(2) + 1];
            if isValid(tmplist(x).node)
                tmplist(x).nodeId = 0; 	% dummy
                tmplist(x).parentId = openListNow.nodeId;
            else
                x = x - 1;
            end
            % No.3
            x = x + 1;
            tmplist(x).node = [openListNow.node(1) + 1, openListNow.node(2) + 1];
            if isValid(tmplist(x).node)
                tmplist(x).nodeId = 0; 	% dummy
                tmplist(x).parentId = openListNow.nodeId;
            else
                x = x - 1;
            end
            % No.4
            x = x + 1;
            tmplist(x).node = [openListNow.node(1) + 1, openListNow.node(2)];
            if isValid(tmplist(x).node)
                tmplist(x).nodeId = 0; 	% dummy
                tmplist(x).parentId = openListNow.nodeId;
            else
                x = x - 1;
            end
            % No.5
            x = x + 1;
            tmplist(x).node = [openListNow.node(1) + 1, openListNow.node(2) - 1];
            if isValid(tmplist(x).node)
                tmplist(x).nodeId = 0; 	% dummy
                tmplist(x).parentId = openListNow.nodeId;
            else
                x = x - 1;
            end
            % No.6
            x = x + 1;
            tmplist(x).node = [openListNow.node(1), openListNow.node(2) - 1];
            if isValid(tmplist(x).node)
                tmplist(x).nodeId = 0; 	% dummy
                tmplist(x).parentId = openListNow.nodeId;
            else
                x = x - 1;
            end
            % No.7
            x = x + 1;
            tmplist(x).node = [openListNow.node(1) - 1, openListNow.node(2) - 1];
            if isValid(tmplist(x).node)
                tmplist(x).nodeId = 0; 	% dummy
                tmplist(x).parentId = openListNow.nodeId;
            else
                x = x - 1;
            end
            % No.8
            x = x + 1;
            tmplist(x).node = [openListNow.node(1) - 1, openListNow.node(2)];
            if isValid(tmplist(x).node)
                tmplist(x).nodeId = 0; 	% dummy
                tmplist(x).parentId = openListNow.nodeId;
            else
                tmplist(x) = [];
                x = x - 1;
            end
            % Check if new points are already considered.
            len = length(list);
            for i = 1 : x
                count = 0;
                for j = 1 : len
                    if ~isequal(tmplist(i).node, list(j).node)
                        count = count + 1;
                    end
                end
                if count == len
                    y = y + 1;
                    q.enqueue(y);
                    list(y) = tmplist(i);
                    list(y).nodeId = y;

                    pause(0.001)
                    alpha=list(y).node(1);
                    beta= list(y).node(2);
                    plot (alpha,beta,'s','MarkerSize',2,'MarkerEdgeColor','g','MarkerFaceColor',[0.09 0.95 0.91]);

                    % Check if reach the goal.
                    if isequal(list(y).node, goalPosition)
                        goalFound = 1;
                        break
                    end
                end
            end
            x = 0;
            tmplist = [];      
        end
    end
    %% Find the optimal path.
    pathX = list(y).node(1);
    pathY = list(y).node(2);
    traceBackId = y;
    while traceBackId ~= 1
        traceBackId = list(traceBackId).parentId;
        pathX = cat(1, pathX, list(traceBackId).node(1));
        pathY = cat(1, pathY, list(traceBackId).node(2));
    end
    % reverse the order of the final path.
    pathX = fliplr(pathX.');
    pathY = fliplr(pathY.');
    for i = 1 : length(pathX)
        plot(pathX(i), pathY(i), 's','MarkerSize',3,'MarkerEdgeColor','r','MarkerFaceColor','r');
        pause(0.02)
    end
end
