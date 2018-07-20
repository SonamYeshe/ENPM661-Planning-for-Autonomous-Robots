% struct m*n means each field has m types values(something we have noe access using struct(m)), it has n groups of values.
% struct(a) is the access to the #a group.
% struct(a).fieldName is the access to a specific field in group #a.
% {'a', 'b'} -- 1x2 cell array; {'a'; 'b'} -- 2x1 cell array.
%% 
function [  ] = PathPlanning(inputing,goalState)
	S.node = inputing;
	S.cost = 0;
	S.costg = 0;
    S.nodeId = 0;
	S.parentId = 0;
    
	histList = S;
    openList = S;
    list = S;    
	bestNodes = S;
	bestNodes(1).parentId = -1;
    
    lastOpen = 1;
	x = 0;
	nBest = 2;
	lastHist = 2;
    goalFound=0;
    id = 0;
	loops = 0;
    
	while ~goalFound && loops < 2000
		costh = CostFunction(S.node, goalState);
        if costh
            % Find blank tile position
            [row, col] = find(S.node == 0);
            % Blank move up
            if row > 1
                x = x + 1;
                list(x).node = S.node;
                list(x).node (row, col) = S.node (row - 1, col);
                list(x).node (row - 1, col) = 0;
                costh = CostFunction (list(x).node, goalState);
                list(x).cost = costh + S.costg + 1;
                list(x).costg = S.costg;
 				list(x).nodeId = 0;	% dummy
				list(x).parentId = S.nodeId;	% dummy
            end
            % Blank move down
            if row < 3
                x = x + 1;
                list(x).node = S.node;
                list(x).node (row, col) = S.node (row + 1, col);
                list(x).node (row + 1, col) = 0;
                costh = CostFunction (list(x).node, goalState);
                list(x).cost = costh + S.costg + 1;
                list(x).costg = S.costg;
 				list(x).nodeId = 0;	% dummy
				list(x).parentId = S.nodeId;	% dummy
			end
            % Blank move left
            if col > 1
                x = x + 1;
                list(x).node = S.node;
                list(x).node (row, col) =  S.node (row, col - 1);
                list(x).node (row, col - 1) = 0;
                costh = CostFunction (list(x).node, goalState);
                list(x).cost = costh + S.costg + 1;
                list(x).costg = S.costg;
 				list(x).nodeId = 0;	% dummy
				list(x).parentId = S.nodeId;	% dummy
            end
            % Blank move right
            if col < 3
                x = x + 1;
                list(x).node = S.node;
                list(x).node (row, col) =  S.node (row, col + 1);
                list(x).node (row, col + 1) = 0;
                costh = CostFunction (list(x).node , goalState);
                list(x).cost = costh + S.costg + 1;
                list(x).costg = S.costg;
 				list(x).nodeId = 0;	% dummy
				list(x).parentId = S.nodeId;	% dummy
            end
			
            % If the child node is in history, delete
            [~, y] = size(histList);
            temp = x;
            while y > 0
                x = temp;
                while x > 0
                    [~, ~, v] = find(histList(y).node == list(x).node);
                    if sum(v) == 9
                        list(x) = [];
                        temp = temp-1;
                    end
                    x = x - 1;
                end
                y = y - 1;
            end
            
            % Set node ID 
            [~, a] = size(list);
            t = 1;
			while t <= a
                id = id + 1;
                list(t).nodeId = id;
                t = t + 1;
			end
			
			[~, y] = size(list);
            % Add child list to open list
            openList(1, lastOpen : lastOpen + y - 1) = list;
            lastOpen = lastOpen + y;
            [~, y] = size(openList);
            % Find smaller cost
            smallNode = y;
            while y > 1    
                if openList(smallNode).cost >= openList(y - 1).cost
                    smallNode = y - 1;
                end
                y = y - 1;
            end

            % Next state node
            S = openList(smallNode);
			S.costg = S.costg + 1;
            
            % Include new child list in history list
            [~, y] = size(list);
            histList(1, lastHist : lastHist + y - 1 ) = list;
            lastHist = lastHist + y;

            % Include best node in closed list
            bestNodes(nBest) = openList(smallNode);
            nBest = nBest + 1;
            list = [];
			
            % Take off small node from openList
            openList(smallNode) = [];
            lastOpen = lastOpen - 1;
			loops = loops + 1;
        else
			% Print sequence
            goalFound = 1;
			nBest = nBest - 1;
            X = sprintf('node #%d \n', nBest);
            fprintf(X);
			disp(bestNodes(nBest).node);
			parent = bestNodes(nBest).parentId;
			while parent >= 0
				v = [];
				while isempty(v) && (parent >= 0)
					[~, ~, v] = find(bestNodes(nBest).nodeId == parent);
					nBest = nBest - 1; 
				end
				nBest = nBest + 1;
                X = sprintf('node #%d:  \n', nBest);
                fprintf(X);
				disp(bestNodes(nBest).node);
				parent = bestNodes(nBest).parentId;
            end
            X = sprintf('Total loops are %d. \n', loops);
			fprintf(X);
		end
	end
end