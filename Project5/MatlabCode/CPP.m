%% Implementation of coverage path planning.
global whiteImage
whiteImageCopy = whiteImage;
%% Cell decomposition.
for i = 1 : length(CriticalPoint)
    Pos = CriticalPoint(i, :);
    % only apply to the critical points inside the area rather than the edge.
    if downFree(Pos) && upFree(Pos)
        count = 1;
        while whiteImage(Pos(2) - count, Pos(1)) == 0 || whiteImage(Pos(2) - count, Pos(1)) == 255
            whiteImageCopy(Pos(2) - count, Pos(1)) = 110;
            count = count + 1;
        end
        count = 1;
        while whiteImage(Pos(2) + count, Pos(1)) == 0 || whiteImage(Pos(2) + count, Pos(1)) == 255
            whiteImageCopy(Pos(2) + count, Pos(1)) = 110;
            count = count + 1;
        end
    end
    whiteImageCopy(Pos(2), Pos(1)) = 110;
end
binaryWhiteImageCopy = ~imbinarize(whiteImageCopy, 0.4);% threshold = 0.4*255 = 102
binaryWhiteImageCopy = bwareaopen(binaryWhiteImageCopy, 50);
global labeledWhiteImageCopy
labeledWhiteImageCopy = bwlabel(binaryWhiteImageCopy);

%% Find start & goal positions for each cell.
stats = regionprops(binaryWhiteImageCopy, 'Centroid');
startPos = zeros(length(stats), 2);
goalPos = zeros(length(stats), 2);
for i = 1 : length(stats)
    [r, c] = find(labeledWhiteImageCopy == i);
    rc = [r c];
    startPos(i, :) = rc(1, :);
    goalYMightBe = r(c == c(end)); 
    validatedColNum = c(end) - c(1) + 1;
    % Calculate the validated column numbers. The columns has only 1 point should be ignored because they don't influent the goal point position.
    for j = c(1) : c(end)
        count = length(c(c == j));
        if count == 1
            validatedColNum = validatedColNum - 1;
        end
    end
    % If validated columns number is even, the goal lies the same side as the start.
    if rem(validatedColNum, 2) == 0
        goalPos(i, 1) = goalYMightBe(1);
        goalPos(i, 2) = c(end);
    % If validated columns number is odd, the goal is on the other side of last column.
    else
        if goalYMightBe(1) ~= rc(1, 1)
            goalPos(i, 1) = goalYMightBe(1);
        else
            goalPos(i, 1) = goalYMightBe(end);
        end
        goalPos(i, 2) = c(end);
    end
end

%% Path planning for all the cells.
connectedCellID = [];
for i = 1 : length(CriticalPoint)
    Pos = CriticalPoint(i, :);
    % only apply to the critical points inside the area rather than the edge.
    if downFree(Pos) && upFree(Pos)
        criticalPointToCellMinDistance = zeros(length(stats), 1);
        for j = 1 : length(stats)
            [r, c] = find(labeledWhiteImageCopy == j);
            rc = [r c];
            dist = zeros(length(r), 1);
            for k = 1 : length(r)
                dist(k) = norm(rc(k, :) - [Pos(2), Pos(1)]);
            end
            dist = sort(dist);
            criticalPointToCellMinDistance(j) = dist(1);
        end
        % Choose the 3 minimal values and save the index number, which is the cell ID.
        n = 3;
        [sortedCriticalPointToCellMinDistance, index] = sort(criticalPointToCellMinDistance);
        neiborCellID = index(1 : n);
        % Pair cell connection.
        cellComeFrom = [];
        cellGoTo = [];
        while n > 0
            if Pos(1) > stats(neiborCellID(n)).Centroid(1)
                cellComeFrom = cat(1, cellComeFrom, neiborCellID(n));
            else
                cellGoTo = cat(1, cellGoTo, neiborCellID(n));
            end
            n = n - 1;
        end
        if length(cellComeFrom) == 1
            connectedCellID = cat(1, connectedCellID, [cellComeFrom(1), cellGoTo(1), 2]);
            connectedCellID = cat(1, connectedCellID, [cellComeFrom(1), cellGoTo(2), 2]);
        elseif length(cellGoTo) == 1
            connectedCellID = cat(1, connectedCellID, [cellComeFrom(1), cellGoTo(1), 2]);
            connectedCellID = cat(1, connectedCellID, [cellComeFrom(2), cellGoTo(1), 2]);
        end
    end
end
% Remove duplicated pairs.
connectedCellID = unique(connectedCellID, 'rows');
% Will only go through cell 1 once.
[xx, ~] = find(connectedCellID(:, 1 : 2) == 1);
connectedCellID(xx, 3) = 1;
connectedCellID(end, 3) = 1;
% Decide the path.
cellPath = [];
nowCellID = 1;
while ~isempty(connectedCellID)
    cellPath = cat(1, cellPath, nowCellID);
    [y, x] = find(connectedCellID(:, 1 : 2)' == nowCellID);
    nowCellID = connectedCellID(x(1), 3 - y(1));
    connectedCellID(x(1), 3) = connectedCellID(x(1), 3) - 1;
    if connectedCellID(x(1), 3) == 0
        n = size(connectedCellID, 1);
        preservedRows = [];
        for i = 1 : n 
            if i ~= x(1)
                preservedRows = cat(2, preservedRows, i);
            end
        end
        connectedCellID = connectedCellID(preservedRows, :);
    end
end
cellPath = cat(1, cellPath, nowCellID);

%% Sweep throughout the freespace while following cellPath order.
alreadySwept = zeros(length(stats), 1);
cellCenterPos = zeros(5, 2);
path = [];
for i = 1 : length(cellPath)
    currentSweepID = cellPath(i);
    if ~alreadySwept(currentSweepID)
        inCellPath = Zigzag(startPos(currentSweepID, :), goalPos(currentSweepID, :));
        if ~isempty(path)
            dist = norm(path(end, :) - inCellPath(1, :));
            if dist > 20
                middlePos = [round(linspace(path(end, 1), inCellPath(1, 1), dist / 20)); ...
                             round(linspace(path(end, 2), inCellPath(1, 2), dist / 20))]';
                path = cat(1, path, middlePos);
            end
        end
        path = cat(1, path, inCellPath);
        cellCenterPos(currentSweepID, :) = path(uint64(length(path) / 2), :);
        alreadySwept(currentSweepID) = 1;
    else
        dist = norm(path(end, :) - cellCenterPos(currentSweepID, :));
        middlePos = [round(linspace(path(end, 1), cellCenterPos(currentSweepID, 1), dist / 20)); ...
                     round(linspace(path(end, 2), cellCenterPos(currentSweepID, 2), dist / 20))]';
        path = cat(1, path, middlePos);
    end
end
path = path - [186, 616];
% path(:, 3) = zeros(length(path), 1);
path(:, 3) = 20;
RCM = [-20, -480, 350];
cppStartPos = path(1, :);
dist = norm(RCM - cppStartPos);
pathToApproach = [round(linspace(RCM(1), cppStartPos(1), dist / 20)); ...
                  round(linspace(RCM(2), cppStartPos(2), dist / 20)); ...
                  round(linspace(RCM(3), cppStartPos(3), dist / 20))]';
path = cat(1, pathToApproach, path);

%% Save final path as a text file.
formatSpec = '%f %f %f \n';
fileID = fopen('path.txt', 'w');
for i = 1 : size(path, 1)
    fprintf(fileID, formatSpec, path(i, 1), path(i, 2) + 100, path(i, 3));
end
fclose(fileID);

%% Detect if the upside of the critical point is free.
function [ifFreeSpace] = upFree(Pos)
    global whiteImage
    ifFreeSpace = [];
    for i = 1 : 50
        if Pos(2) - i <= 0
            break
        elseif whiteImage(Pos(2) - i, Pos(1)) == 0
            ifFreeSpace = true; % true = 1
        end
    end
    if isempty(ifFreeSpace)
        ifFreeSpace = false; % false = 0
    end
end

%% Detect if the downside of the critical point is free.
function [ifFreeSpace] = downFree(Pos)
    global whiteImage
    ifFreeSpace = [];
    for i = 1 : 50
        if Pos(2) + i > size(whiteImage, 1) % size(whiteImage, 1) = 266
            break
        elseif whiteImage(Pos(2) + i, Pos(1)) == 0
            ifFreeSpace = true; % true = 1
        end
    end
    if isempty(ifFreeSpace)
        ifFreeSpace = false; % false = 0
    end
end