function path = dfs_2d_old(maze, start, goal)
    
    visited = false(size(maze));
    stack = {};     % frontier
    stack{end+1} = start;
    turning_points = [];

    parent = zeros(size(maze));
    direction = 'r';
    while ~isempty(stack)
        current = stack{end};
        stack(end) = [];
        
        % check if goal
        if current(1) == goal(1) && current(2) == goal(2)
            break;
        end
        
        % track visited cells
        visited(current(1), current(2)) = true;
        
        [neighbors, direction_new] = getNeighbors(current, maze);
        
        % update frontier
        sz = size(neighbors);
        for i = 1:sz(1)
            neighbor = neighbors(i, :);
            if ~visited(neighbor(1), neighbor(2))
                stack{end+1} = neighbor;
                visited(neighbor(1), neighbor(2)) = true;
             
                if direction_new ~= direction
                    turning_points(end+1) = neighbor;
                end
                
                parent(neighbor(1), neighbor(2)) = sub2ind(size(maze), [current(1), current(2)], doesTurn);
            end
        end
        direction = direction_new;
    end
    
    % traverse
    path = reconstructPath(parent, start, goal, maze);
end

function [neighbors, direction] = getNeighbors(position, maze)
% explore tge neighbors of the current cell
    deltas = [ 0 -1; 0 1; -1 0; 1 0;];  % left, right, up, down
    directions = ['l', 'r', 'u', 'd'];
    neighbors = [];
    for i = 1:size(deltas, 1)
        neighbor = position + deltas(i, :);
        if isValid(neighbor, maze)
            neighbors = [neighbors; neighbor];
            direction = directions(i);
        end
    end
end

function valid = isValid(position, maze)
    valid = position(1) >= 1 && position(1) <= size(maze, 1) && ...
            position(2) >= 1 && position(2) <= size(maze, 2) && ...
            maze(position(1), position(2)) == 0;
end

function path = reconstructPath(parent, start, goal, maze)
    path = [goal(1)-1, goal(2)-1, 1];
    current = sub2ind(size(maze), goal(1), goal(2));
    
    while current ~= sub2ind(size(maze), start(1), start(2))
        [row, col, direction] = ind2sub(size(maze), current);
        disp(direction)
        path = [path; row-1, col-1, 1];
        current = parent(row, col);
    end
    path = [path; start(1)-1, start(2)-1, 1];
    path = flipud(path);  
    
    disp(path);
end

