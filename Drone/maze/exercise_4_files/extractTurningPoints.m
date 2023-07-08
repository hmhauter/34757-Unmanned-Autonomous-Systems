function turning_points = extractTurningPoints(route)
    turning_points = [route(1, :)];  % Initialize with the first point
    
    for i = 2:size(route, 1)-1
        prev_point = route(i-1, :);
        curr_point = route(i, :);
        next_point = route(i+1, :);
        
        direction_prev_curr = curr_point - prev_point;
        direction_curr_next = next_point - curr_point;
        
        % Check if direction changes
        if ~isequal(direction_prev_curr, direction_curr_next)
            turning_points = [turning_points; curr_point];
        end
    end
    
    turning_points = [turning_points; route(end, :)];  % Include the last point
end