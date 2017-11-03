function [value_map, trajectory] = wavefront(map, start, end_point)

value_map = map;

[m,n] = size(value_map);

value = 2;

value_map(end_point(1), end_point(2)) = 2;

dx = [-1 1 1 -1 0 0 -1 1];
dy = [-1 -1 1 1 -1 1 0 0];

while(nnz(value_map) ~= m*n)
    
    %get the indices of the desired potential value
    [X, Y] = find(value_map==value);
    
    r = size(X);
    
    %Loop over the set of indices with current potential value
    for i = 1:r(1)
            
        %neighbour value
        neigh = [value_map((X(i)-1),(Y(i)-1)), value_map((X(i)-1),(Y(i))), value_map((X(i)-1),(Y(i)+1)), value_map((X(i)),(Y(i)-1)), value_map((X(i)),(Y(i)+1)), value_map((X(i)+1),(Y(i)-1)), value_map((X(i)+1),(Y(i))), value_map((X(i)+1),(Y(i)+1))];
        
        %Replace all zeros with the next potential value
        neigh(neigh==0) = value + 1;
        
        %Update the neighbourhood potential in value_map
        value_map((X(i)-1), (Y(i)-1)) = neigh(1);
        value_map((X(i)-1), (Y(i))) = neigh(2);
        value_map((X(i)-1), (Y(i)+1)) = neigh(3);
        value_map((X(i)), (Y(i)-1)) = neigh(4);
        value_map((X(i)), (Y(i)+1)) = neigh(5);
        value_map((X(i)+1), (Y(i)-1)) = neigh(6);
        value_map((X(i)+1), (Y(i))) = neigh(7);
        value_map((X(i)+1), (Y(i)+1)) = neigh(8);
    end
    
    %increment the potential
    value = value + 1;
    
end

%Trajectory building.
% begin from the start position
x = start(1);
y = start(2);
% initialize an array for storing the trajectory
trajectory = zeros(value_map(x,y)-1, 2); %[]
steps = 1;
trajectory(steps, 1) = x;
trajectory(steps, 2) = y;
% the start position is reachable
if value_map(x, y) ~= 0
    % repeat
    while 1
        % declare directions to go
        dir_x = 0;
        dir_y = 0;
        % look at the neighbours
        for i = 1:8
            % if it's not out of boundaries and has less value than the
            % current point
            if x+dx(i) > 0 && x+dx(i) <= m && y+dy(i) > 0 && y+dy(i) <= n && value_map(x+dx(i), y+dy(i)) > 1 && value_map(x+dx(i), y+dy(i)) < value_map(x, y)
                % save the directions
                dir_x = dx(i);
                dir_y = dy(i);
            end;
        end;
        % get the coordinates of the next pixels using the obtained
        % directions
        x = x + dir_x;
        y = y + dir_y;
        % increase the step
        steps = steps + 1;
        % and add new points to the trajectory
        trajectory(steps, 1) = x;
        trajectory(steps, 2) = y;
        % stop the loop when we reach to the goal
        if x == end_point(1) && y == end_point(2)
            break;
        end;
    end;

end