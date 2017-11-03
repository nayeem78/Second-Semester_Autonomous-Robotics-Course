%% RRT algorithm
function [vertices,edges,path]=rrt(map,q_start,q_goal,k,delta_q,p)

[m,n] = size(map);
% Initialize the vertices with q_start
vertices = q_start;

% Initialize the edges as empty
edges = [];

i = 1;
while i < k
    i = i + 1
    % Create a number between 0 and 1
    rand_val = rand;
    if rand_val < p % 30 percent chance 
        q_rand = q_goal;
    else
        q_rand = [randi(m),randi(n)];
    end
    
    % Find q_near from q_rand in 'vertices'
    dif = bsxfun(@minus, vertices, q_rand);
    vertices_dis = sqrt(dif(:,1).^2 + dif(:,2).^2); % Euclidean distance between all the vertices and q_rand
    q_near_index = find(vertices_dis == min(vertices_dis)); % Get the index of q_near
    q_near_index = q_near_index(1,:);
    q_near = vertices(q_near_index,:);
    
    % Get q_new
    
    % the q_rand is short enough
    if vertices_dis(q_near_index) <= delta_q
        q_new = q_rand;
        dis = vertices_dis(q_near_index);
    else
        vector = q_rand - q_near;
        vector = vector ./ norm(vector);
        q_new = q_near + delta_q .* vector;
        dis = delta_q;
    end
    if q_new == q_goal
        vertices = [vertices;q_new];
        edges = [edges;size(vertices,1), q_near_index];
        break;
    end
    % Check if the q_new is valid
    if q_new(1) < 0 || q_new(2) < 0 || q_new(1) > size(map,1) || q_new(2) > size(map,2)
        continue;
    else
        q_new = floor(q_new);
    end
    
    % Check if there is an obstacle between q_near and q_new
    ob = 0; % Label for obstacle
    d = dis / 10;
    vector = q_rand - q_near;
    if norm(vector) ~= 0 % the q_rand is not in the vertices list
        vector = vector ./ norm(vector);
    else
        continue;
    end
    for j = 1 : 10
        q_middle = round(q_near + j * d .* vector);
        if map(q_middle(1), q_middle(2)) ~= 0
            ob = 1;
            break;
        end
    end
    if ob == 1
        continue;
    else
        vertices = [vertices;q_new];
        edges = [edges;size(vertices,1), q_near_index]; 
    end
end

% Get Path
path = [size(vertices,1)];
cur = path(1);
while cur ~= 1
    cur_idx = find(edges(:,1) == cur);
    path = [edges(cur_idx,2),path];
    cur = path(1);
end

end