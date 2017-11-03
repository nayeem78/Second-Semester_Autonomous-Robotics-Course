%A-Star Algorithm
% vertices and edges has to be defined before running the code
function [path,minCost]=Astar(vertices, edges)

% Calculate heuristic distance for all the vertices
goal = vertices(end,1:2);
distance = bsxfun(@minus,vertices(:,1:2),goal);
heu_distance = sqrt(distance(:,1).^2 + distance(:,2).^2);

% Create a matrix storing the distance between two neighbouring vertices
number_node = size(vertices,1);
distance_betw_node = zeros(number_node);
for i = 1 : size(edges,1)
    v1 = vertices(edges(i,1),:);
    v2 = vertices(edges(i,2),:);
    distance_betw_node(edges(i,1),edges(i,2)) = sqrt((v2(1) - v1(1)).^2 + (v2(2) - v1(2)).^2);
    distance_betw_node(edges(i,2),edges(i,1)) = distance_betw_node(edges(i,1),edges(i,2));
end

O = [];
% C list, [current point idx, back point idx], 0 means nothing in the back point
C = [1,0]; 

% Route distance from starting point to the nodes
dis_node = 1000000*ones(number_node,1);
dis_node(1) = 0; 
while 1
    parent_num = C(size(C,1),1);% Current node
    % Find the neighbor nodes by checking if they have distance between
    % each other
    child_num = find(distance_betw_node(parent_num,:) ~= 0);

    if size(O,1) == 0 % no element in O, the element can be directly inserted
        for i = 1 : length(child_num)
            dis_node(child_num(i)) = dis_node(parent_num) + distance_betw_node(parent_num,child_num(i));
            cost = dis_node(child_num(i)) + heu_distance(child_num(i));
            O = [O;child_num(i),cost,parent_num];
        end
    else
        for i = 1 : length(child_num)
            dis_node_tmp = dis_node(parent_num) + distance_betw_node(parent_num,child_num(i));
            
            % if the child is already in Close list || The current route is
            % already not the shortest
            if sum(find(C(:,1) == child_num(i))) ~= 0 || dis_node_tmp > dis_node(child_num(i))
                continue;
            end
            dis_node(child_num(i)) = dis_node_tmp;
            cost = dis_node(child_num(i)) + heu_distance(child_num(i));
            O = [O;child_num(i),cost,parent_num];
        end
    end
    % Sort the O list based on the distance
    [~,I] = sort(O(:,2));
    O = O(I,:);
    
    % Update C list
    C = [C; O(1,1),O(1,3)];
    
    % the first element in O list is not the goal point
    if O(1,1) ~= size(vertices,1) 
        % Pop the first row in the O list out
        O = O(2:end,:);
    else
        minCost = O(1,2);
        break;
    end
end

% Get the path
path = [size(vertices,1)];
cur = path(1);
while cur ~= 1
    cur_idx = find(C(:,1) == cur);
    path = [C(cur_idx,2),path];
    cur = path(1);
end

end


