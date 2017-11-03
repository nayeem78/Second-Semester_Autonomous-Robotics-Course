function [path_smooth]=smooth(map,path,vertices,delta)

end_idx = path(length(path));
q_end = vertices(end_idx,:); % At First q_end is the goal point
path_smooth = [end_idx];
while end_idx ~= path(1)
    % Update start_vertex
    for i = 1 : length(path) - 1
        start_idx = path(i);
        q_start = vertices(start_idx,:);
        
        % Get the unit vector from q_start to q_end
        v = q_end - q_start;
        v_norm = norm(v);
        v = v ./ norm(v);
        
        sep_num = floor(v_norm / delta);
        % q_start and q_end is near enough, q_end can be directly update
        if sep_num == 0
            end_idx = start_idx;
            q_end = vertices(end_idx,:);
            path_smooth = [end_idx,path_smooth];
            break;
        else
            ob = 0;
            for j = 1 : sep_num
                q_middle = floor(q_start + j * delta .* v);
                if map(q_middle(1), q_middle(2)) ~= 0
                    ob = 1;
                    break;
                end
            end
            % Free space between two q_start and q_end
            if ob == 0
                end_idx = start_idx;
                q_end = vertices(end_idx,:);
                path_smooth = [end_idx,path_smooth];
                break;
            end
        end
            
    end
end