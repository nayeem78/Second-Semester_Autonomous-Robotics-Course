%Autonomous Robotics
%Lab 4: Topological Maps - Visibility Graphs 
%Algorithm - Rotational Plane Sweep Algorithm
%Author: Nayee Muddin Khan Dousai

%input: vertices on map
%output: edges of visibility graph

function [ edges ] = RPS( vertices )
    %number of vertices input
    [m,~] = size(vertices);
    %initializations of the edges lists
    edges = [];
    ind_edg = 1;
    %edges of obstacles list
    E =[];
    %determining edges of obstacles
    for i=2:m-1
        if ( vertices(i-1,3) ~= vertices(i,3) )
            nobj = i;
        end
        if ( vertices(i+1,3) == vertices(i,3) )
            E(ind_edg,:) = [i, i+1];
        else
            E(ind_edg,:) = [i, nobj];
        end
        ind_edg = ind_edg + 1;
    end
    ind_edg = 1;
    
    % Checking all the vertices of visibility map
    for i=1:m
        % chosen vertex
        point = vertices(i,:);
        % all vertices except the current one
        vertex = [vertices(1:i-1,:); vertices(i+1:m,:)];
        
        %sorting vertices wrt to their corresponding angles
        %calculating angles of each vertex line
        ang=[];
        for j=1:size(vertex,1)
            ang(j,1)= atan2(vertex(j,2)-point(2), vertex(j,1)-point(1));
            if ang(j,1)<0
                ang(j,1)=ang(j,1)+2*pi;
            end
        end
        % sorting the vertices wrt to their angles
        [~, angsort] = sort(ang(:),'ascend');
        angsort = angsort';

        % initialisation of S list by horizontal line
        E_dist = [];
        S = [];
        [n,~] = size(E);
        s_ind = 1;
        % second point for horizontal line
        [~, max_ind] = max(vertices(:,1));
        % horizontal line endpoints
        line_h = [point(1), point(2), vertices(max_ind,1), point(2)];

        % determines the edges that intersects the line
        for j=1:n
            % end points of the line between current vertex and vertex
            % being checked intersection for
            line = [vertices( E(j,1), 1:2),  vertices( E(j,2), 1:2)];
            % check whether lines intersected with other lines
            [ intersct , dist] = intersected(line_h, line);
            if intersct
                if abs( [line_h(2)-line_h(4), line_h(3)-line_h(1), (line_h(1)*line_h(4))-(line_h(3)*line_h(2))] - [line(2)-line(4), line(3)-line(1), (line(1)*line(4))-(line(3)*line(2))] ) > 0.0001
                    % adding to the S list
                    S(s_ind) = j;
                    E_dist(s_ind) = dist;
                    s_ind = s_ind + 1;
                end
            end
        end
        % sort S list according to distance form the current vertex
        [sorted_E_dist, sorted_ind] = sort(E_dist(:),'ascend');
        sorted_E_dist = sorted_E_dist';
        S = S(sorted_ind');
        
        % checking all other vertices
        for j=1:m-1
            % calculating index of initial vertex in initial array
            if (angsort(j)<i);
                nobj = angsort(j);
            else
                nobj = angsort(j) + 1;
            end;
            %the corresponding vertex being checked
            vi = vertex(angsort(j),:);
            
            % checking visibility and adding to S list if appropriate
            if (isempty(S))
                % adding vertex to the edges list
                edges(ind_edg,:) = [i, nobj];
                ind_edg = ind_edg + 1;
            else
                % if S list not empty run the algo for determining
                % visibility
                dst = norm((point(1:2) - vi(1:2)),2);
                sind = 1;
                %line between point and correspondingvertice
                linep = [point(1) point(2) vi(1) vi(2)];
                %checking intersection
                while ( ( sind <= size(S,2) ) )
                    eind = S(sind);
                    linee = [vertices( E(eind,1), 1:2),  vertices( E(eind,2), 1:2)];
                    [ inter , ~] = intersected(linep, linee);
                    if( inter )
                        break;
                    end
                    sind = sind + 1;
                end
                %visible: add to edges list
                if ~inter
                    edges(ind_edg,:) = [i, nobj];                    
                    ind_edg = ind_edg + 1;
                end
            end
            
            % determining which edges to delete or add in S list
            [insert_edges, delete_edges] = fedges(find( E(:,1) == nobj ),find( E(:,2) == nobj ),point,vertex(angsort(j),:),E,vertices);
            
            % if vi not in S list
            if ~isempty( insert_edges)
                % adding to S list                
                for k=1:size(insert_edges,2)
                    if isempty( find(S(:) == insert_edges(k) ) )
                        dst = norm((point(1:2) - vi(1:2)),2);
                        ind1 = E_dist(:) <= dst;
                        ind2 = E_dist(:) > dst;
                        S = [S(ind1'), insert_edges(k), S(ind2') ];
                        E_dist = [E_dist(ind1'), dst, E_dist(ind2')];
                    end
                end
            end
            
            % if vi is at end of an edge that is in S list
            if ~isempty( delete_edges)
                % delete from S list
                for k=1:size(delete_edges,2)
                    if ~( isempty( find(S(:) == delete_edges(k)) ) )
                        sind = find( S(:) == delete_edges(k) );
                        S(sind) = [];
                        E_dist(sind) = [];
                    end
                end
            end
        end   
    end
    
    % delete internal edges and add polygon lines
    ind_edg = 1;
    % delete edges that belongs to the same polygon
    for i=1:size(edges,1)
        if ( vertices( edges(ind_edg,1),3 ) == vertices( edges(ind_edg,2),3 ) )
            %special case for concave polygon
            %draws a many points on line between vertices
            %checkes if they are inside the polygon or outside
            x1=vertices( edges(ind_edg,1),1);
            y1=vertices( edges(ind_edg,1),2);
            x2=vertices( edges(ind_edg,2),1);
            y2=vertices( edges(ind_edg,2),2);
            r=sqrt((x1-x2).^2+(y1-y2).^2);
            t=atan2(y2-y1,x2-x1);
            d=0.2:0.1:r-0.2;
            y=y1+(d*sin(t));
            x=x1+(d*cos(t));
            k=1;
            xv=[];
            yv=[];
            %determining vertices of polygon
            for j=1:m
                if vertices( edges(ind_edg,1),3 ) == vertices(j,3)
                    xv(k)=vertices(j,1);
                    yv(k)=vertices(j,2);
                    k=k+1;
                end
            end
            xv = [xv' ; xv(1)];
            yv = [yv' ; yv(1)];
            %checking if inside or outside polygon
            [in,~]=inpolygon(x,y,xv,yv);
            if (max(in)~=0)
                edges(ind_edg,:) = [];
            else
                ind_edg=ind_edg+1;
            end
        else
            ind_edg = ind_edg + 1;
        end
    end
    
    %adding obstacle edges to the edges list
    edges = [edges; E];
    %sorting edges
    edges=sortrows(edges,1);
    %deleting redundant edges
    for i=1:size(edges,1)
        for j=i:size(edges,1)
            if edges(i,1)==edges(j,2) && edges(j,1)==edges(i,2)
                edges(j,:)=[0 0];
            end
        end
    end
    edges( ~any(edges,2), : ) = [];

    %drawing visibility graph
    for i=1:size(edges,1)
        plot([vertices( edges(i,1), 1); vertices( edges(i,2), 1)], [vertices( edges(i,1), 2); vertices( edges(i,2), 2)],'r');
        hold on;
    end
    
    %drawing the obstacles
    % starting position 
    plot( vertices(1,1), vertices(1,2), 'k*','MarkerSize', 10);
    hold on;
    % last/goal position
    plot( vertices((size(vertices,1)),1), vertices(size(vertices,1),2), 'g*','MarkerSize', 10);
    hold on;
    % obstacle lines
    for i=1:size(E,1)
        plot([vertices( E(i,1), 1); vertices( E(i,2), 1)],[vertices( E(i,1), 2); vertices( E(i,2), 2)]);
        hold on;
    end

end

%function for checking intersection
function [ intersct, dist ] = intersected(line1, line2)
    % set distance to 0 initially
    dist = 0;
    %change line into ax+by+c=0
    line1_abc = [line1(2)-line1(4), line1(3)-line1(1), (line1(1)*line1(4))-(line1(3)*line1(2))];
    line2_abc = [line2(2)-line2(4), line2(3)-line2(1), (line2(1)*line2(4))-(line2(3)*line2(2))];
    %compute the intersection point
    inst_pt = cross(line1_abc,line2_abc);
    %no intersection?
    if inst_pt(3) == 0
        intersct = false;
    %intersection
    else
        % normalize point to have third component as 1
        inst_pt = inst_pt ./ inst_pt(3);
        % sort the x values of each line
        x_line1 = sort([line1(1), line1(3)],'ascend');
        x_line2 = sort([line2(1), line2(3)],'ascend');
        %check if intersection on a vertex
        in_edge1=false;
        in_edge2=false;
        if(((abs(inst_pt(1)-line1(1))<0.001) && (abs(inst_pt(2)-line1(2))<0.001)) || ((abs(inst_pt(1)-line1(3))<0.001) && (abs(inst_pt(2)-line1(4))<0.001)))
            in_edge1 = true;
        end
        if(((abs(inst_pt(1)-line2(1))<0.001) && (abs(inst_pt(2)-line2(2))< 0.001)) || ((abs(inst_pt(1)-line2(3))<0.001) && (abs(inst_pt(2)-line2(4))<0.001)))
            in_edge2 = true;
        end
        %if intersction on vertices then no intersection with other lines
        if ( in_edge1 || in_edge2)
            intersct = false;
        else
            % intersction true: calculate distance with the edge
            if ( ( inst_pt(1) >= x_line1(1) ) && ( inst_pt(1) <= x_line1(2) ) && ( inst_pt(1) >= x_line2(1) ) && ( inst_pt(1) <= x_line2(2) ) )
                intersct = true;
                dist = norm((inst_pt(1:2) - line1(1:2)),2);
            else
                intersct = false;
            end
        end
    end
end

%This function determines initial edges and end edges
function [ iedges, dedges ] = fedges(  sind, eind, point, vi, E, vertices)
    %line in ax+by+c=0
    linep =  [point(2)-vi(2), vi(1)-point(1), (point(1)*vi(2))-(vi(1)*point(2))];
    linep = linep./ sqrt( linep(1).^2 + linep(2).^2);
    %start and end points
    vstart = [vertices( E(sind,2), 1:2), 1];
    vend = [vertices( E(eind,1), 1:2), 1];
    %initialize output edges
    iedges = [];
    dedges = [];
    proj_start = 0;
    proj_end = 0;
    %determine whether to add or delete
    if(~isempty(sind))
        proj_start = dot(linep, vstart);
    end
    if(~isempty(eind))
        proj_end = dot(linep, vend);
    end
    %initialize indexes
    iind = 1;
    dind = 1;
    %setting edges to add
    if( proj_start > 0  )
        iedges( iind ) = sind;   
        iind = iind + 1;
    end
    %settingg edges to delete
    if( proj_start < 0 ) 
        dedges( dind ) = sind;   
        dind = dind + 1;
    end
    %setting edges to add
    if( proj_end > 0  )
        iedges( iind ) = eind;   
        iind = iind + 1;
    end
    %settingg edges to delete
    if( proj_end < 0 )
        dedges( dind ) = eind;   
        dind = dind + 1;
    end

end
