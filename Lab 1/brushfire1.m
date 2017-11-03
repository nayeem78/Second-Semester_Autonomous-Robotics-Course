function [value_map] = brushfire1(map)

[m,n] = size(map);

value_map = map;

%potential label
count = 1;

%Run the iterations until all the elements are non zero
while(nnz(value_map) ~= m*n)  
    
    %Iterate over the map
    for i=2:m-1
        for j=2:n-1
            
            %Change only the values that are non zero
            if (value_map(i,j) == 0)
                if (value_map(i-1,j-1) == count) || (value_map(i-1,j) == count) ||(value_map(i-1,j+1) == count) || (value_map(i,j-1) == count) || (value_map(i,j+1) == count) || (value_map(i+1,j-1) == count) ||   (value_map(i+1,j) == count) || (value_map(i+1,j+1) == count)value_map(i,j) = count + 1;
                end
            end
    

        end
    end
    
    count = count + 1;
end

no_of_iterations = count - 1

end