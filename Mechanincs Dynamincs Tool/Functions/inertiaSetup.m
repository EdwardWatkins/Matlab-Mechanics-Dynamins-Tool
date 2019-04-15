function Matrix = inertiaSetup(shape,x,y,z)
    q = 1;
    for i = 1:numel(shape(:,1))
        for j = 1:numel(shape(:,2))
            for k = 1:numel(shape(:,3))
                if shape(j,i,k) ~= 0
                    Matrix(q,:) = [x(i),y(j),z(k)];
                    q = q + 1;
                end
            end
        end
    end
end

