function shape=sphere(radius,centre,x,y,z)
    shape=zeros(numel(x),numel(y),numel(z));
    for i=1:numel(x)
        for j=1:numel(y)
            for k=1:numel(z)
                if sqrt((x(i)-centre(1)).^2+(y(j)-centre(2)).^2+(z(k)-centre(3)).^2) < radius
                    shape(j,i,k)=1;
                end
            end
        end
    end
end