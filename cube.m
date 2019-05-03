function shape=cube(xl,yl,zl,x,y,z) % length in x, y, z and x, y, z reference 
    shape=zeros(numel(x),numel(y),numel(z));
    for i=1:numel(x)
        if abs(x(i))<(xl/2)
            for j=1:numel(y)
                if abs(y(j))<(yl/2)
                    for k=1:numel(z)
                        if abs(z(k))<(zl/2)
                            shape(i,j,k)=1;
                        end
                    end
                end
            end
        end
    end
end