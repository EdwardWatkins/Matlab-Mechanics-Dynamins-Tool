function inertia=inertia(shape,mass,x,y,z,point1,point2) % shape variable, shapes total mass, shapes x,y,z info (not mesh-grid), 2 points on the vector of rotation
    mi=mass/sum(shape(:));
    inertia=0;
    vm=sqrt((point1(1)-point2(1)).^2+(point1(2)-point2(2)).^2+(point1(3)-point2(3)).^2);
    for i=1:numel(x)
        for j=1:numel(y)
            for k=1:numel(z)
                if shape(i,j,k) ~= 0
                    vp=sqrt((x(i)-point2(1)).^2+(y(j)-point2(2)).^2+(z(k)-point2(3)).^2);
                    pm=sqrt((point1(1)-x(i)).^2+(point1(2)-y(j)).^2+(point1(3)-z(k)).^2);
                    s=(vm+vp+pm)/2;
                    A=sqrt(s*(s-vm)*(s-vp)*(s-pm));
                    radius=(A*2)/(vm);
                    inertia=shape(i,j,k)*mi*radius.^2+inertia;
                end
            end
        end
    end
end