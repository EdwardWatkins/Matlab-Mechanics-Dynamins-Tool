function inertia=momentOfInertia(shape,mass,xx,yy,zz) % shape variable, shapes total mass, shapes x,y,z info (not mesh-grid)
    mi=mass/sum(shape(:));
    inertia=[0,0,0];
    xm=1; ym=1; zm=1;
    for i=1:(numel(xx)^(1/3))
        for j=1:(numel(yy)^(1/3))
            for k=1:(numel(zz)^(1/3))
                if shape(j,i,k) ~= 0
                    pm=sqrt((-xx(j,i,k)).^2+(-yy(j,i,k)).^2+(-zz(j,i,k)).^2); % distance point to origin
                    %x
                    vp=sqrt((xx(j,i,k)-1).^2+(yy(j,i,k)).^2+(zz(j,i,k)).^2); % distance from point to vector (along axis)
                    s=(xm+vp+pm)/2;
                    A=sqrt(s*(s-xm)*(s-vp)*(s-pm));
                    radius=(A*2)/(xm); % distance from x axis
                    inertia(1)=shape(j,i,k)*mi*radius.^2+inertia(1);
                    %y
                    vp=sqrt((xx(j,i,k)).^2+(yy(j,i,k)-1).^2+(zz(j,i,k)).^2);
                    s=(ym+vp+pm)/2;
                    A=sqrt(s*(s-ym)*(s-vp)*(s-pm));
                    radius=(A*2)/(ym);
                    inertia(2)=shape(j,i,k)*mi*radius.^2+inertia(2);
                    %z
                    vp=sqrt((xx(j,i,k)).^2+(yy(j,i,k)).^2+(zz(j,i,k)-1).^2);
                    s=(zm+vp+pm)/2;
                    A=sqrt(s*(s-zm)*(s-vp)*(s-pm));
                    radius=(A*2)/(zm);
                    inertia(3)=shape(j,i,k)*mi*radius.^2+inertia(3);
                end
            end
        end
    end
end
