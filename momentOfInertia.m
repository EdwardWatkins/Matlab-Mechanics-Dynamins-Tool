function inertia=momentOfInertia(shape,mass,x,y,z) % shape variable, shapes total mass, shapes x,y,z info (not mesh-grid)
    [xx,yy,zz]=meshgrid(x,y,z);
    mi=mass/sum(shape(:));
    inertia=[0,0,0,0,0,0];
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
                    radiusx=(A*2)/(xm); % distance from x axis
                    inertia(1)=shape(j,i,k)*mi*radiusx.^2+inertia(1);
                    %y
                    vp=sqrt((xx(j,i,k)).^2+(yy(j,i,k)-1).^2+(zz(j,i,k)).^2);
                    s=(ym+vp+pm)/2;
                    A=sqrt(s*(s-ym)*(s-vp)*(s-pm));
                    radiusy=(A*2)/(ym);
                    inertia(2)=shape(j,i,k)*mi*radiusy.^2+inertia(2);
                    %z
                    vp=sqrt((xx(j,i,k)).^2+(yy(j,i,k)).^2+(zz(j,i,k)-1).^2);
                    s=(zm+vp+pm)/2;
                    A=sqrt(s*(s-zm)*(s-vp)*(s-pm));
                    radiusz=(A*2)/(zm);
                    inertia(3)=shape(j,i,k)*mi*radiusz.^2+inertia(3);
                    % xy
                    inertia(4)=mi*shape(j,i,k)*radiusx*radiusy+inertia(4);
                    % yz
                    inertia(5)=mi*shape(j,i,k)*radiusy*radiusz+inertia(5);
                    % xz
                    inertia(6)=mi*shape(j,i,k)*radiusx*radiusz+inertia(6);
                end
            end
        end
    end
end
