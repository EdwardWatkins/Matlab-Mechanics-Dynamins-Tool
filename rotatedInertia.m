function inertia = rotatedInertia(Matrix,tX,tY,tZ,pointOfRotation,momentOfInertiaCentre,mass,Mass)
    Matrix = Matrix - pointOfRotation;
    mi = mass/sum(Mass(:));%numel(Matrix(:,1));
    zr=[cosd(tZ),-sind(tZ),0;sind(tZ),cosd(tZ),0;0,0,1]; 
    yr=[cosd(tY),0,sind(tY);0,1,0;-sind(tY),0,cosd(tY)];
    xr=[1,0,0;0,cosd(tX),-sind(tX);0,sind(tX),cosd(tX)];
    r=xr*yr*zr;
    Matrix = r * Matrix(:,:)';
    Matrix = Matrix' + pointOfRotation;
    inertia = [0,0,0,0,0,0];
    for i = 1:numel(Matrix(:,1))
        % point
        pm=sqrt((momentOfInertiaCentre(1)-Matrix(i,1)).^2+(momentOfInertiaCentre(2)-Matrix(i,2)).^2+(momentOfInertiaCentre(3)-Matrix(i,3)).^2); % distance point to origin
        % x
        vp=sqrt((Matrix(i,1)-(momentOfInertiaCentre(1)+1)).^2+(Matrix(i,2)-momentOfInertiaCentre(2)).^2+(Matrix(i,3)-momentOfInertiaCentre(3)).^2); % distance from point to vector (along axis)
        s=(1+vp+pm)/2;
        A=sqrt(s*(s-1)*(s-vp)*(s-pm));
        radiusx= A*2; % distance from x axis
        inertia(1)= mi*Mass(i)*radiusx.^2+inertia(1);
        % y
        vp=sqrt((Matrix(i,1)-momentOfInertiaCentre(1)).^2+(Matrix(i,2)-(momentOfInertiaCentre(2)+1)).^2+(Matrix(i,3)-momentOfInertiaCentre(3)).^2); % distance from point to vector (along axis)
        s=(1+vp+pm)/2;
        A=sqrt(s*(s-1)*(s-vp)*(s-pm));
        radiusy=(A*2);
        inertia(2)= mi*Mass(i)*radiusy.^2+inertia(2);
        % z
        vp=sqrt((Matrix(i,1)-(momentOfInertiaCentre(1))).^2+(Matrix(i,2)-momentOfInertiaCentre(2)).^2+(Matrix(i,3)-(momentOfInertiaCentre(3)+1)).^2); % distance from point to vector (along axis)
        s=(1+vp+pm)/2;
        A=sqrt(s*(s-1)*(s-vp)*(s-pm));
        radiusz= (A*2);
        inertia(3)= mi*Mass(i)*radiusz.^2+inertia(3);
        % xy
        inertia(4)=mi*Mass(i)*radiusx*radiusy+inertia(4);
        % yz
        inertia(5)=mi*Mass(i)*radiusy*radiusz+inertia(5);
        % xz
        inertia(6)=mi*Mass(i)*radiusx*radiusz+inertia(6);
    end
end