function [x y z] = rotateAboutVector(M,t,V,P) % M=[x,y,z] shape data, t=theta(angle in degrees), V=Vector to rotate about, P=Point to rotate about
    M=M-P;
    U=V/norm(V);
    x=U(1);
    y=U(2);
    z=U(3);
    R=[(cosd(t)+x*x*(1-cosd(t))),(x*y*(1-cosd(t))-z*sind(t)),(x*z*(1-cosd(t))+y*sind(t));
        (y*x*(1-cosd(t))+z*sind(t)),(cosd(t)+y*y*(1-cosd(t))),(y*z*(1-cosd(t))-x*sind(t));
        (z*x*(1-cosd(t))-y*sind(t)),(z*y*(1-cosd(t))+x*sind(t)),(cosd(t)+z*z*(1-cosd(t)));];
    M=R*M(:,:)';
    M=M'+P;
    x=M(:,1);
    y=M(:,2);
    z=M(:,3);
end