function [x y z] = rotateAboutAxis(M,tX,tY,tZ,P) % rotates in x, y and z directions
    M=M-P;
    zr=[cosd(tZ),-sind(tZ),0;sind(tZ),cosd(tZ),0;0,0,1];
    yr=[cosd(tY),0,sind(tY);0,1,0;-sind(tY),0,cosd(tY)];
    xr=[1,0,0;0,cosd(tX),-sind(tX);0,sind(tX),cosd(tX)];
    % change to r(r(r(p)))
    r=xr*yr*zr;
    M=r*M(:,:)';
    M=M'+P;
    x=M(:,1);
    y=M(:,2);
    z=M(:,3);
end