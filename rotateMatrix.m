function [x y z] = rotateAboutAxis(M,tX,tY,tZ)
    zr=[cosd(tZ),-sind(tZ),0;sind(tZ),cosd(tZ),0;0,0,1];
    yr=[cosd(tY),0,sind(tY);0,1,0;-sind(tY),0,cosd(tY)];
    xr=[1,0,0;0,cosd(tX),-sind(tX);0,sind(tX),cosd(tX)];
    r=xr*yr*zr;
    for i=1:numel(M)/3
        M(i,:)=r*M(i,:)';
    end
    x=M(:,1);
    y=M(:,2);
    z=M(:,3);
end