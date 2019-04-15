function [alpha,beta,deltaTheta] = linkMotion(v1,p1,v2,p2,L,theta,radius,steps)
    rmin=min(radius);
    rmax=max(radius);
    delta=1e-3;
    rmean=mean(radius);
    c1=v1(1).^2+v1(2).^2+v1(3).^2;
    c2=2*p1(1)*v1(1)-2*p2(1)*v1(1)+2*p1(2)*v1(2)-2*p2(2)*v1(2)+2*p1(3)*v1(3)-2*p2(3)*v1(3);
    c3=-2*v2(1)*v1(1)-2*v2(2)*v1(2)-2*v2(3)*v1(3);
    c4=-2*p1(1)*v2(1)+2*p2(1)*v2(1)-2*p1(2)*v2(2)+2*p2(2)*v2(2)-2*p1(3)*v2(3)+2*p2(3)*v2(3);
    c5=v2(1).^2+v2(2).^2+v2(3).^2;
    c6=p1(1).^2+2*p2(1)*p1(1)+p2(1).^2+p1(2).^2-2*p2(2)*p1(2)+p2(2).^2+p1(3).^2-2*p2(3)*p1(3)+p2(3).^2;
    xm=((-2*c4*c3+4*c5*c2)+(sqrt((2*c4*c3-4*c5*c2)^2-(4*(c3^2-4*c5*c1)*(c4^2-4*c5*c6+4*c5*L^2)))))/(2*(c3^2-4*c5*c1));
    Ox=(-2*c4*c3+4*c5*c2)/(2*c3^2-8*c5*c1);
    Oy=(-2*c2*c3+4*c1*c4)/(2*c3^2-8*c1*c5);
    R=sqrt((xm-Ox)^2);
    theta=rem(theta,360);
    m=tand(theta);
    if 90+delta < theta && theta < 270-delta
        a=c1+c3*m+c5*m*m;
        b=c2+c3*(-m*Ox+Oy)+2*c5*(-m*Ox+Oy)*m+c4*m;
        c=c4*(-m*Ox+Oy)+c5*(-m*Ox+Oy)^2+c6-L^2;
        alpha=(-b-realsqrt(b^2-4*a*c))/(2*a);
        beta=m*alpha+(-m*Ox+Oy);
    elseif 90-delta < theta && theta < 90+delta
        a=c5;
        b=c4+c3*Ox;
        c=c2*Ox+c1*Ox*Ox+c6-L^2;
        alpha=Ox;
        beta=(-b+realsqrt(b^2-4*a*c))/(2*a);
    elseif 270-delta < theta && theta < 270+delta
        a=c5;
        b=c4+c3*Ox;
        c=c2*Ox+c1*Ox*Ox+c6-L^2;
        alpha=Ox;
        beta=(-b-realsqrt(b^2-4*a*c))/(2*a);
    else
        a=c1+c3*m+c5*m*m;
        b=c2+c3*(-m*Ox+Oy)+2*c5*(-m*Ox+Oy)*m+c4*m;
        c=c4*(-m*Ox+Oy)+c5*(-m*Ox+Oy)^2+c6-L^2;
        alpha=(-b+realsqrt(b^2-4*a*c))/(2*a);
        beta=m*alpha+(-m*Ox+Oy);
    end
    %deltaTheta=20*rmin/(((alpha-Ox)^2+(beta-Oy)^2));
    deltaTheta=(rmean^2/((alpha-Ox)^2+(beta-Oy)^2))*(360/steps);
end