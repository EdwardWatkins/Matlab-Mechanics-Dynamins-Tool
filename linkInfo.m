function [rmin,rmax,Xmax,Xmin,Ymax,Ymin,startAlpha,startBeta,perimiter,Ox,Oy,R] = linkInfo(v1,p1,v2,p2,L)
    c1=v1(1).^2+v1(2).^2+v1(3).^2;
    c2=2*p1(1)*v1(1)-2*p2(1)*v1(1)+2*p1(2)*v1(2)-2*p2(2)*v1(2)+2*p1(3)*v1(3)-2*p2(3)*v1(3);
    c3=-2*v2(1)*v1(1)-2*v2(2)*v1(2)-2*v2(3)*v1(3);
    c4=-2*p1(1)*v2(1)+2*p2(1)*v2(1)-2*p1(2)*v2(2)+2*p2(2)*v2(2)-2*p1(3)*v2(3)+2*p2(3)*v2(3);
    c5=v2(1).^2+v2(2).^2+v2(3).^2;
    c6=p1(1).^2+2*p2(1)*p1(1)+p2(1).^2+p1(2).^2-2*p2(2)*p1(2)+p2(2).^2+p1(3).^2-2*p2(3)*p1(3)+p2(3).^2;
    Ox=(-2*c4*c3+4*c5*c2)/(2*c3^2-8*c5*c1);
    Oy=(-2*c2*c3+4*c1*c4)/(2*c3^2-8*c1*c5);
    xm=((-2*c4*c3+4*c5*c2)+(sqrt((2*c4*c3-4*c5*c2)^2-(4*(c3^2-4*c5*c1)*(c4^2-4*c5*c6+4*c5*L^2)))))/(2*(c3^2-4*c5*c1));
    R=sqrt((xm-Ox)^2);
    i=1;    
    radius = zeros(3600);
    for theta = 0:0.1:360
        Theta=rem(theta,360);
        m=tand(Theta);
        if 90 < Theta && Theta < 270
            a=c1+c3*m+c5*m*m;
            b=c2+c3*(-m*Ox+Oy)+2*c5*(-m*Ox+Oy)*m+c4*m;
            c=c4*(-m*Ox+Oy)+c5*(-m*Ox+Oy)^2+c6-L^2;
            alpha=(-b-realsqrt(b^2-4*a*c))/(2*a);
            beta=m*alpha+(-m*Ox+Oy);
        elseif Theta == 90
            a=c5;
            b=c4+c3*Ox;
            c=c2*Ox+c1*Ox*Ox+c6-L^2;
            alpha=Ox;
            beta=(-b+realsqrt(b^2-4*a*c))/(2*a);
        elseif Theta == 270
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
        radius(i)=realsqrt((alpha-Ox)^2+(beta-Oy)^2);
        i=i+1;
    end
    rmin=min(radius(:));
    rmax=max(radius(:));
    a=(c3^2-4*c5*c1);
    b=(2*c2*c3-4*c1*c4);
    c=(c2^2-4*c1*c6+4*c1*L^2);
    Y(1)=(-(b)+real(sqrt((b)^2-4*(a)*(c))))/(2*(a));
    Y(2)=(-(b)-real(sqrt((b)^2-4*(a)*(c))))/(2*(a));
    Ymax=max(Y);
    Ymin=min(Y);
    a=(c3^2-4*c1*c5);
    b=(2*c4*c3-4*c5*c2);
    c=(c4^2-4*c5*c6+4*c5*L^2);
    X(1)=(-(b)+real(sqrt((b)^2-4*(a)*(c))))/(2*(a));
    X(2)=(-(b)-real(sqrt((b)^2-4*(a)*(c))))/(2*(a));
    Xmax=max(X);
    Xmin=min(X);
    a=c1;
    b=c2+c3*(Oy);
    c=c4*(+Oy)+c5*(+Oy)^2+c6-L^2;
    startAlpha=(-b+realsqrt(b^2-4*a*c))/(2*a);
    startBeta=Oy;
    perimiter=4*(a+b)*(pi/4)^((4*a*b)/((a+b)^2));
end