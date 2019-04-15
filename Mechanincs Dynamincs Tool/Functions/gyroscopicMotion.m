function [xxx,yyy,zzz,X,Y,Z,a,b,c,p1,p2,p3] = gyroscopicMotion(xxxyyyzzz,precession,nutation,spin,XYZ,abc,p1p2p3)
    xxx=xxxyyyzzz(:,1); yyy=xxxyyyzzz(:,2); zzz=xxxyyyzzz(:,3);
    X=XYZ(1); Y=XYZ(2); Z=XYZ(3);
    p1=p1p2p3(1); p2=p1p2p3(2); p3=p1p2p3(3);
    a=abc(1); b=abc(2); c=abc(3);
    % shape motion
    % precession
    [xxx,yyy,zzz]=rotateAboutVector([xxx,yyy,zzz],precession,[0,0,1],[0,0,0]);
    [X,Y,Z]=rotateAboutVector([X,Y,Z],precession,[0,0,1],[0,0,0]);
    [a,b,c]=rotateAboutVector([a,b,c],precession,[0,0,1],[0,0,0]);
    % nutation
    [xxx,yyy,zzz]=rotateAboutVector([xxx,yyy,zzz],nutation,[a,b,c],[0,0,0]);
    [X,Y,Z]=rotateAboutVector([X,Y,Z],nutation,[a,b,c],[0,0,0]);
    % spin
    [xxx,yyy,zzz]=rotateAboutVector([xxx,yyy,zzz],spin,[X,Y,Z],[0,0,0]);
    % point motion
    if exist('point')
        %[p1,p2,p3]=point;
        [p1,p2,p3]=rotateAboutVector([p1 p2 p3],precession,[0,0,1],[0,0,0]);
        [p1,p2,p3]=rotateAboutVector([p1 p2 p3],nutation,[a,b,c],[0,0,0]);
        [p1,p2,p3]=rotateAboutVector([p1 p2 p3],spin,[X,Y,Z],[0,0,0]);
    end
end