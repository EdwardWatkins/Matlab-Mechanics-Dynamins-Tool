function [xxx,yyy,zzz] = positionAlongVector(M,p1,v1)
    xxx=M(:,1);
    yyy=M(:,2);
    zzz=M(:,3);
    % Turn through v1
    v2=cross([0,0,1],v1/norm(v1));
    theta=atan2d(norm(cross([0,0,1],v1)),dot([0,0,1],v1));
    if theta ~= 0
        [xxx,yyy,zzz]=rotateAboutVector([xxx,yyy,zzz],theta,v2,[0,0,0]);
    end
    % Displace by p1
    M(:,1)=xxx;
    M(:,2)=yyy;
    M(:,3)=zzz;
    M=M+p1;
    xxx=M(:,1);
    yyy=M(:,2);
    zzz=M(:,3);
end