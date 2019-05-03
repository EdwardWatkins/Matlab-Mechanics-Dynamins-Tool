function [xxx,yyy,zzz,f] = generateShape(shape,x,y,z)
    [f,v] = isosurface(x,y,z,shape,0.5);
    xxx=v(:,1); yyy=v(:,2); zzz=v(:,3);
end

