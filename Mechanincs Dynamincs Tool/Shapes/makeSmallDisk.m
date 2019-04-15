clear; clc; warning('off','MATLAB:datetime:InvalidSystemTimeZone');
s2=2.1; gs2=101;
[x2,y2,z2,xx2,yy2,zz2]=defineAxis(s2,gs2,s2,gs2,s2,gs2);

shape_2=vectorCylinder(0.5,[0,1.55,0],[0,1.65,0],x2,y2,z2);

[X2,Y2,Z2]=massCenter(shape_2,x2,y2,z2);
[f2,v2] = isosurface(xx2,yy2,zz2,shape_2,0.5);
xxx2=v2(:,1); yyy2=v2(:,2); zzz2=v2(:,3);