clear; clc; warning('off','MATLAB:datetime:InvalidSystemTimeZone');
s1=2.1; gs1=101;
[x1,y1,z1,xx1,yy1,zz1]=defineAxis(s1,gs1,s1,gs1,s1,gs1);

shape1_1=cylinder(2,[0,0,-0.05],0.1,x1,y1,z1);
shape2_1=polyhedron([1.5,-0.1,0.55;1.5,-0.1,-0.55;1.5,0.1,0.55;1.5,0.1,-0.55;1.7,-0.1,0.55;1.7,-0.1,-0.55;1.7,0.1,0.55;1.7,0.1,-0.55],x1,y1,z1,xx1,yy1,zz1);
shape_1=addShapes(shape1_1,shape2_1,-1);

[X1,Y1,Z1]=massCenter(shape_1,x1,y1,z1);
[f1,v1] = isosurface(xx1,yy1,zz1,shape_1,0.5);
xxx1=v1(:,1); yyy1=v1(:,2); zzz1=v1(:,3);