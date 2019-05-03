clear
clc
close all
addpath Functions
[x,y,z]=defineAxis(0.15,401);
s=sphere(0.03,[0,-0.07071,0.07071],x,y,z);
U=[0.1,0,0
    0.1,-0.14,0
    0.1,0,0.14
    -0.1,0,0
    -0.1,-0.14,0
    -0.1,0,0.14];
p=polyhedron(U,x,y,z);
c=cone(0.03,[0,0,0],[0,-0.07071,0.07071],x,y,z);
shape=addShapes(s,p,-1);
shape=addShapes(shape,c,1);
[xx,yy,zz,f]=generateShape(shape,x,y,z);
figure; hold on
plot3([0,0.1],[0,0],[0,0],'k--')
plot3([0,0],[0,0.1],[0,0],'k--')
plot3([0,0],[0,0],[0,0.1],'k--')
patch('Faces',f,'Vertices',[xx,yy,zz],'EdgeColor','none','FaceColor','blue');
camlight; axis vis3d; grid on; axis([-0.15 0.15 -0.15 0.15 -0.15 0.15]);daspect([1 1 1]); view([1 1 1])