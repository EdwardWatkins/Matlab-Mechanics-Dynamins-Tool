clear; clc; warning('off','MATLAB:datetime:InvalidSystemTimeZone');
link1s=3.6; link1gs=101;
[link1x,link1y,link1z,link1xx,link1yy,link1zz]=defineAxis(link1s,link1gs,link1s,link1gs,link1s,link1gs);

link1shape1=cylinder(0.35,[0,0,0],1,link1x,link1y,link1z);
link1shape2=cylinder(0.25,[0,0,0],1,link1x,link1y,link1z);
link1shape=addShapes(link1shape1,link1shape2,-1);

[link1X,link1Y,link1Z]=massCenter(link1shape,link1x,link1y,link1z);
[link1f,link1v] = isosurface(link1xx,link1yy,link1zz,link1shape,0.5);
link1xxx=link1v(:,1); link1yyy=link1v(:,2); link1zzz=link1v(:,3);