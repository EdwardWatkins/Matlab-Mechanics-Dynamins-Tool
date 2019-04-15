clear; clc; warning('off','MATLAB:datetime:InvalidSystemTimeZone');
shaft2s=3.6; shaft2gs=201;
[shaft2x,shaft2y,shaft2z,shaft2xx,shaft2yy,shaft2zz]=defineAxis(shaft2s,shaft2gs,shaft2s,shaft2gs,shaft2s,shaft2gs);

shaft2shape=cylinder(0.25,[0,0,0],7,shaft2x,shaft2y,shaft2z);

[shaft2X,shaft2Y,shaft2Z]=massCenter(shaft2shape,shaft2x,shaft2y,shaft2z);
[shaft2f,shaft2v] = isosurface(shaft2xx,shaft2yy,shaft2zz,shaft2shape,0.5);
shaft2xxx=shaft2v(:,1); shaft2yyy=shaft2v(:,2); shaft2zzz=shaft2v(:,3);