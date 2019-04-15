clear; clc; warning('off','MATLAB:datetime:InvalidSystemTimeZone');
rods=3.6; rodgs=101;
[rodx,rody,rodz,rodxx,rodyy,rodzz]=defineAxis(rods,rodgs,rods,rodgs,rods,rodgs);

rodshape=cylinder(0.25,[0,0,0],7,rodx,rody,rodz);

[rodX,rodY,rodZ]=massCenter(rodshape,rodx,rody,rodz);
[rodf,rodv] = isosurface(rodxx,rodyy,rodzz,rodshape,0.5);
rodxxx=rodv(:,1); rodyyy=rodv(:,2); rodzzz=rodv(:,3);