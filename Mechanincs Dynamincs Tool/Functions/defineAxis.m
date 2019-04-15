function [x,y,z]=defineAxis(scale,points)
    x=linspace(-scale,scale,points);
    y=linspace(-scale,scale,points);
    z=linspace(-scale,scale,points);
end