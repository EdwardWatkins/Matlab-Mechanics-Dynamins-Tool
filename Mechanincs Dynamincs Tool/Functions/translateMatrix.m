function [x,y,z] = translateMatrix(Matrix,xdiff,ydiff,zdiff)
    Matrix=Matrix+[xdiff,ydiff,zdiff];
    x=Matrix(:,1);
    y=Matrix(:,2);
    z=Matrix(:,3);
end