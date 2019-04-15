function shape = vectorCylinder(radius,centre,vector,x,y,z)
    shape = zeros(numel(x),numel(y),numel(z));
    x1 = [centre(1),centre(2),centre(3)];
    x2 = [vector(1),vector(2),vector(3)];
    b = sqrt(sum(power(abs(x2-x1),2)));
    for i = 1:numel(x)
        for j = 1:numel(y)
            for k = 1:numel(z)
                % New point
                x0 = [x(i),y(j),z(k)];
                % Find closest point on line -
                t=-(dot((x1-x0),(x2-x1)))/(power(vecnorm(x2-x1),2));
                tx=x1(1)+(x2(1)-x1(1))*(t);
                ty=x1(2)+(x2(2)-x1(2))*(t);
                tz=x1(3)+(x2(3)-x1(3))*(t);
                if (tx <= x1(1) && tx >= x2(1)) || (tx <= x2(1) && tx >= x1(1)) % within upper and lower limits
                    if (ty <= x1(2) && ty >= x2(2)) || (ty <= x2(2) && ty >= x1(2))
                        if (tz <= x1(3) && tz >= x2(3)) || (tz <= x2(3) && tz >= x1(3))
                            % find distance to the line -
                            a=sqrt(sum(power(abs(x1-x0),2)));
                            c=sqrt(sum(power(abs(x2-x0),2)));
                            s=(a+b+c)/2;
                            h=(2/b)*sqrt(s*(s-a)*(s-b)*(s-c));
                            if  h <= radius % within radius then
                                shape(j,i,k) = 1;
                            end
                        end
                    end
                end
            end
        end
    end
end