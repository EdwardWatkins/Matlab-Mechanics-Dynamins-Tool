function shape = polyhedron(U,x,y,z)
    shape = zeros(numel(x),numel(y),numel(z));
    f = convhull(U);
    sf = size(f);
    meanPoint = [mean(U(:,1)),mean(U(:,2)),mean(U(:,3))];
    volume = 0;
    for i = 1:sf(1)
        bdcd(i,:) = cross((U(f(i,2),:)-U(f(i,1),:)),(U(f(i,3),:)-U(f(i,1),:)));
        volume = volume + abs(dot((meanPoint-U(f(i,1),:)),bdcd(i,:)))/6;
    end
    for i = 1:numel(x)
        for j = 1:numel(y)
            for k = 1:numel(z)
                s = 0;
                q = 1;
                testVolume = 0;
                while testVolume <= volume
                    testVolume = testVolume + abs(dot((([x(i),y(j),z(k)])-U(f(q,1),:)),bdcd(q,:)))/6;
                    if q == sf(1) && testVolume <= volume*1.01 % allow for 1% rounding error
                        s = 1;
                        break
                    end
                    q = q + 1;
                end
                shape(j,i,k) = s;
            end
        end
    end
end