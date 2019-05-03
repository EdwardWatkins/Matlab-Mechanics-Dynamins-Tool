function [mx my mz]=massCenter(shape,x,y,z)
    numberofcells=sum(shape(:));
    midcell=numberofcells/2;
    sumshapei=0;
    sumshapej=0;
    sumshapek=0;
    s=size(shape);
    for i=1:s(1)
        si=shape(i,:,:);
        sumshapei=sumshapei+sum(si(:));
        if sumshapei>=midcell
            mxi=i;
            break
        end
    end
    for j=1:s(2)
        sj=shape(:,j,:);
        sumshapej=sumshapej+sum(sj(:));
        if sumshapej>=midcell
            myj=j;
            break
        end
    end
    for k=1:s(3)
        sk=shape(:,:,k);
        sumshapek=sumshapek+sum(sk(:));
        if sumshapek>=midcell
            mzk=k;
            break
        end
    end
    my=x(mxi);
    mx=y(myj);
    mz=z(mzk);
end