function shape=addShapes(shape1,shape2,sign)
    shape=shape1+shape2*sign;
    for i=1:numel(shape(:))
        if shape(i)==-1
            shape(i)=0;
        elseif shape(i)==2
            shape(i)=1;
        end
    end
end