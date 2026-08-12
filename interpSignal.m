function Yq = interpSignal(x,Y,xq)

nCol = size(Y,2);

Yq = zeros(length(xq),nCol);

for c = 1:nCol

    Yq(:,c) = interp1( ...
        x,...
        Y(:,c),...
        xq,...
        'linear');

end

end