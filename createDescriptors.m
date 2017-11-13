function retMat = createDescriptors(im, row, col, rad)    
    pad = zeros(2*rad + 1);
    pad(rad+1,rad+1) = 1;
    im = imfilter(im, pad, 'replicate', 'full');
    size = numel(row);
    retMat = zeros(size, (2*rad+1)^2);
    for i = 1:size
        horizontalRange = row(i):row(i)+2*rad;
        verticalRange   = col(i):col(i)+2*rad;
        neighbors = im(horizontalRange, verticalRange);
        retMat(i,:) = neighbors(:);
    end
end