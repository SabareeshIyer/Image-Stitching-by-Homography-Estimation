function dif = finddif (A, B)
    size1 = numel(A(:,1));
    for i=1:size1
        x = A(i,1) - B(i,1);
        y = A(i,2) - B(i,2);
        z = x^2 + y^2;
        dif(i) = sqrt(z);
    end    
end