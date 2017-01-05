function [ irOut ] = irsensor( k, d )

irOut = zeros(1,size(d,2));

for i = 1 : size(d,2)
    irOut(i) = k(1)/d(i) + k(2);
end

end

