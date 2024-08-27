function x_ref = generateReference(x,x_des,h)
    x_ref = zeros(4, h);
    for i = 1 : 4
        x_ref(i,:) = linspace(x(i), x_des(i), h);
    end
end