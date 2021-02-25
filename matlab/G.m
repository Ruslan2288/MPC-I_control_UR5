function [ T ] = G( R, P )
    T = zeros(4);
    T(1:3, 1:3) = R;
    T(1, 4) = P(1);
    T(2, 4) = P(2);
    T(3, 4) = P(3);
    T(4, 4) = 1;
end

