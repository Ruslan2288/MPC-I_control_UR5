function [ g ] = transformation( theta, pos )

    g = eye(4); % initialize the G matrix with identity
    g(1:3, 1:3) = eulerzyx(theta);
    g(1, 4) = pos(1);
    g(2, 4) = pos(2);
    g(3, 4) = pos(3);
end

