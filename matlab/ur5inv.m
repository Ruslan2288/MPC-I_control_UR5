function [ theta ] = ur5inv( T )

    
    %% DH parametrs
    tcp = [0 0 0.2];
    theta = zeros(6, 8);
    d = [0.089159 0 0 0.10915 0.09465 0.0823+tcp(3)];
    a = [0 -0.425 -0.39225 0 0 0];
    alpha = [pi./2 0 0 pi./2 -pi./2 0];

    %% Calculating theta1
    
    p05 = T * [0, 0, -d(6), 1]'  - [0, 0, 0, 1]';
    psi = atan2(p05(2), p05(1));
    phi = acos(d(4) ./ sqrt(p05(2).*p05(2) + p05(1).*p05(1)));
    theta(1, 1:4) = pi./2 + psi + phi;
    theta(1, 5:8) = pi./2 + psi - phi;
    theta = real(theta);
    
    
    %% Calculating theta5
    cols = [1, 5];
    for i=1:length(cols)
        c = cols(i);
        T10 = inv(DH(a(1), alpha(1), d(1), theta(1,c)));
        T16 = T10 * T;
        p16z = T16(3,4);
        t5 = acos((p16z-d(4))./d(6));
        theta(5, c:c+1) = t5;
        theta(5, c+2:c+3) = -t5;
    end
    theta = real(theta);
    
    %% Calculating theta6
    cols = [1, 3, 5, 7];
    for i=1:length(cols)
        c = cols(i);
        T01 = DH(a(1), alpha(1), d(1), theta(1,c));
        T61 = inv(T) * T01;
        T61zy = T61(2, 3);
        T61zx = T61(1, 3);
        t5 = theta(5, c);
        theta(6, c:c+1) = atan2(-T61zy./sin(t5), T61zx./sin(t5));
    end
    theta = real(theta);
    
    %% Calculating theta3
    cols = [1, 3, 5, 7];
    for i=1:length(cols)
        c = cols(i);
        T10 = inv(DH(a(1), alpha(1), d(1), theta(1,c)));
        T65 = inv(DH(a(6), alpha(6), d(6), theta(6,c)));
        T54 = inv(DH(a(5), alpha(5), d(5), theta(5,c)));
        T14 = T10 * T * T65 * T54;
        p13 = T14 * [0, -d(4), 0, 1]' - [0,0,0,1]';
        p13norm2 = norm(p13) .^ 2;
        t3p = acos((p13norm2-a(2).*a(2)-a(3)*a(3))./(2.*a(2).*a(3)));
        theta(3, c) = t3p;
        theta(3, c+1) = -t3p;
    end
    theta = real(theta);
    
    % Calculating theta2 and theta4
    cols = [1, 2, 3, 4, 5, 6, 7, 8];
    for i=1:length(cols)
        c = cols(i);
        T10 = inv(DH(a(1), alpha(1), d(1), theta(1,c)));
        T65 = inv(DH(a(6), alpha(6), d(6), theta(6,c)));
        T54 = inv(DH(a(5), alpha(5), d(5), theta(5,c)));
        T14 = T10 * T * T65 * T54;
        p13 = T14 * [0, -d(4), 0, 1]' - [0,0,0,1]';
        p13norm = norm(p13);
        theta(2, c) = -atan2(p13(2), -p13(1))+asin(a(3)*sin(theta(3,c))/p13norm);
        T32 = inv(DH(a(3), alpha(3), d(3), theta(3,c)));
        T21 = inv(DH(a(2), alpha(2), d(2), theta(2,c)));
        T34 = T32 * T21 * T14;
        theta(4, c) = atan2(T34(2,1), T34(1,1));
    end
    theta = real(theta);
    
    % Put theta at good range
    for i=1:6
        for j=1:8
            if theta(i,j) < 0
                theta(i,j) = theta(i,j) + 2.*pi;
            elseif theta(i,j) > 2.*pi
                theta(i,j) = theta(i,j) - 2.*pi;
            end
        end
    end
                
                    
end

