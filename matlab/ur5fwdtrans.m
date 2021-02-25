function [ T06 ] = ur5fwdtrans( targetJoints, i )

    d = [0.089159 0 0 0.10915 0.09465 0.0823];
    a = [0 -0.425 -0.39225 0 0 0];
    alpha = [pi./2 0 0 pi./2 -pi./2 0];
    
    T = zeros(4, 4, 6);
%     T(:,:, 1) = DH(a(1), alpha(1), d(1), targetJoints(1));  %T01
%     T(:,:, 2) = DH(a(2), alpha(2), d(2), targetJoints(2));  %T12
%     T(:,:, 3) = DH(a(3), alpha(3), d(3), targetJoints(3));  %T23
%     T(:,:, 4) = DH(a(4), alpha(4), d(4), targetJoints(4));  %T34
%     T(:,:, 5) = DH(a(5), alpha(5), d(5), targetJoints(5));  %T45
%     T(:,:, 6) = DH(a(6), alpha(6), d(6), targetJoints(6));  %T56
    for k = 1:6
        T(:,:, k) = DH(a(k), alpha(k), d(k), targetJoints(k));
    end
    T06 = eye(4);
    if i > 0 && i <= 6
        for j = 1:i
            Tj = T(:,:, j)';
            T06 = T06*Tj;
        end
    end

end

