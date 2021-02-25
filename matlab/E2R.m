function R = E2R(alpha, beta, gamma)
R = [cos(alpha).*cos(beta), cos(alpha).*sin(gamma).*sin(beta)-sin(alpha).*cos(gamma), cos(alpha).*sin(beta).*cos(gamma)+sin(alpha).*sin(gamma);
     sin(alpha).*cos(beta), sin(alpha).*sin(beta).*sin(gamma)+cos(alpha).*cos(gamma), sin(alpha).*sin(beta).*cos(gamma)-cos(alpha).*sin(gamma);
   - sin(beta),             cos(beta).*sin(gamma),                                    cos(beta).*cos(gamma)];
end

