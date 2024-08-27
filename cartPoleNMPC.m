function f = cartPoleNMPC(Q, R, q0, qd, p_h, h)

    nmpc = casadi.Opti();
    opts = struct('ipopt', struct('print_level', 0, 'warm_start_init_point', 'yes', 'max_iter', 100));
    nmpc.solver('ipopt', opts);
   X_ref = generateReference(q0,qd',p_h);
   X = nmpc.variable(4,p_h); 
    F = nmpc.variable(1,p_h); 
   nmpc.subject_to ( X(:,1) == q0 );
   for k = 1 : p_h-1
        % dynamics:
        dX = cartPoleDynamics(F(:,k), X(:,k));

        nmpc.subject_to( X(:, k+1) == X(:, k) + h*dX );
        nmpc.subject_to( -40 <= F(:,k) <= 40);
   end
   
    J = 0;
    % cost function
    for k = 1 : p_h
        J = J + (X_ref(:,k) - X(:,k))' * Q ... 
            * (X_ref(:,k) - X(:,k)) + R * F(:,k)^2;
    end
    nmpc.minimize(J);
    solution = nmpc.solve();
    f_all = solution.value(F);
    f = f_all(1);
end