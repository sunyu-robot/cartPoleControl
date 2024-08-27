function dX = cartPoleDynamics(f, X)
global mc mp l g
    x = X(1);
    q = X(2);
    dx = X(3);
    dq = X(4);

    M = [mc+mp, mp*l*cos(q);
     mp*l*cos(q), mp*(l^2)];
    C = [0, -mp*l*sin(q)*dq;
         0, 0];
    tau = [0; mp*g*l*sin(q)];
    
    B =[1;0];
    a = M\(tau + B*f - C*X(3:4));
    ddx = a(1);
    ddq = a(2);

    dX = [dx; dq; ddx ; ddq];
end