function K = cartPoleLQR(Q, R)
global mc mp l g

A = [0 0 1 0;
    0 0 0 1;
    0 -mp*g/mc 0 0;
    0 (mp*g + mc*g)/(mc*l) 0 0];
B = [0;0;1/mc;-1/(mc*l)];
C = eye(4);
D = 0;

cartpole = ss(A,B,C,D);
K = lqr(cartpole, Q, R);
% f = K*(qd'-q);
end