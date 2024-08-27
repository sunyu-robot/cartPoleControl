function f = cartPoleLMPC(Q, R,x_k, xd, N, step)
    global mc mp l g

    A = [0 0 1 0;
    0 0 0 1;
    0 -mp*g/mc 0 0;
    0 (mp*g + mc*g)/(mc*l) 0 0]*step + eye(4);
    B = [0;0;1/mc;-1/(mc*l)]*step;
    
    X_ref = generateReference(x_k,xd',N);
    X_ref_vec = [x_k;reshape(X_ref, [], 1)];
    
    F =Q;
    n = size(A,1); 
    p = size(B,2); 
    M = [eye(n);zeros(N*n,n)]; 
    C = zeros((N+1)*n,N*p); 
    tmp = eye(n); 
    for i = 1:N
        rows = i*n + (1:n);
        C(rows, :) = [tmp*B, C(rows-n, 1:end-p)];
        tmp = A*tmp;
        M(rows,:) = tmp;
    end
    S_q = size(Q,1);
    S_r = size(R,1);
    Q_bar = zeros((N+1)*S_q,(N+1)*S_q);
    R_bar = zeros(N*S_r,N*S_r);
    for i = 0:N-1
        Q_bar(i*S_q+1:(i+1)*S_q,i*S_q+1:(i+1)*S_q) = Q;
    end
    Q_bar(N*S_q+1:(N+1)*S_q, N*S_q+1:(N+1)*S_q) = F;
    for i = 0:N-1
        R_bar(i*S_r+1:(i+1)*S_r, i*S_r+1:(i+1)*S_r) = R;
    end
%     G = M'*Q_bar*M;
%     E = M'*Q_bar*C;
    H = C'*Q_bar*C + R_bar;

    f = (x_k'*M' - X_ref_vec')*Q_bar*C;
    % constraint
    lb = zeros(N*S_r, 1); ub = zeros(N*S_r, 1);
    for i = 0:N-1
        lb(i+1) = -40;
        ub(i+1) = 40;
    end
    f_all = quadprog(H,f', [], [],[],[], lb, ub); 
    f = f_all(1);
end