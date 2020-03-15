function [log] = mpc_solver(p_0, max_v, min_v, max_a, min_a, max_j, min_j, Target_p, no_step)
v_0 = 0;
a_0 = 0;
j_0 = 0;
K = 20;
dt = 0.2;
log = [0 p_0 v_0 a_0 j_0];
w1 = 10;
w2 = 1;
w3 = 1;
w4 = 1;
w5 = 1e2;
planing_time = dt*(no_step-1);
count = 1;
for t = 0.2:0.2:planing_time
    %% Construct the prediction matrix
    [Tp, Tv, Ta, Bp, Bv, Ba] = getPredictionMatrix(K, dt, p_0, v_0, a_0);
    
    %% Target_ p - Tp should be 0
    % Target_p_20: further 20 step target position
    Target_point = Target_p(count:count+K-1);
    %%Construct the optimization problem
    H = blkdiag(w1*(Tp'*Tp)+w4*eye(K), w5*eye(K));
    F = [2*w1*(Bp'*Tp-Target_point*Tp), zeros(1,K)];
    
    A = [ Tv zeros(K); -Tv -eye(K); Ta zeros(K); -Ta -eye(K); eye(K) zeros(K); -eye(K) -eye(K); zeros(size(Ta)) -eye(K)];
    b = [ max_v*ones(K,1)-Bv; min_v*ones(K,1)+Bv; max_a*ones(K,1)-Ba; min_a*ones(K,1)+Ba; max_j*ones(K,1); min_j*ones(K,1); zeros(K,1)];
    %% Solve the optimization problem
    J = quadprog(H, F, A, b);
    
    %% Solve the optimization problem
    j = J(1);
    p_0 = p_0 + v_0 * dt + 0.5 * a_0 * dt ^ 2 + 1/6 * j * dt ^ 3;
    v_0 = v_0 + a_0 * dt + 0.5 * j * dt ^ 2;
    a_0 = a_0 + j * dt;
    j_0 = j;
    %% Log the states
    log = [log; t p_0 v_0 a_0 j_0];
    count = count +1;
end