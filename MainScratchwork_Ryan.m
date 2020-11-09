%% Problem variables
m_c = 1
m_b = 10
d_0 = 10
d_1 = 1
g = -9.81
F = (m_c + m_b)*10

%% LQR Dynamics
A = eye(8);
A(2,2) = 0;
A(8,8) = 0;
A(4,4) = F/(m_b+m_c);
A(6,6) = -d_1*m_c*F/(m_b*m_c*d_0^2);

B = zeros(8,2);
B(8,1) = 1;
B(2,2) =  (F-(m_b+m_c)*g)/(m_b+m_c);
B(6,1) = -d_0*m_b/(m_b*m_c*d_0^2);

R = [1 0; 0 0];
N = zeros(8,2);
Q = eye(8);

%% LQR iteration
iterMax = 1e6;
absTol = 1e-8
closeEnough = 0
iter = 0;

pLast = Q;
kLast = zeros(size(R,1),size(Q,1));
while iter < iterMax && closeEnough == 0
    pNow = Q + kLast'*R*kLast + (A - B*kLast)'*pLast*(A - B*kLast);
    kNow = (R + B'*pLast*B)*B'*pLast*A;
    if max(abs(kLast - kNow)) <= absTol
        closeEnough = 1;
    end
    pLast = pNow;
    kLast = kNow;
    iter = iter + 1
end 

kNow
pNow

%%
u = -kNow*x

