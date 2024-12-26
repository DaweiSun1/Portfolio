A = [1 1; 0 1];
B = [0.125; 0.25];
C = [1 0];
D = [0];
zeta = 0.707;
tau = 4;
T = 1;

r = exp(-T/tau);

theta = sqrt((log(r)^2/zeta^2)-log(r)^2);

P = [r*cos(theta) + j*(r*sin(theta)); r*cos(theta)-j*(r*sin(theta))];
K = acker(A, B, P);

Anew = A - B*K;
Anew
Bnew = B;
Cnew = C;
Dnew = D; 
closed_loop_poles = eig(Anew);
closed_loop_poles
P
poly(Anew)
a1d = -sum(P);
a1d
a2d = P(1)*P(2);
a2d

O = transpose([C; C*A]);
O
rank(O)

Pnew = alpha * P;
%Pnew = [exp(-10*T); exp(-100*T)];
Pnew
G = transpose(acker(transpose(A), transpose(C), Pnew));
G
no = A - G*C;
no
new_A = [A -B*K; G*C (A-G*C-B*K)];
new_B = [B; B]; new_C = [C 0 0]; new_D = 0;
J = 1/(new_C * inv(eye(4) - new_A)*new_B);
J
eig(new_A)
new_B = J*new_B;
error_A = [(A - B*K) B*K; zeros(2) (A- G*C)];
error_B = [B*J; 0; 0]; error_C = [zeros(2) eye(2)]; error_D = 0;
eig(error_A)
new_A
new_B
new_C
error_A
error_B
error_C
[y1, x1] = dlsim(new_A, new_B, new_C, new_D, ones(60,1), [0 0 0 1]');
[y2, x2] = dlsim(error_A, error_B, error_C, error_D, ones(60,1), [0 0 0 1]');
t = [];
for i = 1:60
    t = [t; (i-1)*T];
end
[y3, x3] = dlsim(new_A, new_B, -[K zeros(1,2)], J, ones(60,1), [0 0 0 1]');
subplot(3,1,1), plot(t,y1,'r')
subplot(3,1,2), plot(t,y3,'r')
subplot(3,1,3), plot(t,y2(:,1),'r',t,y2(:,2),'g')

A11 = A(1,1);
A12 = A(1,2);
A21 = A(2,1);
A22 = A(2,2);
B1 = B(1);
B2 = B(2);
C1 = C(1);
A11
A12
A21
A22
B1
B2
C1

desired_observer_pole = exp(-10*T);
desired_observer_pole 
L = acker(A22', A12', desired_observer_pole);
L
A22-L*A12
-L*A12*L+A21-L*A11+A22*L
B2-L*B1
K
K1 = K(1);
K2 = K(2);
K1
K2
W1 = [
    A11-B1*K1-B1*K2*L;
    A21-B2*K1-B2*K2*L;
    A21-L*A12*L-L*A11+A22*L-(B2-L*B1)*(K1+K2*L)
];

W2 = [A12; A22; 0];

W3 = [-B1*K2; -B2*K2; (A22-L*A12-(B2-L*B1)*K2)];
W = [W1 W2 W3];
W
X = [B1; B2; B2-L*B1];
Y = [1 0 0];
Z = 0;
J1 = 1/(Y*inv(eye(3)-W)*X);
X=X*J1
J1
[y1,x1] = dlsim(W,X,Y,Z,ones(60,1),[0 0 1]');
[y2, x2] = dlsim(W,X, [-L eye(1) -eye(1)],Z,ones(60,1),[0 0 1]');
t = [];
for i = 1:60
    t = [t; (i-1)*T];
end
subplot(2,1,1), plot(t,y1)
subplot(2,1,2), plot(t,y2(:,1))