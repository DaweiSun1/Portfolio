T = 0.005;

deriv = c2d(tf([100 0], [1 100]), T, 'tustin');

A = [[0 1 0 0];[0 0 -52.0624 0];[0 0 0 1];[0 0 76.1771 0]];
B = [0;34.4136;0;-13.3529];
C = eye(4);
contSys = ss(A,B,C,0);
discSys = c2d(contSys,T);
Ad = discSys.A;
Bd = discSys.B;

sP = [-11,-10,-12,-14];
zP = exp(sP*T);

K = place(Ad,Bd,zP);

syms x [4 1];
K*x