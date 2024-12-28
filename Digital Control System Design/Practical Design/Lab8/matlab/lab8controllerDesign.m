gain = 0.2375;
wd = 64.1136229;
zeta = 0.005;

wn = wd/sqrt(1-zeta^2);

G = tf(gain*wn^2, [1 2*zeta*wn wn^2]);

Gz = c2d(G,0.001);


% after control system designer
kloop = 3.15;
zeroC = 0.947;
kd = 48.56/400;

syms kp ki;
T=0.001;
e1 = kloop - kp+ki*T/2;
e2 = -zeroC - (ki*T-2*kp)/(2*kp+ki*T);
[ki,kp] = solve(e1,e2);
ki=double(ki);
kp=double(kp);

piController = kloop*tf([1 -zeroC],[1 -1], 0.001);
dController = kd*tf([400 -400],[1 -0.6],0.001);

controller = kp + ki*T/2*tf([1 1],[1 -1],0.001) - kd*tf([400 -400],[1 -0.6],0.001);